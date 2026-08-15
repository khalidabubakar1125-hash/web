#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Relative-state estimation and True Proportional Navigation guidance engine.

Target platform
---------------
    Compute      : NVIDIA Jetson Orin Nano (Ubuntu 22.04 LTS, JetPack 6.x)
    Middleware   : ROS 2 (rclpy, Humble)
    Sensor       : Arducam Global Shutter MIPI CSI-2 module, 1080p @ 60 Hz
    Autopilot    : Pixhawk / Cube Orange over UART @ 921600 baud (pymavlink)

Signal chain
------------
    [MIPI camera 60 Hz] -> [detector / depth solver] -> geometry_msgs/PointStamped
        -> [polar conversion] -> [latency-compensated EKF] -> [3-D True ProNav]
        -> [saturation] -> [SET_POSITION_TARGET_LOCAL_NED / MAV_FRAME_BODY_NED]

Frames
------
    CAMERA (optical) : +x right, +y down, +z forward (REP-103 optical frame)
    BODY   (FRD)     : +x forward, +y right, +z down

    All estimation and guidance is carried out in the BODY FRD frame, which is
    the native frame of MAV_FRAME_BODY_NED velocity setpoints. Because BODY is a
    rotating frame, the filter propagation carries the transport (Coriolis)
    terms -omega x r and -omega x v, with omega taken from the autopilot
    ATTITUDE stream. This keeps the track stationary in the estimator while the
    airframe manoeuvres underneath it.

State vector (interleaved, per the ICD)
---------------------------------------
    x = [x, vx, y, vy, z, vz]^T   relative target position / velocity, BODY FRD

Measurement vector
------------------
    z = [azimuth, elevation, range]^T

    The sensor is a camera: bearing accuracy is set by pixel pitch and is
    excellent, while range accuracy degrades with the square of distance. A
    Cartesian measurement model would smear that anisotropy into an isotropic
    blob, so the update is performed in polar space with an analytic Jacobian --
    this is the source of the filter's "extended" nonlinearity.
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PointStamped, TwistStamped

from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Physical and protocol constants
# ---------------------------------------------------------------------------

G0 = 9.80665                      # Standard gravity [m/s^2]
MPH_TO_MPS = 0.44704              # Miles per hour -> metres per second

MAX_ACCEL_MPS2 = 6.0 * G0         # 6 G structural limit  -> 58.8399 m/s^2
MAX_SPEED_MPS = 120.0 * MPH_TO_MPS  # 120 mph ceiling      -> 53.6448 m/s

# SET_POSITION_TARGET_LOCAL_NED type_mask bit definitions. A set bit means
# "ignore this field", so an active field is expressed by leaving its bit clear.
TM_IGNORE_PX, TM_IGNORE_PY, TM_IGNORE_PZ = 1 << 0, 1 << 1, 1 << 2
TM_IGNORE_VX, TM_IGNORE_VY, TM_IGNORE_VZ = 1 << 3, 1 << 4, 1 << 5
TM_IGNORE_AX, TM_IGNORE_AY, TM_IGNORE_AZ = 1 << 6, 1 << 7, 1 << 8
TM_FORCE_SETPOINT = 1 << 9
TM_IGNORE_YAW = 1 << 10
TM_IGNORE_YAW_RATE = 1 << 11

# Command velocity + yaw-rate; ignore position, acceleration and absolute yaw.
TYPE_MASK_VELOCITY_YAWRATE = (
    TM_IGNORE_PX | TM_IGNORE_PY | TM_IGNORE_PZ
    | TM_IGNORE_AX | TM_IGNORE_AY | TM_IGNORE_AZ
    | TM_IGNORE_YAW
)  # == 1479

# Rotation from the REP-103 optical camera frame into BODY FRD.
#   body_x (fwd)   <-  cam_z (fwd)
#   body_y (right) <-  cam_x (right)
#   body_z (down)  <-  cam_y (down)
R_CAM_TO_BODY = np.array(
    [[0.0, 0.0, 1.0],
     [1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0]],
    dtype=np.float64,
)

# Interleaved state indices: [x, vx, y, vy, z, vz]
IDX_POS = [0, 2, 4]
IDX_VEL = [1, 3, 5]

EPS = 1e-9


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class SensorConfig:
    """Camera pipeline timing and noise characterisation."""

    frame_rate_hz: float = 60.0
    # Fixed hardware pipeline latency, expressed in frames. ISP + CSI-2 DMA +
    # detector inference land the sample two frames behind real time.
    pipeline_latency_frames: float = 2.0

    # Bearing noise: 1-sigma angular error [rad]. Roughly one pixel at 1080p
    # over a ~70 deg horizontal FOV (~0.63 mrad/px) with a small margin.
    sigma_bearing_rad: float = 1.2e-3

    # Range noise model: sigma_r = base + slope * r^2. The quadratic term is the
    # depth-error growth of a fixed-baseline triangulation solver,
    # dZ = Z^2 * d_disp / (f * B); at f ~ 1370 px, B = 0.10 m and a quarter-pixel
    # disparity error that slope is ~1.8e-3 m^-1.
    sigma_range_base_m: float = 0.05
    sigma_range_quad: float = 0.0018

    # Reject a frame outright if the reported range falls outside these bounds.
    # Beyond the upper bound the quadratic term makes range effectively
    # unobservable; inside it the filter simply de-weights range and coasts on
    # the bearing channel, which is the correct behaviour rather than a fault.
    min_range_m: float = 0.35
    max_range_m: float = 150.0

    @property
    def pipeline_latency_s(self) -> float:
        return self.pipeline_latency_frames / max(self.frame_rate_hz, EPS)


@dataclass
class FilterConfig:
    """EKF tuning."""

    # Continuous-time white-noise-acceleration spectral density [m^2/s^5].
    # Sized for a non-cooperative target capable of aggressive manoeuvres.
    process_noise_psd: float = 9.0

    # Initial covariance seeds for an unobserved track.
    init_pos_var: float = 25.0
    init_vel_var: float = 100.0

    # Ring-buffer depth for retrodiction. Must comfortably span the pipeline
    # latency at the control rate; 64 epochs is >1 s of history at 60 Hz.
    history_depth: int = 64

    # Mahalanobis gate on the 3-DOF innovation (chi-square, p = 0.999).
    innovation_gate_chi2: float = 16.266

    # Consecutive gate rejections tolerated before the track is reinitialised
    # from the raw measurement -- protects against a diverged, over-confident
    # filter permanently locking out valid data.
    max_consecutive_rejects: int = 8

    # Track is declared stale (and guidance inhibited) after this quiet period.
    track_timeout_s: float = 0.25


@dataclass
class GuidanceConfig:
    """True Proportional Navigation and command-shaping parameters."""

    nav_constant: float = 4.0            # N
    max_accel_mps2: float = MAX_ACCEL_MPS2
    max_speed_mps: float = MAX_SPEED_MPS

    # Floor applied to the closing speed used as the ProNav gain. The law scales
    # with closure, so at zero or negative closure (station-keeping, or a target
    # opening the range) the lateral channel would collapse or invert. Flooring
    # it keeps the command steering toward the target's angular drift.
    min_effective_closure_mps: float = 1.0

    # Maximum closure along the LOS [m/s]. Positive closes range.
    closure_speed_mps: float = 4.0
    # Standoff range held by the closure term [m].
    standoff_range_m: float = 3.0
    # Proportional gain on range error feeding the LOS-axial closure term [1/s].
    closure_gain: float = 0.8
    # Damping gain converting a closure-rate error into axial acceleration [1/s].
    closure_damping: float = 2.0

    # First-order washout on the velocity integrator [s]. Bleeds the command
    # back toward zero if guidance stops being refreshed, preventing wind-up.
    command_washout_tau_s: float = 1.5

    # Yaw-rate servo that keeps the target inside the camera FOV.
    yaw_rate_gain: float = 1.6           # [1/s] on azimuth error
    max_yaw_rate_rps: float = 1.2        # [rad/s]


@dataclass
class LinkConfig:
    """MAVLink transport settings."""

    device: str = "/dev/ttyTHS1"
    baud: int = 921600
    source_system: int = 1
    source_component: int = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
    target_system: int = 1
    target_component: int = 1
    heartbeat_hz: float = 2.0


@dataclass
class EngineConfig:
    """Top-level engine configuration."""

    sensor: SensorConfig = field(default_factory=SensorConfig)
    filt: FilterConfig = field(default_factory=FilterConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    link: LinkConfig = field(default_factory=LinkConfig)

    control_rate_hz: float = 60.0
    measurement_topic: str = "/perception/target_point"
    command_echo_topic: str = "/gnc/velocity_command"


# ---------------------------------------------------------------------------
# Linear-algebra helpers
# ---------------------------------------------------------------------------

def wrap_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix such that skew(a) @ b == cross(a, b)."""
    return np.array(
        [[0.0, -v[2], v[1]],
         [v[2], 0.0, -v[0]],
         [-v[1], v[0], 0.0]],
        dtype=np.float64,
    )


def saturate_norm(vec: np.ndarray, limit: float) -> np.ndarray:
    """Scale a vector so its 2-norm never exceeds ``limit``, preserving direction.

    Direction preservation matters: per-axis clipping would rotate the commanded
    acceleration away from the ProNav solution exactly when the manoeuvre is
    most demanding.
    """
    norm = float(np.linalg.norm(vec))
    if norm <= limit or norm < EPS:
        return vec
    return vec * (limit / norm)


def enforce_symmetry(P: np.ndarray) -> np.ndarray:
    """Re-symmetrise a covariance matrix to suppress round-off asymmetry."""
    return 0.5 * (P + P.T)


def cartesian_to_polar(p: np.ndarray) -> np.ndarray:
    """BODY FRD Cartesian position -> [azimuth, elevation, range].

    Azimuth is measured about the down axis (positive to starboard) and
    elevation positive upward, matching the aerospace convention for FRD.
    """
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    ground = math.hypot(x, y)
    rng = math.sqrt(x * x + y * y + z * z)
    az = math.atan2(y, x)
    el = math.atan2(-z, max(ground, EPS))
    return np.array([az, el, rng], dtype=np.float64)


def polar_to_cartesian(az: float, el: float, rng: float) -> np.ndarray:
    """Inverse of :func:`cartesian_to_polar`."""
    ce = math.cos(el)
    return np.array(
        [rng * ce * math.cos(az),
         rng * ce * math.sin(az),
         -rng * math.sin(el)],
        dtype=np.float64,
    )


# ---------------------------------------------------------------------------
# Measurement container
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Measurement:
    """A single latency-corrected sensor observation."""

    capture_time: float          # Epoch of light hitting the sensor [s]
    polar: np.ndarray            # [az, el, range]
    R: np.ndarray                # 3x3 measurement covariance


# ---------------------------------------------------------------------------
# Latency-compensated Extended Kalman Filter
# ---------------------------------------------------------------------------

@dataclass
class FilterEpoch:
    """One entry of the retrodiction ring buffer.

    Stores the *posterior* at ``timestamp`` together with everything needed to
    reconstruct the step that produced it, so the trajectory can be replayed
    bit-for-bit after a delayed sample is spliced into the past.
    """

    timestamp: float
    state: np.ndarray
    cov: np.ndarray
    body_rate: np.ndarray
    measurement: Optional[Measurement]


class LatencyCompensatedEKF:
    """6-state relative kinematic tracker with out-of-sequence measurement fusion.

    The camera pipeline delivers samples two frames stale. Fusing them at the
    arrival epoch would inject a systematic lag proportional to closing speed --
    at 50 m/s that is a 1.7 m position bias, which ProNav converts directly into
    a steady-state miss. Instead the filter keeps a ring buffer of posteriors,
    rolls back to the epoch that brackets the true capture time, injects the
    measurement there, and replays every subsequent step (including any
    measurements already fused) forward to the current epoch.
    """

    DIM = 6

    def __init__(self, config: FilterConfig, logger: logging.Logger) -> None:
        self._cfg = config
        self._log = logger
        self._lock = threading.RLock()

        self._x = np.zeros(self.DIM, dtype=np.float64)
        self._P = np.eye(self.DIM, dtype=np.float64)
        self._epoch = 0.0
        self._initialised = False
        self._reject_streak = 0
        self._last_measurement_time = -math.inf

        self._history: Deque[FilterEpoch] = deque(maxlen=config.history_depth)

    # -- Introspection ------------------------------------------------------

    @property
    def initialised(self) -> bool:
        with self._lock:
            return self._initialised

    def snapshot(self) -> Tuple[np.ndarray, np.ndarray, float]:
        """Thread-safe copy of (state, covariance, epoch)."""
        with self._lock:
            return self._x.copy(), self._P.copy(), self._epoch

    def relative_position(self) -> np.ndarray:
        with self._lock:
            return self._x[IDX_POS].copy()

    def relative_velocity(self) -> np.ndarray:
        with self._lock:
            return self._x[IDX_VEL].copy()

    def is_track_fresh(self, now: float) -> bool:
        with self._lock:
            return (
                self._initialised
                and (now - self._last_measurement_time) <= self._cfg.track_timeout_s
            )

    # -- Lifecycle ----------------------------------------------------------

    def reset(self) -> None:
        """Drop the track and clear all retrodiction history."""
        with self._lock:
            self._x = np.zeros(self.DIM, dtype=np.float64)
            self._P = np.eye(self.DIM, dtype=np.float64)
            self._initialised = False
            self._reject_streak = 0
            self._last_measurement_time = -math.inf
            self._history.clear()

    def _initialise_from(self, meas: Measurement) -> None:
        """Seed the track directly from a single observation."""
        az, el, rng = meas.polar
        pos = polar_to_cartesian(az, el, rng)

        self._x = np.zeros(self.DIM, dtype=np.float64)
        self._x[IDX_POS] = pos

        # Map the polar measurement covariance into Cartesian position variance
        # through the local Jacobian, then inflate: a single frame carries no
        # velocity information whatsoever.
        J = self._polar_jacobian_position(pos)
        try:
            J_inv = np.linalg.inv(J)
            pos_cov = J_inv @ meas.R @ J_inv.T
        except np.linalg.LinAlgError:
            pos_cov = np.eye(3) * self._cfg.init_pos_var

        self._P = np.zeros((self.DIM, self.DIM), dtype=np.float64)
        self._P[np.ix_(IDX_POS, IDX_POS)] = enforce_symmetry(pos_cov) + np.eye(3) * 1e-3
        self._P[np.ix_(IDX_VEL, IDX_VEL)] = np.eye(3) * self._cfg.init_vel_var

        self._epoch = meas.capture_time
        self._last_measurement_time = meas.capture_time
        self._initialised = True
        self._reject_streak = 0
        self._history.clear()
        self._push_history(np.zeros(3), meas)
        self._log.info("Track initialised at range %.2f m", rng)

    # -- Models -------------------------------------------------------------

    @staticmethod
    def _continuous_dynamics(body_rate: np.ndarray) -> np.ndarray:
        """Continuous plant matrix A for relative kinematics in a rotating frame.

            d/dt r = v - omega x r
            d/dt v =   - omega x v      (target acceleration -> process noise)
        """
        Omega = skew(body_rate)
        A = np.zeros((LatencyCompensatedEKF.DIM, LatencyCompensatedEKF.DIM))
        A[np.ix_(IDX_POS, IDX_POS)] = -Omega
        A[np.ix_(IDX_POS, IDX_VEL)] = np.eye(3)
        A[np.ix_(IDX_VEL, IDX_VEL)] = -Omega
        return A

    @classmethod
    def _state_transition(cls, body_rate: np.ndarray, dt: float) -> np.ndarray:
        """Discrete transition via a truncated matrix exponential.

        A third-order series is exact to well below sensor noise at dt ~ 16 ms
        and body rates below a few rad/s, while costing a fraction of a full
        ``expm`` on the Orin's CPU.
        """
        A = cls._continuous_dynamics(body_rate)
        Adt = A * dt
        Adt2 = Adt @ Adt
        Adt3 = Adt2 @ Adt
        return np.eye(cls.DIM) + Adt + 0.5 * Adt2 + (1.0 / 6.0) * Adt3

    def _process_noise(self, dt: float) -> np.ndarray:
        """Discretised continuous white-noise-acceleration covariance."""
        q = self._cfg.process_noise_psd
        dt2 = dt * dt
        dt3 = dt2 * dt

        Q = np.zeros((self.DIM, self.DIM), dtype=np.float64)
        I3 = np.eye(3)
        Q[np.ix_(IDX_POS, IDX_POS)] = I3 * (q * dt3 / 3.0)
        Q[np.ix_(IDX_POS, IDX_VEL)] = I3 * (q * dt2 / 2.0)
        Q[np.ix_(IDX_VEL, IDX_POS)] = I3 * (q * dt2 / 2.0)
        Q[np.ix_(IDX_VEL, IDX_VEL)] = I3 * (q * dt)
        return Q

    @staticmethod
    def _polar_jacobian_position(pos: np.ndarray) -> np.ndarray:
        """d[az, el, range] / d[x, y, z] evaluated at ``pos`` (BODY FRD)."""
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        ground_sq = max(x * x + y * y, EPS)
        ground = math.sqrt(ground_sq)
        r_sq = max(ground_sq + z * z, EPS)
        r = math.sqrt(r_sq)

        J = np.zeros((3, 3), dtype=np.float64)
        # Azimuth = atan2(y, x)
        J[0, 0] = -y / ground_sq
        J[0, 1] = x / ground_sq
        J[0, 2] = 0.0
        # Elevation = atan2(-z, ground)
        J[1, 0] = (z * x) / (ground * r_sq)
        J[1, 1] = (z * y) / (ground * r_sq)
        J[1, 2] = -ground / r_sq
        # Range
        J[2, 0] = x / r
        J[2, 1] = y / r
        J[2, 2] = z / r
        return J

    def _measurement_jacobian(self, state: np.ndarray) -> np.ndarray:
        """Full 3x6 observation Jacobian H (velocity states are unobserved)."""
        H = np.zeros((3, self.DIM), dtype=np.float64)
        H[:, IDX_POS] = self._polar_jacobian_position(state[IDX_POS])
        return H

    # -- Core steps ---------------------------------------------------------

    def _predict_inplace(
        self,
        state: np.ndarray,
        cov: np.ndarray,
        body_rate: np.ndarray,
        dt: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Single prediction step; returns fresh (state, covariance)."""
        if dt <= 0.0:
            return state, cov
        F = self._state_transition(body_rate, dt)
        new_state = F @ state
        new_cov = enforce_symmetry(F @ cov @ F.T + self._process_noise(dt))
        return new_state, new_cov

    def _update_inplace(
        self,
        state: np.ndarray,
        cov: np.ndarray,
        meas: Measurement,
        apply_gate: bool,
    ) -> Tuple[np.ndarray, np.ndarray, bool]:
        """Joseph-form EKF update in polar measurement space.

        Returns (state, covariance, accepted).
        """
        H = self._measurement_jacobian(state)
        predicted = cartesian_to_polar(state[IDX_POS])

        innovation = meas.polar - predicted
        innovation[0] = wrap_pi(innovation[0])   # azimuth wraps
        innovation[1] = wrap_pi(innovation[1])   # elevation wraps

        S = enforce_symmetry(H @ cov @ H.T + meas.R)
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self._log.warning("Singular innovation covariance; dropping sample")
            return state, cov, False

        if apply_gate:
            nis = float(innovation @ S_inv @ innovation)
            if nis > self._cfg.innovation_gate_chi2:
                return state, cov, False

        K = cov @ H.T @ S_inv
        new_state = state + K @ innovation

        # Joseph form: numerically stable and stays positive-definite even with
        # an imperfect gain, which matters over hours of 60 Hz operation.
        I_KH = np.eye(self.DIM) - K @ H
        new_cov = enforce_symmetry(I_KH @ cov @ I_KH.T + K @ meas.R @ K.T)
        return new_state, new_cov, True

    def _push_history(self, body_rate: np.ndarray, meas: Optional[Measurement]) -> None:
        self._history.append(
            FilterEpoch(
                timestamp=self._epoch,
                state=self._x.copy(),
                cov=self._P.copy(),
                body_rate=body_rate.copy(),
                measurement=meas,
            )
        )

    # -- Public API ---------------------------------------------------------

    def predict_to(self, target_time: float, body_rate: np.ndarray) -> None:
        """Advance the filter to ``target_time`` and record the epoch."""
        with self._lock:
            if not self._initialised:
                return
            dt = target_time - self._epoch
            if dt <= 0.0:
                return
            self._x, self._P = self._predict_inplace(self._x, self._P, body_rate, dt)
            self._epoch = target_time
            self._push_history(body_rate, None)

    def fuse(self, meas: Measurement, body_rate: np.ndarray) -> bool:
        """Fuse a (possibly stale) measurement, rewinding the filter as needed.

        Returns True if the sample was accepted into the track.
        """
        with self._lock:
            if not self._initialised:
                self._initialise_from(meas)
                return True

            # Sample older than anything retained -> unrecoverable, discard.
            if self._history and meas.capture_time < self._history[0].timestamp:
                self._log.warning(
                    "Measurement %.1f ms older than ring buffer; discarded",
                    (self._history[0].timestamp - meas.capture_time) * 1e3,
                )
                return False

            if meas.capture_time >= self._epoch:
                # Sample is current or ahead: ordinary predict/update.
                self._x, self._P = self._predict_inplace(
                    self._x, self._P, body_rate, meas.capture_time - self._epoch
                )
                self._epoch = meas.capture_time
                accepted = self._fuse_at_current_epoch(meas, body_rate)
                return accepted

            return self._retrodict_and_replay(meas, body_rate)

    def _fuse_at_current_epoch(self, meas: Measurement, body_rate: np.ndarray) -> bool:
        """Apply an update at the filter's present epoch, honouring the gate."""
        force = self._reject_streak >= self._cfg.max_consecutive_rejects
        new_x, new_P, accepted = self._update_inplace(
            self._x, self._P, meas, apply_gate=not force
        )

        if not accepted:
            self._reject_streak += 1
            self._push_history(body_rate, None)
            return False

        if force:
            self._log.warning(
                "Gate forced open after %d rejections; track reseeded",
                self._reject_streak,
            )

        self._x, self._P = new_x, new_P
        self._reject_streak = 0
        self._last_measurement_time = max(self._last_measurement_time, meas.capture_time)
        self._push_history(body_rate, meas)
        return True

    def _retrodict_and_replay(self, meas: Measurement, body_rate: np.ndarray) -> bool:
        """Splice a delayed measurement into the past, then rebuild the present.

        Procedure:
          1. Locate the newest buffered posterior at or before the capture time.
          2. Restore that posterior and predict forward to the capture instant.
          3. Fuse the delayed sample there.
          4. Replay every buffered epoch after it -- re-predicting and
             re-applying whatever measurement each already carried -- so the
             current estimate reflects the full, correctly ordered sample set.
        """
        anchor_index = -1
        for i in range(len(self._history) - 1, -1, -1):
            if self._history[i].timestamp <= meas.capture_time:
                anchor_index = i
                break

        if anchor_index < 0:
            self._log.warning("No retrodiction anchor available; sample discarded")
            return False

        anchor = self._history[anchor_index]
        state = anchor.state.copy()
        cov = anchor.cov.copy()
        cursor = anchor.timestamp

        # (2) Roll forward to the exact capture instant using the body rate that
        #     was in effect over that interval.
        state, cov = self._predict_inplace(
            state, cov, anchor.body_rate, meas.capture_time - cursor
        )
        cursor = meas.capture_time

        # (3) Inject the delayed observation.
        force = self._reject_streak >= self._cfg.max_consecutive_rejects
        state, cov, accepted = self._update_inplace(state, cov, meas, apply_gate=not force)
        if not accepted:
            self._reject_streak += 1
            return False

        self._reject_streak = 0
        self._last_measurement_time = max(self._last_measurement_time, meas.capture_time)

        # (4) Replay the tail of the buffer on top of the corrected past.
        replayed: List[FilterEpoch] = [
            FilterEpoch(cursor, state.copy(), cov.copy(), anchor.body_rate.copy(), meas)
        ]
        for epoch in list(self._history)[anchor_index + 1:]:
            state, cov = self._predict_inplace(
                state, cov, epoch.body_rate, epoch.timestamp - cursor
            )
            cursor = epoch.timestamp
            if epoch.measurement is not None:
                state, cov, ok = self._update_inplace(
                    state, cov, epoch.measurement, apply_gate=False
                )
                if not ok:  # pragma: no cover - gate disabled on replay
                    self._log.debug("Replay update skipped at t=%.4f", cursor)
            replayed.append(
                FilterEpoch(cursor, state.copy(), cov.copy(), epoch.body_rate.copy(),
                            epoch.measurement)
            )

        # Rebuild the ring buffer: history up to the anchor, then the replay.
        preserved = list(self._history)[: anchor_index + 1]
        self._history = deque(preserved + replayed, maxlen=self._cfg.history_depth)

        self._x, self._P = state, cov
        self._epoch = cursor
        return True


# ---------------------------------------------------------------------------
# 3-D True Proportional Navigation
# ---------------------------------------------------------------------------

@dataclass
class GuidanceSolution:
    """Output of one guidance evaluation."""

    accel_body: np.ndarray       # Commanded acceleration, BODY FRD [m/s^2]
    velocity_body: np.ndarray    # Integrated velocity setpoint, BODY FRD [m/s]
    yaw_rate: float              # Commanded yaw rate [rad/s]
    los_rate: np.ndarray         # LOS angular velocity vector [rad/s]
    closing_speed: float         # Positive when range is shrinking [m/s]
    range_m: float
    saturated: bool


class ProNavGuidance:
    """True Proportional Navigation in vector form, plus command shaping.

    True ProNav commands acceleration normal to the line of sight, proportional
    to both the LOS rotation rate and the closing speed:

        omega   = (r x v) / (r . r)              LOS angular velocity [rad/s]
        Vc      = -(v . r_hat)                   closing speed, +ve closing
        a_pronav = N * Vc * (omega x r_hat)      normal to LOS by construction

    Note the ordering: ``omega x r_hat`` and not ``omega x v``. The latter form
    appears in the literature written as ``N * (omega x V_r)``, but there
    ``V_r`` is the velocity of the *pursuer relative to the target*. This filter
    stores the opposite sign convention -- target relative to vehicle -- so
    using ``v`` directly would both invert the lateral command (steering away
    from the target's angular drift instead of leading it) and leave a spurious
    ``-|v_t|^2 / r`` component along the LOS. Crossing with the unit LOS vector
    and scaling by the scalar closing speed is algebraically equivalent to the
    textbook law and is unambiguous about sign.

    A separate LOS-axial term regulates range to the configured standoff; it is
    deliberately kept out of the ProNav channel because axial acceleration does
    nothing to null the LOS rate and would only consume control authority.
    """

    def __init__(self, config: GuidanceConfig, logger: logging.Logger) -> None:
        self._cfg = config
        self._log = logger
        self._velocity_cmd = np.zeros(3, dtype=np.float64)
        self._engaged = False

    def reset(self) -> None:
        """Zero the velocity integrator and drop the engagement latch."""
        self._velocity_cmd = np.zeros(3, dtype=np.float64)
        self._engaged = False

    def relax(self, dt: float) -> np.ndarray:
        """Bleed the standing command toward zero when guidance is inhibited."""
        tau = max(self._cfg.command_washout_tau_s, EPS)
        decay = math.exp(-dt / tau)
        self._velocity_cmd = self._velocity_cmd * decay
        if float(np.linalg.norm(self._velocity_cmd)) < 1e-3:
            self._velocity_cmd = np.zeros(3, dtype=np.float64)
        self._engaged = False
        return self._velocity_cmd.copy()

    def compute(
        self,
        rel_pos: np.ndarray,
        rel_vel: np.ndarray,
        dt: float,
    ) -> Optional[GuidanceSolution]:
        """Evaluate the guidance law for one control cycle.

        ``rel_pos`` / ``rel_vel`` are the target's position and velocity
        relative to the vehicle, expressed in BODY FRD.
        """
        rng = float(np.linalg.norm(rel_pos))
        if rng < EPS or not np.all(np.isfinite(rel_pos)) or not np.all(np.isfinite(rel_vel)):
            return None

        los_unit = rel_pos / rng

        # LOS angular velocity: the component of relative motion perpendicular
        # to the LOS, normalised by range.
        los_rate = np.cross(rel_pos, rel_vel) / (rng * rng)

        # Closing speed: positive shrinks the range.
        closing_speed = -float(np.dot(rel_vel, los_unit))
        effective_closure = max(closing_speed, self._cfg.min_effective_closure_mps)

        # True ProNav acceleration command, normal to the LOS by construction.
        accel_pronav = (
            self._cfg.nav_constant * effective_closure * np.cross(los_rate, los_unit)
        )

        # Axial channel: drive range toward the standoff set point.
        range_error = rng - self._cfg.standoff_range_m
        desired_closure = float(
            np.clip(
                self._cfg.closure_gain * range_error,
                -self._cfg.closure_speed_mps,
                self._cfg.closure_speed_mps,
            )
        )
        axial_accel = (
            self._cfg.closure_damping * (desired_closure - closing_speed) * los_unit
        )

        accel_raw = accel_pronav + axial_accel
        accel_cmd = saturate_norm(accel_raw, self._cfg.max_accel_mps2)
        accel_saturated = not np.allclose(accel_cmd, accel_raw)

        # Integrate acceleration into the velocity setpoint the autopilot wants.
        if not self._engaged:
            # Seed on acquisition with the pure closure solution so the first
            # frame does not slam a step change through the integrator.
            self._velocity_cmd = saturate_norm(
                desired_closure * los_unit, self._cfg.max_speed_mps
            )
            self._engaged = True
        else:
            self._velocity_cmd = self._velocity_cmd + accel_cmd * dt

        velocity_raw = self._velocity_cmd
        self._velocity_cmd = saturate_norm(velocity_raw, self._cfg.max_speed_mps)
        speed_saturated = not np.allclose(self._velocity_cmd, velocity_raw)

        # Yaw servo: null the LOS azimuth so the target stays centred laterally.
        azimuth_error = math.atan2(float(rel_pos[1]), float(rel_pos[0]))
        yaw_rate = float(
            np.clip(
                self._cfg.yaw_rate_gain * azimuth_error,
                -self._cfg.max_yaw_rate_rps,
                self._cfg.max_yaw_rate_rps,
            )
        )

        return GuidanceSolution(
            accel_body=accel_cmd,
            velocity_body=self._velocity_cmd.copy(),
            yaw_rate=yaw_rate,
            los_rate=los_rate,
            closing_speed=closing_speed,
            range_m=rng,
            saturated=accel_saturated or speed_saturated,
        )


# ---------------------------------------------------------------------------
# MAVLink transport
# ---------------------------------------------------------------------------

class MavlinkVelocityBridge:
    """Serial MAVLink bridge to the flight controller.

    Owns three responsibilities:
      * emit SET_POSITION_TARGET_LOCAL_NED velocity setpoints in BODY_NED,
      * maintain a companion-computer heartbeat so the autopilot keeps offboard
        control alive,
      * consume ATTITUDE to supply body angular rates to the estimator.
    """

    def __init__(self, config: LinkConfig, logger: logging.Logger) -> None:
        self._cfg = config
        self._log = logger
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._threads: List[threading.Thread] = []

        self._body_rate = np.zeros(3, dtype=np.float64)
        self._rate_lock = threading.Lock()
        self._boot_time = time.monotonic()

        self._master: Optional[mavutil.mavfile] = None

    # -- Lifecycle ----------------------------------------------------------

    def connect(self, wait_for_heartbeat: bool = True, timeout_s: float = 10.0) -> None:
        """Open the link and, optionally, block until the autopilot answers."""
        self._log.info("Opening MAVLink on %s @ %d baud", self._cfg.device, self._cfg.baud)
        self._master = mavutil.mavlink_connection(
            self._cfg.device,
            baud=self._cfg.baud,
            source_system=self._cfg.source_system,
            source_component=self._cfg.source_component,
            autoreconnect=True,
        )

        if wait_for_heartbeat:
            hb = self._master.wait_heartbeat(timeout=timeout_s)
            if hb is None:
                raise TimeoutError(
                    f"No heartbeat from autopilot within {timeout_s:.0f} s"
                )
            self._cfg.target_system = self._master.target_system
            self._cfg.target_component = self._master.target_component
            self._log.info(
                "Autopilot online: system %d, component %d",
                self._cfg.target_system,
                self._cfg.target_component,
            )

        self._running.set()
        self._spawn(self._heartbeat_loop, "mav-heartbeat")
        self._spawn(self._receive_loop, "mav-receive")

    def _spawn(self, target, name: str) -> None:
        thread = threading.Thread(target=target, name=name, daemon=True)
        thread.start()
        self._threads.append(thread)

    def close(self) -> None:
        """Stop the service threads and release the serial port."""
        self._running.clear()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()
        if self._master is not None:
            try:
                self._master.close()
            except Exception:  # pragma: no cover - best-effort teardown
                self._log.debug("Exception while closing MAVLink port", exc_info=True)
            self._master = None
        self._log.info("MAVLink link closed")

    # -- Service threads ----------------------------------------------------

    def _heartbeat_loop(self) -> None:
        """Announce the companion computer at a steady cadence."""
        period = 1.0 / max(self._cfg.heartbeat_hz, EPS)
        while self._running.is_set():
            try:
                with self._lock:
                    if self._master is not None:
                        self._master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0, 0,
                            mavutil.mavlink.MAV_STATE_ACTIVE,
                        )
            except Exception:
                self._log.warning("Heartbeat transmit failed", exc_info=True)
            time.sleep(period)

    def _receive_loop(self) -> None:
        """Drain the inbound stream, latching body angular rates from ATTITUDE."""
        while self._running.is_set():
            try:
                with self._lock:
                    master = self._master
                    msg = master.recv_match(blocking=False) if master is not None else None
            except Exception:
                self._log.warning("MAVLink receive error", exc_info=True)
                time.sleep(0.05)
                continue

            if msg is None:
                time.sleep(0.002)
                continue

            if msg.get_type() == "ATTITUDE":
                with self._rate_lock:
                    self._body_rate = np.array(
                        [msg.rollspeed, msg.pitchspeed, msg.yawspeed],
                        dtype=np.float64,
                    )

    # -- Accessors ----------------------------------------------------------

    def body_rate(self) -> np.ndarray:
        """Latest body angular velocity [p, q, r] in BODY FRD [rad/s]."""
        with self._rate_lock:
            return self._body_rate.copy()

    def request_attitude_stream(self, rate_hz: float = 50.0) -> None:
        """Ask the autopilot for ATTITUDE at the rate the estimator needs."""
        if self._master is None:
            return
        interval_us = int(1e6 / max(rate_hz, EPS))
        with self._lock:
            self._master.mav.command_long_send(
                self._cfg.target_system,
                self._cfg.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE),
                float(interval_us),
                0.0, 0.0, 0.0, 0.0, 0.0,
            )
        self._log.info("Requested ATTITUDE stream at %.0f Hz", rate_hz)

    # -- Command path -------------------------------------------------------

    def send_velocity_setpoint(self, velocity_body: np.ndarray, yaw_rate: float) -> bool:
        """Transmit a BODY_NED velocity setpoint.

        MAV_FRAME_BODY_NED interprets the vector in the vehicle's forward /
        right / down axes, which is exactly the frame the estimator and the
        guidance law already work in -- no rotation into the local NED frame is
        required, and the command stays valid regardless of vehicle heading.
        """
        if self._master is None:
            return False

        vx, vy, vz = (float(component) for component in velocity_body)
        elapsed_ms = int((time.monotonic() - self._boot_time) * 1e3) & 0xFFFFFFFF

        try:
            with self._lock:
                self._master.mav.set_position_target_local_ned_send(
                    elapsed_ms,
                    self._cfg.target_system,
                    self._cfg.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    TYPE_MASK_VELOCITY_YAWRATE,
                    0.0, 0.0, 0.0,          # position (ignored)
                    vx, vy, vz,             # velocity setpoint [m/s]
                    0.0, 0.0, 0.0,          # acceleration (ignored)
                    0.0,                    # yaw (ignored)
                    float(yaw_rate),        # yaw rate [rad/s]
                )
            return True
        except Exception:
            self._log.error("Failed to transmit velocity setpoint", exc_info=True)
            return False

    def send_hold(self) -> bool:
        """Command a zero-velocity hold -- the failsafe posture."""
        return self.send_velocity_setpoint(np.zeros(3, dtype=np.float64), 0.0)


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class RelativeNavigationNode(Node):
    """Binds perception input, estimation, guidance and the MAVLink command path."""

    def __init__(self, config: EngineConfig, bridge: MavlinkVelocityBridge) -> None:
        super().__init__("relative_nav_pronav")

        self._cfg = config
        self._bridge = bridge

        self._ekf = LatencyCompensatedEKF(config.filt, logging.getLogger("gnc.ekf"))
        self._guidance = ProNavGuidance(config.guidance, logging.getLogger("gnc.pronav"))

        self._state_lock = threading.RLock()
        self._last_control_time: Optional[float] = None
        self._frames_ingested = 0
        self._frames_accepted = 0
        self._commands_sent = 0
        self._last_solution: Optional[GuidanceSolution] = None

        # Perception ingest and the control loop run on separate mutually
        # exclusive groups: the two may overlap (the whole point of the
        # multi-threaded executor), but neither may re-enter itself. Re-entrant
        # control ticks would race on the guidance integrator, and re-entrant
        # ingest would interleave two rollbacks over one ring buffer. The EKF is
        # additionally guarded by its own lock, since both groups touch it.
        sensor_group = MutuallyExclusiveCallbackGroup()
        control_group = MutuallyExclusiveCallbackGroup()

        # Sensor-grade QoS: best-effort and shallow. A dropped detection is
        # cheaper than a stale one queued behind it at 60 Hz.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._subscription = self.create_subscription(
            PointStamped,
            config.measurement_topic,
            self._on_measurement,
            sensor_qos,
            callback_group=sensor_group,
        )

        self._command_publisher = self.create_publisher(
            TwistStamped, config.command_echo_topic, 10
        )

        control_period = 1.0 / max(config.control_rate_hz, EPS)
        self._control_timer = self.create_timer(
            control_period, self._on_control_tick, callback_group=control_group
        )
        self._telemetry_timer = self.create_timer(
            2.0, self._on_telemetry_tick, callback_group=control_group
        )

        self.get_logger().info(
            f"Engine armed | control {config.control_rate_hz:.0f} Hz | "
            f"latency compensation {config.sensor.pipeline_latency_s * 1e3:.1f} ms | "
            f"N={config.guidance.nav_constant:.1f} | "
            f"a_max={config.guidance.max_accel_mps2:.2f} m/s^2 | "
            f"v_max={config.guidance.max_speed_mps:.2f} m/s"
        )

    # -- Time ---------------------------------------------------------------

    def _now(self) -> float:
        """Current ROS time in seconds; the single clock domain for the engine."""
        return self.get_clock().now().nanoseconds * 1e-9

    # -- Measurement ingest -------------------------------------------------

    def _build_measurement_covariance(self, rng: float) -> np.ndarray:
        """Polar measurement covariance R at a given range."""
        sensor = self._cfg.sensor
        sigma_bearing = sensor.sigma_bearing_rad
        sigma_range = sensor.sigma_range_base_m + sensor.sigma_range_quad * rng * rng
        return np.diag(
            [sigma_bearing ** 2, sigma_bearing ** 2, sigma_range ** 2]
        ).astype(np.float64)

    def _on_measurement(self, msg: PointStamped) -> None:
        """Convert a perception detection into a latency-corrected observation."""
        self._frames_ingested += 1

        point_camera = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=np.float64
        )
        if not np.all(np.isfinite(point_camera)):
            self.get_logger().warning("Non-finite detection discarded", throttle_duration_sec=2.0)
            return

        point_body = R_CAM_TO_BODY @ point_camera
        rng = float(np.linalg.norm(point_body))
        if not (self._cfg.sensor.min_range_m <= rng <= self._cfg.sensor.max_range_m):
            self.get_logger().warning(
                f"Detection at {rng:.2f} m outside valid range band; discarded",
                throttle_duration_sec=2.0,
            )
            return

        # Recover the true capture epoch. The header stamp is applied when the
        # frame reaches the detector, which is a fixed two frames downstream of
        # exposure; subtracting that constant puts the sample where it belongs
        # on the timeline. Set pipeline_latency_frames to 0 if the driver
        # already stamps at exposure.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = self._now()
        capture_time = stamp - self._cfg.sensor.pipeline_latency_s

        measurement = Measurement(
            capture_time=capture_time,
            polar=cartesian_to_polar(point_body),
            R=self._build_measurement_covariance(rng),
        )

        body_rate = self._bridge.body_rate()
        with self._state_lock:
            if self._ekf.fuse(measurement, body_rate):
                self._frames_accepted += 1

    # -- Control loop -------------------------------------------------------

    def _on_control_tick(self) -> None:
        """Fixed-rate guidance evaluation and command transmission."""
        now = self._now()
        dt = (
            now - self._last_control_time
            if self._last_control_time is not None
            else 1.0 / self._cfg.control_rate_hz
        )
        self._last_control_time = now

        # Guard against clock jumps (e.g. an NTP step or a simulated-time reset).
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self._cfg.control_rate_hz

        body_rate = self._bridge.body_rate()

        with self._state_lock:
            track_live = self._ekf.is_track_fresh(now)
            if track_live:
                # Bring the estimate up to the current epoch before solving.
                self._ekf.predict_to(now, body_rate)
                rel_pos = self._ekf.relative_position()
                rel_vel = self._ekf.relative_velocity()
            else:
                rel_pos = rel_vel = None

        if track_live and rel_pos is not None and rel_vel is not None:
            solution = self._guidance.compute(rel_pos, rel_vel, dt)
        else:
            solution = None

        if solution is None:
            # Failsafe: wash the standing command out rather than dropping it,
            # so a single-frame perception dropout does not produce a jolt.
            velocity_cmd = self._guidance.relax(dt)
            yaw_rate = 0.0
            self._last_solution = None
            if not track_live:
                self.get_logger().warning(
                    "Track stale; washing out velocity command",
                    throttle_duration_sec=1.0,
                )
        else:
            velocity_cmd = solution.velocity_body
            yaw_rate = solution.yaw_rate
            self._last_solution = solution
            if solution.saturated:
                # Persistent saturation means the engagement geometry is asking
                # for more than the airframe can deliver -- worth surfacing, but
                # the command itself is already clipped and safe to fly.
                self.get_logger().warning(
                    f"Command saturated at range {solution.range_m:.1f} m "
                    f"(|a| = {float(np.linalg.norm(solution.accel_body)):.1f} m/s^2, "
                    f"|v| = {float(np.linalg.norm(velocity_cmd)):.1f} m/s)",
                    throttle_duration_sec=1.0,
                )

        if self._bridge.send_velocity_setpoint(velocity_cmd, yaw_rate):
            self._commands_sent += 1

        self._publish_command_echo(now, velocity_cmd, yaw_rate)

    def _publish_command_echo(
        self, stamp: float, velocity: np.ndarray, yaw_rate: float
    ) -> None:
        """Mirror the transmitted setpoint onto a ROS topic for logging/analysis."""
        msg = TwistStamped()
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        msg.header.frame_id = "body_frd"
        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = float(velocity[2])
        msg.twist.angular.z = float(yaw_rate)
        self._command_publisher.publish(msg)

    # -- Telemetry ----------------------------------------------------------

    def _on_telemetry_tick(self) -> None:
        """Periodic health line covering the whole signal chain."""
        with self._state_lock:
            state, cov, epoch = self._ekf.snapshot()
            initialised = self._ekf.initialised

        if not initialised:
            self.get_logger().info(
                f"No track | frames in {self._frames_ingested} | "
                f"commands {self._commands_sent}"
            )
            return

        pos = state[IDX_POS]
        vel = state[IDX_VEL]
        pos_sigma = math.sqrt(max(float(np.trace(cov[np.ix_(IDX_POS, IDX_POS)])), 0.0))
        acceptance = (
            100.0 * self._frames_accepted / self._frames_ingested
            if self._frames_ingested
            else 0.0
        )

        solution = self._last_solution
        guidance_line = (
            f"Vc {solution.closing_speed:6.2f} m/s | "
            f"LOS-rate {float(np.linalg.norm(solution.los_rate)) * 1e3:6.1f} mrad/s"
            if solution is not None
            else "guidance idle"
        )

        self.get_logger().info(
            f"range {float(np.linalg.norm(pos)):6.2f} m | "
            f"rel-v [{vel[0]:6.2f} {vel[1]:6.2f} {vel[2]:6.2f}] m/s | "
            f"pos-sigma {pos_sigma:5.2f} m | {guidance_line} | "
            f"accept {acceptance:5.1f}% | epoch age {(self._now() - epoch) * 1e3:5.1f} ms"
        )

    # -- Shutdown -----------------------------------------------------------

    def shutdown(self) -> None:
        """Command a hold and tear down timers before the process exits."""
        try:
            self._control_timer.cancel()
            self._telemetry_timer.cancel()
        except Exception:  # pragma: no cover - teardown ordering
            pass
        self._guidance.reset()
        self._bridge.send_hold()
        self.get_logger().info("Guidance disengaged; zero-velocity hold sent")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def build_config(args: argparse.Namespace) -> EngineConfig:
    """Fold CLI arguments into the engine configuration tree."""
    config = EngineConfig()
    config.link.device = args.device
    config.link.baud = args.baud
    config.control_rate_hz = args.control_rate
    config.measurement_topic = args.topic
    config.sensor.frame_rate_hz = args.frame_rate
    config.sensor.pipeline_latency_frames = args.latency_frames
    config.guidance.nav_constant = args.nav_constant
    config.guidance.standoff_range_m = args.standoff
    return config


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Latency-compensated EKF + True ProNav relative navigation engine",
    )
    parser.add_argument("--device", default="/dev/ttyTHS1",
                        help="MAVLink endpoint (serial path or udpout:host:port)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--topic", default="/perception/target_point",
                        help="geometry_msgs/PointStamped topic of target coordinates")
    parser.add_argument("--control-rate", type=float, default=60.0,
                        help="Guidance/command loop rate [Hz]")
    parser.add_argument("--frame-rate", type=float, default=60.0,
                        help="Camera frame rate [Hz]")
    parser.add_argument("--latency-frames", type=float, default=2.0,
                        help="Fixed perception pipeline latency, in camera frames")
    parser.add_argument("--nav-constant", type=float, default=4.0,
                        help="ProNav navigation constant N")
    parser.add_argument("--standoff", type=float, default=3.0,
                        help="Commanded standoff range [m]")
    parser.add_argument("--no-wait-heartbeat", action="store_true",
                        help="Do not block on the autopilot heartbeat at startup")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d %(name)-12s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )
    log = logging.getLogger("gnc.main")

    config = build_config(args)
    bridge = MavlinkVelocityBridge(config.link, logging.getLogger("gnc.mavlink"))

    try:
        bridge.connect(wait_for_heartbeat=not args.no_wait_heartbeat)
        bridge.request_attitude_stream(rate_hz=50.0)
    except Exception:
        log.exception("Unable to establish the MAVLink link")
        return 1

    rclpy.init(args=None)
    node = RelativeNavigationNode(config, bridge)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    stop_requested = threading.Event()

    def _handle_signal(signum, _frame):
        log.info("Signal %d received; shutting down", signum)
        stop_requested.set()
        executor.shutdown(timeout_sec=0.0)

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    exit_code = 0
    try:
        executor.spin()
    except KeyboardInterrupt:  # pragma: no cover - interactive path
        log.info("Interrupted")
    except Exception:
        log.exception("Fatal error in the control executor")
        exit_code = 1
    finally:
        try:
            node.shutdown()
        finally:
            executor.remove_node(node)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            bridge.close()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())

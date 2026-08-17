#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Autonomous inspection GNC engine -- single-file deployment.

    Compute   : NVIDIA Jetson Orin Nano (Ubuntu 22.04 LTS, JetPack 6.x)
    Sensor    : Arducam Global Shutter MIPI CSI-2 module, 1080p @ 60 Hz
    Autopilot : Pixhawk / Cube Orange, UART1 /dev/ttyTHS1 @ 921600 baud

Dependencies: numpy, opencv-python (with GStreamer), pymavlink, pyserial.
No ROS 2 required.

Pipeline
--------
    [capture thread]   cap.read() -> stamp -> latest-frame slot (overwrite)
    [detector thread]  ego-motion differencing -> polar measurement -> queue
    [control thread]   60 Hz: fuse -> predict -> ProNav -> MAVLink setpoint
    [supervisor]       200 Hz independent watchdog -> authority + handover

Every hand-off between stages is non-blocking and lossy by design, so no
camera hiccup can stretch the 60 Hz estimation cycle. Frames are stamped at
grab and that stamp rides into the measurement, so a slow detector produces a
correctly back-dated observation rather than a mis-timed one.

Frames
------
    CAMERA (optical) : +x right, +y down, +z forward (REP-103 optical frame)
    BODY   (FRD)     : +x forward, +y right, +z down

Estimation and guidance run in BODY FRD, the native frame of
MAV_FRAME_BODY_NED velocity setpoints. BODY rotates, so the filter carries the
transport terms -omega x r and -omega x v from the autopilot ATTITUDE stream.

State vector (interleaved)
--------------------------
    x = [x, vx, y, vy, z, vz]^T   relative target position / velocity, BODY FRD

Measurement vector
------------------
    z = [azimuth, elevation, range]^T

Bearing accuracy is set by pixel pitch and is excellent; range accuracy
degrades with the square of distance. Updating in polar space with an analytic
Jacobian represents that anisotropy instead of averaging it into an isotropic
blob -- and is what makes this an EKF rather than a linear KF.

Usage
-----
    # Bench, no hardware:
    python3 inspection_gnc.py --backend synthetic --device udpout:127.0.0.1:14550

    # Flight hardware:
    python3 inspection_gnc.py --backend argus --device /dev/ttyTHS1 --baud 921600
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import signal
import sys
import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np
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

# Command velocity + acceleration + yaw-rate. Feeding the ProNav acceleration
# through as a feed-forward term lets the autopilot anticipate the manoeuvre
# instead of differentiating our velocity command, which removes one lag pole
# from the loop.
#
# Firmware caveat: acceleration targets in SET_POSITION_TARGET_LOCAL_NED are a
# comparatively recent ArduPilot Copter feature and older firmware silently
# ignores the acceleration fields rather than rejecting the packet -- the
# vehicle simply flies the velocity term and you get no error. Confirm against
# your Cube Orange's firmware version before relying on it, and keep
# send_acceleration=False (the default) until you have.
TYPE_MASK_VELOCITY_ACCEL_YAWRATE = (
    TM_IGNORE_PX | TM_IGNORE_PY | TM_IGNORE_PZ
    | TM_IGNORE_YAW
)  # == 1031

# ArduPilot Copter flight-mode numbers, used when handing authority back. The
# live mapping is read from the vehicle at connect time; this table is the
# fallback if the autopilot does not supply one.
ARDUPILOT_COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
    "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9, "DRIFT": 11,
    "SPORT": 13, "FLIP": 14, "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17,
    "THROW": 18, "AVOID_ADSB": 19, "GUIDED_NOGPS": 20, "SMART_RTL": 21,
}

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
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class FilterConfig:
    """EKF tuning."""

    # Continuous-time white-noise-acceleration spectral density [m^2/s^5].
    # Sized for a non-cooperative *target* capable of aggressive manoeuvres.
    # Our own acceleration is fed in separately as a control input, so it does
    # not need to be covered here; if you call predict/fuse without own_accel,
    # raise this to cover your own airframe's manoeuvre envelope as well.
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

    # Include the ProNav acceleration as a feed-forward term in the setpoint.
    # Off by default: older Copter firmware ignores the acceleration fields
    # silently, so enabling it without checking buys nothing and hides the fact.
    send_acceleration: bool = False

@dataclass
class CameraConfig:
    """Capture pipeline settings for the Arducam GS module."""

    # "argus"     : nvarguscamerasrc, for Argus-compatible modules (IMX296 etc.)
    # "v4l2"      : v4l2src, for Arducam's raw mono GS drivers (OV9281/OV2311)
    # "synthetic" : software frame generator, for bench runs without hardware
    backend: str = "argus"

    sensor_id: int = 0
    device: str = "/dev/video0"
    width: int = 1920
    height: int = 1080
    frame_rate_hz: float = 60.0

    # nvvidconv flip-method (0 none, 2 180 deg, ...). Argus backend only.
    flip_method: int = 0

    # Fixed sensor-to-userspace latency in frames: exposure midpoint through
    # readout, CSI-2 DMA and ISP, up to the instant read() hands us the buffer.
    # This is the constant the estimator's retrodiction was sized for. Measure
    # it once per build with an LED strobe; do not guess it per flight.
    sensor_latency_frames: float = 2.0

    # Fail the health check if no frame arrives within this many nominal periods.
    stale_frame_periods: float = 6.0

    # Try to place the capture thread on a real-time scheduling class. Requires
    # CAP_SYS_NICE; the failure path is a warning, never a hard stop.
    realtime_priority: Optional[int] = 20

    @property
    def frame_period_s(self) -> float:
        return 1.0 / max(self.frame_rate_hz, EPS)

    @property
    def sensor_latency_s(self) -> float:
        return self.sensor_latency_frames * self.frame_period_s


@dataclass
class IntrinsicsConfig:
    """Pinhole intrinsics and Brown-Conrady distortion for the fitted lens."""

    fx: float = 1371.0
    fy: float = 1371.0
    cx: float = 960.0
    cy: float = 540.0
    # [k1, k2, p1, p2, k3]
    distortion: Tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)

    # 1-sigma centroid repeatability of the detector, in pixels. This is the
    # dominant bearing error term and is a property of the detector, not the
    # lens -- measure it from a static target on the bench.
    centroid_sigma_px: float = 0.7
    # 1-sigma repeatability of the measured target extent, in pixels. Always
    # worse than the centroid: two edges, each blurred.
    extent_sigma_px: float = 2.5


@dataclass
class TargetPrior:
    """What we assume about the target's physical size.

    Monocular range is size-scale range: it cannot be better than the size
    prior. ``width_sigma_m`` is not decoration -- it enters the range covariance
    as a multiplicative term and is what stops the EKF from trusting a
    confidently-wrong range.
    """

    width_m: float = 0.50
    width_sigma_m: float = 0.10


@dataclass
class DetectorConfig:
    """Ego-motion-compensated motion detector tuning."""

    # Detection runs on a downscaled image. At 0.5 the 1080p frame becomes
    # 960x540, which is where the Orin Nano's CPU can hold 60 Hz with headroom.
    detect_scale: float = 0.5

    # Temporal baseline for differencing, in frames. Differencing against the
    # immediately previous frame is the wrong choice at 60 Hz: a target drifting
    # at a few pixels per second moves a small fraction of a pixel between
    # consecutive frames, so the motion residual sits in the noise floor and the
    # detector goes blind on exactly the slow, distant targets that matter most.
    # Referencing N frames back multiplies the displacement by N; the ego-motion
    # warp absorbs the correspondingly larger camera motion. At 60 Hz a stride
    # of 4 gives a 67 ms baseline.
    temporal_stride: int = 4

    # Ego-motion estimation: sparse LK flow between consecutive frames.
    max_corners: int = 240
    corner_quality: float = 0.01
    min_corner_distance_px: float = 12.0
    lk_window_px: int = 21
    lk_levels: int = 3
    min_inliers: int = 12

    # Residual-difference thresholding, in units of the residual image's own
    # standard deviation. Adaptive, so it survives exposure and gain changes.
    threshold_sigma: float = 5.0
    min_absolute_threshold: int = 12

    morph_kernel_px: int = 3
    min_blob_area_px: float = 12.0
    max_blob_area_fraction: float = 0.10

    # Silhouette refinement. The motion residual locates the target but must not
    # be used to size it -- see EgoMotionBlobDetector._refine_extent.
    refine_extent: bool = True
    extent_window_factor: float = 3.0
    min_extent_window_px: int = 24
    # A component filling more than this fraction of the refinement window is
    # background, not target; reject that polarity.
    max_extent_area_fraction: float = 0.5

    # Systematic bias of the segmented extent, in pixels, subtracted from the
    # measured width. connectedComponents reports a span of N pixel *centres* as
    # width N, while the object's true edge-to-edge extent is closer to N - 0.5
    # once partially-covered boundary pixels are accounted for. Left
    # uncorrected this reads as an over-wide target and biases range LOW by
    # roughly 0.5/w -- 2.8% on a 18 px target, and worse as the target shrinks.
    # A bias is the one error the EKF cannot average away, so calibrate this on
    # the bench against a known target at surveyed ranges rather than trusting
    # the geometric default.
    extent_bias_px: float = 0.5

    # Once locked, search a window around the last detection instead of the full
    # frame. Cheaper and far less likely to latch onto clutter.
    roi_margin_factor: float = 4.0
    min_roi_px: int = 96
    # Consecutive misses tolerated before falling back to a full-frame search.
    max_track_misses: int = 8


@dataclass
class FrontendConfig:
    """Top-level front-end configuration."""

    camera: CameraConfig = field(default_factory=CameraConfig)
    intrinsics: IntrinsicsConfig = field(default_factory=IntrinsicsConfig)
    target: TargetPrior = field(default_factory=TargetPrior)
    detector: DetectorConfig = field(default_factory=DetectorConfig)

    # Detector worker threads. One is correct for a strictly ordered pipeline;
    # two lets a slow frame overlap with the next one at the cost of occasional
    # out-of-order delivery, which the EKF's retrodiction handles natively.
    detector_threads: int = 1

    # Bound on the measurement hand-off queue. Deliberately shallow: anything
    # deeper is latency the estimator would have to undo.
    measurement_queue_depth: int = 8

    # Range acceptance band. Outside it the size-scale solution is meaningless.
    min_range_m: float = 0.35
    max_range_m: float = 150.0


# ---------------------------------------------------------------------------
# Intrinsics
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class SupervisorConfig:
    """Trip thresholds and handover behaviour."""

    # Supervisor tick rate. At 200 Hz the detection granularity is 5 ms, an
    # order of magnitude finer than the tightest threshold below.
    check_rate_hz: float = 200.0

    # --- Liveness thresholds ------------------------------------------------
    # Camera hardware: 100 ms is six frames at 60 Hz. This is fed from the
    # capture thread, so it measures exactly what it says -- frames arriving
    # from the sensor. A wedged CSI-2 pipeline, an unplugged ribbon or a stuck
    # ISP shows up here within six frame periods.
    max_frame_gap_s: float = 0.100

    # Target track: frames are arriving but none of them are yielding a usable
    # measurement. This is a *different* failure from a dead camera and needs a
    # different budget -- a detector p99 spike or a target briefly occluded is
    # routine, and the EKF is built to coast through it. Tying this to the
    # 100 ms camera budget would turn every detector hiccup into a handover.
    # The filter's own covariance growth is the backstop: if coasting goes on
    # long enough to matter, FILTER_DIVERGED fires on its own.
    max_measurement_gap_s: float = 0.500

    # Control loop: nine missed 60 Hz cycles. Deliberately not three.
    # CPython on a loaded Jetson will occasionally lose the control thread to a
    # scheduler preemption or a GC pause for 50-80 ms; measured on a 4-core box
    # under full pipeline load, worst-case control gaps reached 82 ms with the
    # loop otherwise healthy. A threshold tight enough to catch those would
    # hand the vehicle to Loiter mid-inspection for a hiccup that cost it a
    # third of a metre. Sustained degradation is caught separately and more
    # meaningfully by max_consecutive_overruns below, which detects a loop that
    # can no longer hold its rate rather than one that stumbled once.
    max_control_gap_s: float = 0.150

    # A single slow cycle is normal; sustained overrun means the loop can no
    # longer hold its rate and the guidance integration step is wrong.
    max_control_cycle_s: float = 0.025
    max_consecutive_overruns: int = 10

    # ArduPilot emits HEARTBEAT at 1 Hz. Three missed beats is unambiguous.
    max_heartbeat_age_s: float = 3.0

    # --- Filter health ------------------------------------------------------
    # Position uncertainty above this means the track is no longer trustworthy
    # enough to steer by.
    max_position_sigma_m: float = 12.0
    max_velocity_sigma_mps: float = 25.0
    # The estimate must be no older than this when guidance consumes it.
    max_filter_epoch_age_s: float = 0.20

    # --- Link quality -------------------------------------------------------
    max_consecutive_transmit_failures: int = 5

    # --- Debounce and grace -------------------------------------------------
    # Consecutive supervisor ticks a fault must persist before it trips. The
    # duration-based checks are self-debouncing; this guards the instantaneous
    # ones against a single bad sample.
    trip_debounce_ticks: int = 3
    # No trips during startup: the camera has not delivered a frame yet and the
    # filter has no track, which would otherwise trip instantly.
    startup_grace_s: float = 3.0

    # --- Handover -----------------------------------------------------------
    # Tried in order until one is confirmed. LOITER holds position and waits for
    # the pilot; BRAKE stops without needing a home position; RTL flies home;
    # ALT_HOLD and LAND are the no-GPS fallbacks. Each is verified by reading
    # the flight mode back from HEARTBEAT.
    handover_modes: tuple = ("LOITER", "BRAKE", "RTL", "ALT_HOLD")
    # Modes requiring a 3-D position fix, skipped when the GPS cannot support
    # them rather than wasting the confirmation timeout on a refusal.
    position_dependent_modes: frozenset = frozenset({"LOITER", "RTL", "SMART_RTL", "POSHOLD"})
    mode_confirm_timeout_s: float = 1.0
    mode_attempts: int = 2

    # Latch the trip until a human calls reset().
    latch: bool = True

    # Best-effort real-time priority. Above the capture thread: the watchdog
    # must be scheduled even when the pipeline is saturating the CPU.
    realtime_priority: Optional[int] = 40


# ---------------------------------------------------------------------------
# Health reporting helper
# ---------------------------------------------------------------------------

@dataclass
class IntegrationConfig:
    """Everything the integrated engine needs."""

    frontend: FrontendConfig = field(default_factory=FrontendConfig)
    filt: FilterConfig = field(default_factory=FilterConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    link: LinkConfig = field(default_factory=LinkConfig)
    supervisor: SupervisorConfig = field(default_factory=SupervisorConfig)

    control_rate_hz: float = 60.0
    telemetry_period_s: float = 2.0

    # Only transmit while the autopilot is in GUIDED. Turning this off removes
    # the pilot's ability to take over by mode switch, so it stays on.
    require_guided_mode: bool = True

    # Wait for the autopilot heartbeat before starting. Off only for camera-only
    # bench runs where no flight controller is attached.
    wait_for_heartbeat: bool = True


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


# ---------------------------------------------------------------------------
# Latency-compensated Extended Kalman Filter
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
    own_accel: np.ndarray
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
        self._push_history(np.zeros(3), meas, np.zeros(3))
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

    @staticmethod
    def _control_matrix(body_rate: np.ndarray, dt: float) -> np.ndarray:
        """Discrete input matrix G (6x3) mapping own-vehicle acceleration.

        Our own acceleration is *known*, not random. Left in the process noise
        it inflates the covariance and drags the estimate: during a 6 G
        manoeuvre the filter is being told "relative velocity may have changed
        by some unknown amount" when in fact most of that change is our own and
        is measured. Feeding it as a control input removes it from the
        uncertainty budget entirely, leaving the process noise to cover only
        what is genuinely unknown -- the target's manoeuvre.

        For A = [[-Omega, I], [0, -Omega]] the blocks commute, so
        exp(A*t) = [[E, t*E], [0, E]] with E = exp(-Omega*t), and

            G = integral(0..dt) [ s*exp(-Omega*s) ; exp(-Omega*s) ] ds
              = [ dt^2/2 I - dt^3/3 Omega + dt^4/8 Omega^2 ;
                  dt   I - dt^2/2 Omega + dt^3/6 Omega^2 ]

        truncated to the same order as the state transition. At dt = 1/60 the
        Omega terms are a ~2% correction at 1 rad/s, but they cost two matrix
        products and keep G consistent with F.
        """
        Omega = skew(body_rate)
        Omega2 = Omega @ Omega
        I3 = np.eye(3)

        dt2, dt3, dt4 = dt * dt, dt * dt * dt, dt * dt * dt * dt
        g_pos = (dt2 / 2.0) * I3 - (dt3 / 3.0) * Omega + (dt4 / 8.0) * Omega2
        g_vel = dt * I3 - (dt2 / 2.0) * Omega + (dt3 / 6.0) * Omega2

        G = np.zeros((LatencyCompensatedEKF.DIM, 3), dtype=np.float64)
        G[IDX_POS, :] = g_pos
        G[IDX_VEL, :] = g_vel
        return G

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
        own_accel: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Single prediction step; returns fresh (state, covariance).

        ``own_accel`` is this vehicle's own acceleration in BODY FRD. The state
        is the target *relative* to us, so our acceleration enters with a
        negative sign: accelerating toward the target reduces the relative
        velocity. Pass None (or zeros) to fall back to treating all relative
        acceleration as process noise.
        """
        if dt <= 0.0:
            return state, cov
        F = self._state_transition(body_rate, dt)
        new_state = F @ state
        if own_accel is not None and np.any(own_accel):
            G = self._control_matrix(body_rate, dt)
            new_state = new_state - G @ np.asarray(own_accel, dtype=np.float64)
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

    def _push_history(self, body_rate: np.ndarray, meas: Optional[Measurement],
                      own_accel: Optional[np.ndarray] = None) -> None:
        self._history.append(
            FilterEpoch(
                timestamp=self._epoch,
                state=self._x.copy(),
                cov=self._P.copy(),
                body_rate=body_rate.copy(),
                own_accel=(np.zeros(3) if own_accel is None
                           else np.asarray(own_accel, dtype=np.float64).copy()),
                measurement=meas,
            )
        )

    # -- Public API ---------------------------------------------------------

    def predict_to(self, target_time: float, body_rate: np.ndarray,
                   own_accel: Optional[np.ndarray] = None) -> None:
        """Advance the filter to ``target_time`` and record the epoch.

        ``own_accel`` is recorded alongside the epoch so a later retrodiction
        replays this interval with the acceleration that was actually in force
        over it, not with whatever is current when the delayed sample lands.
        """
        with self._lock:
            if not self._initialised:
                return
            dt = target_time - self._epoch
            if dt <= 0.0:
                return
            self._x, self._P = self._predict_inplace(
                self._x, self._P, body_rate, dt, own_accel
            )
            self._epoch = target_time
            self._push_history(body_rate, None, own_accel)

    def fuse(self, meas: Measurement, body_rate: np.ndarray,
             own_accel: Optional[np.ndarray] = None) -> bool:
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
                    self._x, self._P, body_rate,
                    meas.capture_time - self._epoch, own_accel
                )
                self._epoch = meas.capture_time
                accepted = self._fuse_at_current_epoch(meas, body_rate, own_accel)
                return accepted

            return self._retrodict_and_replay(meas, body_rate, own_accel)

    def _fuse_at_current_epoch(self, meas: Measurement, body_rate: np.ndarray,
                               own_accel: Optional[np.ndarray] = None) -> bool:
        """Apply an update at the filter's present epoch, honouring the gate."""
        force = self._reject_streak >= self._cfg.max_consecutive_rejects
        new_x, new_P, accepted = self._update_inplace(
            self._x, self._P, meas, apply_gate=not force
        )

        if not accepted:
            self._reject_streak += 1
            self._push_history(body_rate, None, own_accel)
            return False

        if force:
            self._log.warning(
                "Gate forced open after %d rejections; track reseeded",
                self._reject_streak,
            )

        self._x, self._P = new_x, new_P
        self._reject_streak = 0
        self._last_measurement_time = max(self._last_measurement_time, meas.capture_time)
        self._push_history(body_rate, meas, own_accel)
        return True

    def _retrodict_and_replay(self, meas: Measurement, body_rate: np.ndarray,
                              own_accel: Optional[np.ndarray] = None) -> bool:
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
            state, cov, anchor.body_rate, meas.capture_time - cursor, anchor.own_accel
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
            FilterEpoch(cursor, state.copy(), cov.copy(), anchor.body_rate.copy(),
                        anchor.own_accel.copy(), meas)
        ]
        for epoch in list(self._history)[anchor_index + 1:]:
            state, cov = self._predict_inplace(
                state, cov, epoch.body_rate, epoch.timestamp - cursor, epoch.own_accel
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
                            epoch.own_accel.copy(), epoch.measurement)
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

        # Vehicle state latched from the inbound stream. Guarded by its own
        # lock, never by the transmit lock: the safety supervisor reads this on
        # every tick and must never queue behind a serial write.
        self._state_lock = threading.Lock()
        self._last_heartbeat_time = 0.0
        self._custom_mode: Optional[int] = None
        self._base_mode = 0
        self._armed = False
        self._gps_fix_type = 0
        self._satellites = 0
        self._ekf_flags: Optional[int] = None
        self._messages_received = 0
        self._setpoints_sent = 0
        self._transmit_failures = 0

        self._mode_mapping: dict = dict(ARDUPILOT_COPTER_MODES)
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
            # Take both IDs from the heartbeat we actually received, not from
            # master.target_component -- pymavlink leaves that at 0, meaning
            # "broadcast to every component". That is fine for addressing
            # commands, but using it as a receive filter discards every real
            # autopilot heartbeat, which arrives from component 1.
            self._cfg.target_system = hb.get_srcSystem()
            self._cfg.target_component = hb.get_srcComponent()
            self._log.info(
                "Autopilot online: system %d, component %d",
                self._cfg.target_system,
                self._cfg.target_component,
            )
            with self._state_lock:
                self._last_heartbeat_time = time.monotonic()
                self._custom_mode = getattr(hb, "custom_mode", None)
                self._base_mode = getattr(hb, "base_mode", 0)

            # Prefer the mapping the connected vehicle reports over our table:
            # mode numbers differ between Copter, Plane and Rover.
            try:
                mapping = self._master.mode_mapping()
                if mapping:
                    self._mode_mapping = {str(k).upper(): int(v)
                                          for k, v in mapping.items()}
                    self._log.info("Loaded %d flight modes from the vehicle",
                                   len(self._mode_mapping))
            except Exception:
                self._log.warning(
                    "Vehicle did not supply a mode mapping; using the Copter table"
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
        # Drain a batch per lock acquisition rather than one message per
        # acquisition. The transmit path shares this lock, so a message-at-a-
        # time loop makes every setpoint write contend with the receiver on a
        # busy link -- and on a GIL-bound runtime the churn preempts the
        # detector thread as well.
        batch_limit = 16
        pending: List = []

        while self._running.is_set():
            if not pending:
                try:
                    with self._lock:
                        master = self._master
                        if master is not None:
                            for _ in range(batch_limit):
                                message = master.recv_match(blocking=False)
                                if message is None:
                                    break
                                pending.append(message)
                except Exception:
                    self._log.warning("MAVLink receive error", exc_info=True)
                    time.sleep(0.05)
                    continue

            if not pending:
                time.sleep(0.002)
                continue

            msg = pending.pop(0)
            try:
                self._dispatch(msg)
            except Exception:
                # One unparseable or unexpected message must never take the
                # receive thread down with it. Without this, a single missing
                # field silently kills the reader, the heartbeat goes stale and
                # the supervisor trips on a link that is actually fine.
                self._log.warning(
                    "Dropping malformed %s message", msg.get_type(), exc_info=True
                )

    def _dispatch(self, msg) -> None:
        """Latch one inbound message into the shared vehicle state."""
        kind = msg.get_type()
        with self._state_lock:
            self._messages_received += 1

        if kind == "ATTITUDE":
            with self._rate_lock:
                self._body_rate = np.array(
                    [msg.rollspeed, msg.pitchspeed, msg.yawspeed],
                    dtype=np.float64,
                )

        elif kind == "HEARTBEAT":
            # Only the flight controller's heartbeat counts. Filter by
            # system, then by the autopilot field rather than by component
            # id: MAV_AUTOPILOT_INVALID is the convention for "not a flight
            # controller", so this rejects our own companion heartbeat, a
            # GCS and a gimbal without needing to know their component ids,
            # and it keeps working if the autopilot is not on component 1.
            if msg.get_srcSystem() != self._cfg.target_system:
                return
            if getattr(msg, "autopilot", None) == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                return
            armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            with self._state_lock:
                self._last_heartbeat_time = time.monotonic()
                self._custom_mode = int(msg.custom_mode)
                self._base_mode = int(msg.base_mode)
                self._armed = armed

        elif kind == "GPS_RAW_INT":
            with self._state_lock:
                self._gps_fix_type = int(msg.fix_type)
                self._satellites = int(msg.satellites_visible)

        elif kind == "EKF_STATUS_REPORT":
            with self._state_lock:
                self._ekf_flags = int(msg.flags)

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

    @staticmethod
    def _sanitise(vector: Optional[np.ndarray]) -> Tuple[float, float, float]:
        """Coerce a vector to three finite floats.

        A NaN reaching the wire is worse than a wrong number: the packet still
        checksums, the autopilot accepts it, and the controller poisons itself.
        The estimator should never emit one, but this is the last gate before
        the serial port and it costs nothing.
        """
        if vector is None:
            return 0.0, 0.0, 0.0
        out = []
        for component in vector[:3]:
            value = float(component)
            out.append(value if math.isfinite(value) else 0.0)
        while len(out) < 3:
            out.append(0.0)
        return out[0], out[1], out[2]

    def send_velocity_setpoint(
        self,
        velocity_body: np.ndarray,
        yaw_rate: float,
        acceleration_body: Optional[np.ndarray] = None,
    ) -> bool:
        """Transmit a BODY_NED velocity (and optional acceleration) setpoint.

        MAV_FRAME_BODY_NED interprets the vector in the vehicle's forward /
        right / down axes, which is exactly the frame the estimator and the
        guidance law already work in -- no rotation into the local NED frame is
        required, and the command stays valid regardless of vehicle heading.

        The acceleration fields are only populated when the link is configured
        for it *and* a vector is supplied; otherwise their ignore bits stay set
        so the autopilot cannot act on stale zeros.
        """
        if self._master is None:
            return False

        vx, vy, vz = self._sanitise(velocity_body)
        yaw_rate_value = float(yaw_rate) if math.isfinite(float(yaw_rate)) else 0.0

        use_acceleration = self._cfg.send_acceleration and acceleration_body is not None
        if use_acceleration:
            ax, ay, az = self._sanitise(acceleration_body)
            type_mask = TYPE_MASK_VELOCITY_ACCEL_YAWRATE
        else:
            ax = ay = az = 0.0
            type_mask = TYPE_MASK_VELOCITY_YAWRATE

        elapsed_ms = int((time.monotonic() - self._boot_time) * 1e3) & 0xFFFFFFFF

        try:
            with self._lock:
                self._master.mav.set_position_target_local_ned_send(
                    elapsed_ms,
                    self._cfg.target_system,
                    self._cfg.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    type_mask,
                    0.0, 0.0, 0.0,          # position (ignored)
                    vx, vy, vz,             # velocity setpoint [m/s]
                    ax, ay, az,             # acceleration feed-forward [m/s^2]
                    0.0,                    # yaw (ignored)
                    yaw_rate_value,         # yaw rate [rad/s]
                )
            with self._state_lock:
                self._setpoints_sent += 1
            return True
        except Exception:
            with self._state_lock:
                self._transmit_failures += 1
            self._log.error("Failed to transmit velocity setpoint", exc_info=True)
            return False

    def send_hold(self) -> bool:
        """Command a zero-velocity hold -- the failsafe posture."""
        return self.send_velocity_setpoint(np.zeros(3, dtype=np.float64), 0.0)

    # -- Mode control -------------------------------------------------------

    def mode_number(self, mode_name: str) -> Optional[int]:
        """Resolve a flight-mode name to the connected vehicle's mode number."""
        return self._mode_mapping.get(mode_name.upper())

    def current_mode_number(self) -> Optional[int]:
        with self._state_lock:
            return self._custom_mode

    def _mode_name_for(self, number: Optional[int]) -> str:
        """Resolve a mode number to a name. Takes no locks by design.

        Callers that already hold ``_state_lock`` must use this rather than
        current_mode_name(), which reacquires it.
        """
        if number is None:
            return "UNKNOWN"
        for name, value in self._mode_mapping.items():
            if value == number:
                return name
        return f"MODE_{number}"

    def current_mode_name(self) -> str:
        """Human-readable current flight mode, or 'UNKNOWN'."""
        return self._mode_name_for(self.current_mode_number())

    def request_mode(self, mode_name: str) -> bool:
        """Ask the autopilot to change flight mode. Does not wait for the change.

        Sends DO_SET_MODE as a COMMAND_LONG rather than the legacy SET_MODE
        message: the command form is acknowledged, so a rejected mode change is
        visible in the log instead of vanishing.
        """
        if self._master is None:
            return False
        number = self.mode_number(mode_name)
        if number is None:
            self._log.error("Flight mode '%s' is not in the vehicle's mapping", mode_name)
            return False

        try:
            with self._lock:
                self._master.mav.command_long_send(
                    self._cfg.target_system,
                    self._cfg.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                    float(number),
                    0.0, 0.0, 0.0, 0.0, 0.0,
                )
            self._log.warning("Requested flight mode %s (%d)", mode_name.upper(), number)
            return True
        except Exception:
            self._log.error("Failed to send mode change to %s", mode_name, exc_info=True)
            return False

    def confirm_mode(self, mode_name: str, timeout_s: float) -> bool:
        """Block until HEARTBEAT reports ``mode_name``, or the timeout expires.

        Verification matters more than the request: a mode change can be refused
        for reasons the companion computer cannot see (no GPS lock for LOITER,
        no home position for RTL, a pre-arm check). Treating "sent" as "done"
        would leave the supervisor believing it had handed over when it had not.
        """
        target = self.mode_number(mode_name)
        if target is None:
            return False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.current_mode_number() == target:
                return True
            time.sleep(0.02)
        return False

    # -- Health -------------------------------------------------------------

    def heartbeat_age(self) -> float:
        """Seconds since the autopilot's last HEARTBEAT; inf if never seen."""
        with self._state_lock:
            last = self._last_heartbeat_time
        return math.inf if last <= 0.0 else (time.monotonic() - last)

    def is_armed(self) -> bool:
        with self._state_lock:
            return self._armed

    def has_position_fix(self) -> bool:
        """True when the GPS solution is good enough for LOITER or RTL.

        Fix type 3 is a 3D fix. Position-holding modes will be refused without
        one, so the supervisor must know before it picks a fallback mode --
        commanding LOITER indoors achieves nothing except burning a second of
        the failsafe budget.
        """
        with self._state_lock:
            return self._gps_fix_type >= 3 and self._satellites >= 6

    def link_status(self) -> dict:
        """Snapshot of link and vehicle state.

        The mode name is resolved after the lock is released: resolving it
        inside would reacquire ``_state_lock`` through current_mode_name() and
        deadlock, since it is a plain Lock rather than an RLock.
        """
        with self._state_lock:
            last_heartbeat = self._last_heartbeat_time
            custom_mode = self._custom_mode
            armed = self._armed
            fix_type = self._gps_fix_type
            satellites = self._satellites
            received = self._messages_received
            sent = self._setpoints_sent
            failures = self._transmit_failures

        return {
            "heartbeat_age_s": (
                math.inf if last_heartbeat <= 0.0
                else time.monotonic() - last_heartbeat
            ),
            "mode": self._mode_name_for(custom_mode),
            "armed": armed,
            "gps_fix_type": fix_type,
            "satellites": satellites,
            "messages_received": received,
            "setpoints_sent": sent,
            "transmit_failures": failures,
        }


# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Camera front-end
# ---------------------------------------------------------------------------

class CameraIntrinsics:
    """Pixel <-> bearing conversion for the calibrated lens."""

    def __init__(self, config: IntrinsicsConfig) -> None:
        self._cfg = config
        self.K = np.array(
            [[config.fx, 0.0, config.cx],
             [0.0, config.fy, config.cy],
             [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.dist = np.array(config.distortion, dtype=np.float64).reshape(1, 5)
        self._has_distortion = bool(np.any(np.abs(self.dist) > 1e-12))

    @classmethod
    def from_file(cls, path: str) -> "CameraIntrinsics":
        """Load intrinsics from a ROS ``camera_info`` YAML or a plain JSON dict.

        Accepts both the ROS layout (``camera_matrix.data`` /
        ``distortion_coefficients.data``) and a flat ``{"fx": ..., ...}`` dict,
        so a calibration produced by ``camera_calibration`` drops straight in.
        """
        with open(path, "r", encoding="utf-8") as handle:
            text = handle.read()

        try:
            raw = json.loads(text)
        except json.JSONDecodeError:
            try:
                import yaml  # PyYAML ships with every ROS 2 desktop install
            except ImportError as exc:  # pragma: no cover - environment issue
                raise ValueError(
                    f"{path} is not JSON and PyYAML is unavailable to parse it"
                ) from exc
            raw = yaml.safe_load(text)

        if "camera_matrix" in raw:
            matrix = list(raw["camera_matrix"]["data"])
            coeffs = list(raw.get("distortion_coefficients", {}).get("data", []))
            fx, cx, fy, cy = matrix[0], matrix[2], matrix[4], matrix[5]
        else:
            fx, fy = float(raw["fx"]), float(raw["fy"])
            cx, cy = float(raw["cx"]), float(raw["cy"])
            coeffs = list(raw.get("distortion", []))

        coeffs = (coeffs + [0.0] * 5)[:5]
        config = IntrinsicsConfig(
            fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy),
            distortion=tuple(float(c) for c in coeffs),  # type: ignore[arg-type]
        )
        return cls(config)

    def scaled(self, factor: float) -> "CameraIntrinsics":
        """Intrinsics for an image resized by ``factor`` (distortion is scale-free)."""
        cfg = IntrinsicsConfig(
            fx=self._cfg.fx * factor,
            fy=self._cfg.fy * factor,
            cx=self._cfg.cx * factor,
            cy=self._cfg.cy * factor,
            distortion=self._cfg.distortion,
            centroid_sigma_px=self._cfg.centroid_sigma_px,
            extent_sigma_px=self._cfg.extent_sigma_px,
        )
        return CameraIntrinsics(cfg)

    def pixel_to_ray(self, u: float, v: float) -> np.ndarray:
        """Undistorted unit ray in the REP-103 optical frame (+x right, +y down, +z fwd)."""
        if self._has_distortion:
            src = np.array([[[float(u), float(v)]]], dtype=np.float64)
            norm = cv2.undistortPoints(src, self.K, self.dist)
            xn, yn = float(norm[0, 0, 0]), float(norm[0, 0, 1])
        else:
            xn = (float(u) - self._cfg.cx) / self._cfg.fx
            yn = (float(v) - self._cfg.cy) / self._cfg.fy

        ray = np.array([xn, yn, 1.0], dtype=np.float64)
        return ray / np.linalg.norm(ray)

    @property
    def focal_mean(self) -> float:
        return 0.5 * (self._cfg.fx + self._cfg.fy)


# ---------------------------------------------------------------------------
# Frame transport: single-slot, latest-wins
# ---------------------------------------------------------------------------

@dataclass
class Frame:
    """One captured image with the epoch at which its photons landed."""

    sequence: int
    capture_time: float      # Exposure epoch, already latency-corrected [s]
    grab_time: float         # When read() returned, same clock domain [s]
    image: np.ndarray        # BGR or GRAY, full sensor resolution


class LatestFrameSlot:
    """A one-deep mailbox that always holds the newest frame.

    A Queue(maxsize=1) is the wrong primitive here: the producer would block
    (stalling capture) or raise (forcing the producer to handle drops), and
    either way the consumer can end up reading a frame that is already two
    periods stale. Overwrite-in-place gives the consumer the freshest frame
    available at the instant it asks, which is exactly what an estimator wants.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ready = threading.Condition(self._lock)
        self._frame: Optional[Frame] = None
        self._closed = False
        self.dropped = 0
        self.published = 0

    def put(self, frame: Frame) -> None:
        """Publish a frame, discarding any unconsumed predecessor. Never blocks."""
        with self._ready:
            if self._frame is not None:
                self.dropped += 1
            self._frame = frame
            self.published += 1
            self._ready.notify()

    def take(self, timeout: float) -> Optional[Frame]:
        """Wait up to ``timeout`` for a frame; returns None on timeout or close."""
        with self._ready:
            if self._frame is None and not self._closed:
                self._ready.wait(timeout)
            frame, self._frame = self._frame, None
            return frame

    def peek_nowait(self) -> Optional[Frame]:
        """Non-blocking read that leaves the slot occupied (for preview/telemetry)."""
        with self._lock:
            return self._frame

    def close(self) -> None:
        with self._ready:
            self._closed = True
            self._ready.notify_all()


# ---------------------------------------------------------------------------
# Capture backends
# ---------------------------------------------------------------------------

def build_gstreamer_pipeline(config: CameraConfig) -> str:
    """GStreamer launch string for the configured Arducam backend.

    ``drop=true max-buffers=1 sync=false`` on the appsink is load-bearing: it
    makes GStreamer itself discard stale buffers rather than queueing them, so
    the newest frame is always the one waiting for us.
    """
    if config.backend == "argus":
        # Argus path: ISP-processed NVMM buffers, for IMX-series GS modules.
        return (
            f"nvarguscamerasrc sensor-id={config.sensor_id} ! "
            f"video/x-raw(memory:NVMM),width={config.width},height={config.height},"
            f"framerate={int(round(config.frame_rate_hz))}/1 ! "
            f"nvvidconv flip-method={config.flip_method} ! "
            f"video/x-raw,format=BGRx ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink drop=true max-buffers=1 sync=false"
        )

    if config.backend == "v4l2":
        # Direct V4L2 path: Arducam's mono GS drivers (OV9281/OV2311) bypass
        # Argus entirely and expose a plain V4L2 node.
        return (
            f"v4l2src device={config.device} io-mode=2 ! "
            f"video/x-raw,width={config.width},height={config.height},"
            f"framerate={int(round(config.frame_rate_hz))}/1 ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink drop=true max-buffers=1 sync=false"
        )

    raise ValueError(f"No GStreamer pipeline for backend '{config.backend}'")


class FrameSource(ABC):
    """Minimal capture interface, so the synthetic bench source is a peer."""

    @abstractmethod
    def open(self) -> None:
        ...

    @abstractmethod
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        ...

    @abstractmethod
    def release(self) -> None:
        ...


class GStreamerFrameSource(FrameSource):
    """OpenCV VideoCapture over the Jetson GStreamer pipeline."""

    def __init__(self, config: CameraConfig, logger: logging.Logger) -> None:
        self._cfg = config
        self._log = logger
        self._cap: Optional[cv2.VideoCapture] = None

    def open(self) -> None:
        pipeline = build_gstreamer_pipeline(self._cfg)
        self._log.info("Opening capture: %s", pipeline)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError(
                f"Failed to open the {self._cfg.backend} pipeline. Check that the "
                f"sensor is detected, that OpenCV was built with GStreamer "
                f"support, and that no other process holds the camera."
            )
        # Belt and braces: even with appsink drop=true, ask OpenCV for a shallow
        # internal buffer so read() cannot serve us a backlog.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cap = cap

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        if self._cap is None:
            return False, None
        return self._cap.read()

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class SyntheticFrameSource(FrameSource):
    """Software camera that renders a moving target over a drifting background.

    Not a stub: it produces real images at the configured rate with a known
    ground-truth trajectory, which is how the detector, the timing behaviour and
    the EKF hand-off get exercised on a desk with no sensor attached.
    """

    def __init__(
        self,
        config: CameraConfig,
        intrinsics: IntrinsicsConfig,
        target: TargetPrior,
        seed: int = 0,
    ) -> None:
        self._cfg = config
        self._intr = intrinsics
        self._target = target
        self._rng = np.random.default_rng(seed)
        self._t0 = time.monotonic()
        self._background: Optional[np.ndarray] = None
        self._noise_bank: List[np.ndarray] = []
        self._frame_index = 0
        self._next_release = time.monotonic()
        self.truth_position: Optional[np.ndarray] = None   # optical frame [m]

    def open(self) -> None:
        h, w = self._cfg.height, self._cfg.width
        # Static texture so the ego-motion estimator has features to lock onto.
        base = self._rng.integers(60, 120, size=(h // 8, w // 8), dtype=np.uint8)
        self._background = cv2.cvtColor(
            cv2.resize(base, (w, h), interpolation=cv2.INTER_LINEAR), cv2.COLOR_GRAY2BGR
        )
        # Drawing 2.7 M Gaussians per frame costs more than the frame period and
        # would make the bench source, not the pipeline, the bottleneck. A small
        # bank of precomputed noise fields is indistinguishable for our purposes.
        self._noise_bank = [
            self._rng.normal(0.0, 2.5, size=(h, w, 3)).astype(np.int16)
            for _ in range(8)
        ]

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        assert self._background is not None
        t = time.monotonic() - self._t0
        h, w = self._cfg.height, self._cfg.width

        # Camera pans slowly; the target crosses and closes.
        pan_px = 40.0 * math.sin(0.30 * t)
        tilt_px = 18.0 * math.sin(0.21 * t)
        M = np.array([[1.0, 0.0, pan_px], [0.0, 1.0, tilt_px]], dtype=np.float64)
        frame = cv2.warpAffine(self._background, M, (w, h), borderMode=cv2.BORDER_REFLECT)

        z = 26.0 - 1.1 * t                      # closing range [m]
        x = 3.0 * math.sin(0.55 * t)            # lateral drift [m]
        y = -1.2 + 0.6 * math.sin(0.4 * t)
        if z < 3.0:
            self._t0 = time.monotonic()         # loop the run
            z = 26.0
        self.truth_position = np.array([x, y, z], dtype=np.float64)

        u = self._intr.cx + self._intr.fx * x / z
        v = self._intr.cy + self._intr.fy * y / z
        radius = max(2.0, 0.5 * self._intr.fx * self._target.width_m / z)
        cv2.circle(frame, (int(round(u)), int(round(v))), int(round(radius)),
                   (235, 235, 235), -1, lineType=cv2.LINE_AA)

        self._frame_index += 1
        noise = self._noise_bank[self._frame_index % len(self._noise_bank)]
        frame = cv2.add(frame, noise, dtype=cv2.CV_8U)

        # Pace to the nominal frame period, accounting for render time so the
        # source actually delivers the configured rate.
        target = self._next_release + self._cfg.frame_period_s
        now = time.monotonic()
        if target > now:
            time.sleep(target - now)
            self._next_release = target
        else:
            self._next_release = now
        return True, frame

    def release(self) -> None:
        self._background = None


# ---------------------------------------------------------------------------
# Capture thread
# ---------------------------------------------------------------------------

class AsyncCameraCapture:
    """Owns the capture thread and publishes into a :class:`LatestFrameSlot`.

    The thread does the minimum possible work per iteration: read, stamp,
    publish. Every millisecond spent here is a millisecond of jitter injected
    into the timestamp, so colour conversion, scaling and detection all belong
    downstream.
    """

    def __init__(
        self,
        config: CameraConfig,
        source: FrameSource,
        logger: logging.Logger,
        clock: Callable[[], float] = time.monotonic,
        frame_callback: Optional[Callable[[], None]] = None,
    ) -> None:
        self._cfg = config
        self._source = source
        self._log = logger
        self._clock = clock
        # Invoked on the capture thread immediately after each frame is
        # published. Intended for a safety watchdog, so it must be trivial --
        # anything slow here becomes timestamp jitter on the next frame.
        self._frame_callback = frame_callback

        self.slot = LatestFrameSlot()
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()

        self._stats_lock = threading.Lock()
        self._sequence = 0
        self._last_grab_time = 0.0
        self._read_failures = 0
        self._interval_history: Deque[float] = deque(maxlen=240)

    # -- Lifecycle ----------------------------------------------------------

    def start(self) -> None:
        self._source.open()
        self._running.set()
        self._thread = threading.Thread(
            target=self._capture_loop, name="cam-capture", daemon=True
        )
        self._thread.start()
        self._log.info(
            "Capture thread started (%dx%d @ %.0f Hz, backend=%s)",
            self._cfg.width, self._cfg.height, self._cfg.frame_rate_hz, self._cfg.backend,
        )

    def stop(self) -> None:
        self._running.clear()
        self.slot.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                self._log.warning("Capture thread did not exit within 2 s")
            self._thread = None
        self._source.release()
        # Clear the freshness stamp: a stopped pipeline must report unhealthy
        # immediately rather than looking alive for one more staleness window.
        with self._stats_lock:
            self._last_grab_time = 0.0
        self._log.info("Capture thread stopped")

    # -- Thread body --------------------------------------------------------

    def _apply_realtime_priority(self) -> None:
        """Best-effort SCHED_FIFO placement for the capture thread."""
        priority = self._cfg.realtime_priority
        if priority is None:
            return
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
            self._log.info("Capture thread running SCHED_FIFO at priority %d", priority)
        except (PermissionError, OSError, AttributeError) as exc:
            # Entirely expected without CAP_SYS_NICE. The pipeline is correct
            # either way; it just gets a little more timestamp jitter.
            self._log.warning(
                "Could not set real-time priority (%s); running SCHED_OTHER", exc
            )

    def _capture_loop(self) -> None:
        self._apply_realtime_priority()
        latency = self._cfg.sensor_latency_s
        consecutive_failures = 0

        while self._running.is_set():
            ok, image = self._source.read()
            grab_time = self._clock()

            if not ok or image is None:
                consecutive_failures += 1
                with self._stats_lock:
                    self._read_failures += 1
                # Back off gently: a hammering retry loop on a wedged pipeline
                # starves the detector threads of CPU.
                if consecutive_failures <= 3:
                    continue
                self._log.warning(
                    "Capture read failed %d times consecutively", consecutive_failures
                )
                time.sleep(min(0.05 * consecutive_failures, 0.5))
                continue

            consecutive_failures = 0
            with self._stats_lock:
                self._sequence += 1
                sequence = self._sequence
                if self._last_grab_time > 0.0:
                    self._interval_history.append(grab_time - self._last_grab_time)
                self._last_grab_time = grab_time

            # The stamp that matters: when the photons landed, not when Python
            # got the buffer. Everything downstream inherits this value.
            self.slot.put(
                Frame(
                    sequence=sequence,
                    capture_time=grab_time - latency,
                    grab_time=grab_time,
                    image=image,
                )
            )

            if self._frame_callback is not None:
                try:
                    self._frame_callback()
                except Exception:
                    self._log.error("Frame callback raised", exc_info=True)

    # -- Health -------------------------------------------------------------

    def is_streaming(self, now: Optional[float] = None) -> bool:
        """True if a frame arrived recently enough to call the stream healthy."""
        now = self._clock() if now is None else now
        with self._stats_lock:
            last = self._last_grab_time
        if last <= 0.0:
            return False
        return (now - last) < self._cfg.stale_frame_periods * self._cfg.frame_period_s

    def statistics(self) -> dict:
        with self._stats_lock:
            intervals = list(self._interval_history)
            frames = self._sequence
            failures = self._read_failures
        if intervals:
            arr = np.asarray(intervals)
            measured_hz = 1.0 / max(float(np.mean(arr)), EPS)
            jitter_ms = float(np.std(arr)) * 1e3
            worst_ms = float(np.max(arr)) * 1e3
        else:
            measured_hz = jitter_ms = worst_ms = 0.0
        return {
            "frames": frames,
            "read_failures": failures,
            "measured_hz": measured_hz,
            "interval_jitter_ms": jitter_ms,
            "worst_interval_ms": worst_ms,
            "slot_dropped": self.slot.dropped,
        }


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------

@dataclass
class Detection:
    """A target found in one frame, in full-resolution pixel coordinates."""

    u: float                 # Centroid column [px]
    v: float                 # Centroid row [px]
    width_px: float          # Apparent width -- the range observable
    height_px: float
    area_px: float
    confidence: float        # 0..1, used only for logging and gating


class TargetDetector(ABC):
    """Frame -> optional detection. Swap in a CNN by implementing this."""

    @abstractmethod
    def detect(self, gray_full: np.ndarray) -> Optional[Detection]:
        """Full-resolution grayscale frame -> detection in full-resolution pixels."""
        ...

    @abstractmethod
    def reset(self) -> None:
        ...


class EgoMotionBlobDetector(TargetDetector):
    """Motion detector that first cancels the camera's own motion.

    Plain frame differencing is useless on a moving airframe: every edge in the
    scene lights up. This estimates the image motion from sparse LK flow between
    a reference frame and the current one, warps the reference forward, and
    differences what is left. Static structure cancels; an independently moving
    target does not.

    The reference is deliberately several frames old (``temporal_stride``)
    rather than the immediately preceding frame -- at 60 Hz consecutive frames
    are nearly identical, and a target's per-frame displacement is far too small
    to clear the sensor noise floor.

    Once a target is held, the search collapses to a window around the previous
    detection -- roughly an order of magnitude cheaper than the full frame, and
    far less prone to latching onto clutter at the frame edge.
    """

    def __init__(self, config: DetectorConfig, logger: logging.Logger) -> None:
        self._cfg = config
        self._log = logger
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (config.morph_kernel_px, config.morph_kernel_px)
        )
        self.reset()

    def reset(self) -> None:
        # Ring of recent downscaled frames; the oldest is the differencing
        # reference. Depth is stride + 1 so the reference is exactly `stride`
        # frames old once the ring has filled.
        self._history: Deque[np.ndarray] = deque(maxlen=max(1, self._cfg.temporal_stride) + 1)
        # Kept in detection (downscaled) coordinates: it only ever feeds the ROI
        # window, which is computed against the downscaled frame.
        self._last_detection: Optional[Detection] = None
        self._misses = 0

    # -- Ego-motion ---------------------------------------------------------

    def _estimate_ego_motion(
        self, previous: np.ndarray, current: np.ndarray
    ) -> Optional[np.ndarray]:
        """2x3 partial affine mapping ``previous`` onto ``current``, or None."""
        corners = cv2.goodFeaturesToTrack(
            previous,
            maxCorners=self._cfg.max_corners,
            qualityLevel=self._cfg.corner_quality,
            minDistance=self._cfg.min_corner_distance_px,
        )
        if corners is None or len(corners) < self._cfg.min_inliers:
            return None

        window = (self._cfg.lk_window_px, self._cfg.lk_window_px)
        tracked, status, _ = cv2.calcOpticalFlowPyrLK(
            previous, current, corners, None,
            winSize=window, maxLevel=self._cfg.lk_levels,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        if tracked is None or status is None:
            return None

        keep = status.reshape(-1).astype(bool)
        src, dst = corners[keep], tracked[keep]
        if len(src) < self._cfg.min_inliers:
            return None

        # RANSAC so the target's own motion cannot bias the ego estimate.
        matrix, inliers = cv2.estimateAffinePartial2D(
            src, dst, method=cv2.RANSAC, ransacReprojThreshold=2.0
        )
        if matrix is None or inliers is None or int(inliers.sum()) < self._cfg.min_inliers:
            return None
        return matrix

    # -- Blob extraction ----------------------------------------------------

    def _residual_mask(self, warped_prev: np.ndarray, current: np.ndarray) -> np.ndarray:
        residual = cv2.absdiff(current, warped_prev)
        residual = cv2.GaussianBlur(residual, (5, 5), 0)

        mean, stddev = cv2.meanStdDev(residual)
        threshold = float(mean[0, 0] + self._cfg.threshold_sigma * stddev[0, 0])
        threshold = max(threshold, float(self._cfg.min_absolute_threshold))

        _, mask = cv2.threshold(residual, threshold, 255, cv2.THRESH_BINARY)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)

    def _select_blob(
        self, mask: np.ndarray, offset: Tuple[int, int], frame_area: float
    ) -> Optional[Detection]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        max_area = self._cfg.max_blob_area_fraction * frame_area
        best: Optional[Detection] = None
        best_score = -math.inf

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self._cfg.min_blob_area_px or area > max_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 2 or h < 2:
                continue

            # Prefer compact blobs: a coherent object fills its bounding box,
            # a residual smear from imperfect ego-motion cancellation does not.
            fill = area / float(w * h)
            aspect = min(w, h) / float(max(w, h))
            score = area * (0.5 + fill) * (0.5 + aspect)

            if score > best_score:
                best_score = score
                best = Detection(
                    u=x + 0.5 * w + offset[0],
                    v=y + 0.5 * h + offset[1],
                    width_px=float(w),
                    height_px=float(h),
                    area_px=area,
                    confidence=float(np.clip(fill * aspect, 0.0, 1.0)),
                )
        return best

    # -- Silhouette refinement ----------------------------------------------

    def _refine_extent(self, gray: np.ndarray, blob: Detection) -> Detection:
        """Re-measure the target's extent from the intensity image.

        The motion residual is a locator, not a sizer. Differencing an object
        that moved between frames yields a crescent whose bounding box is
        inflated by the inter-frame displacement -- and once the displacement
        exceeds the object's own width, it splits into two disjoint crescents
        and the box collapses onto one of them. Measured against ground truth
        the residual box runs ~1.4x the true silhouette on average with large
        scatter, which the size-scale model would turn directly into a ~30%
        range bias, biased *low* (an inflated width reads as a closer target).

        So the residual supplies the search location only. The extent is
        re-derived by segmenting the object in the current frame: Otsu inside a
        local window, both polarities tried (targets may be brighter or darker
        than their background), keeping the compact component that contains the
        residual centroid. Falls back to the residual box if nothing qualifies.
        """
        height, width = gray.shape[:2]
        span = self._cfg.extent_window_factor * max(blob.width_px, blob.height_px)
        span = max(span, float(self._cfg.min_extent_window_px))

        x0 = int(max(0, math.floor(blob.u - span)))
        y0 = int(max(0, math.floor(blob.v - span)))
        x1 = int(min(width, math.ceil(blob.u + span)))
        y1 = int(min(height, math.ceil(blob.v + span)))
        if (x1 - x0) < 6 or (y1 - y0) < 6:
            return blob

        window = gray[y0:y1, x0:x1]
        seed_x = int(round(blob.u)) - x0
        seed_y = int(round(blob.v)) - y0
        seed_x = int(np.clip(seed_x, 0, window.shape[1] - 1))
        seed_y = int(np.clip(seed_y, 0, window.shape[0] - 1))

        window_area = float(window.shape[0] * window.shape[1])
        best: Optional[Detection] = None

        for invert in (False, True):
            flags = cv2.THRESH_OTSU | (cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY)
            _, mask = cv2.threshold(window, 0, 255, flags)
            count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
            if count < 2:
                continue

            label = int(labels[seed_y, seed_x])
            if label == 0:
                continue   # the seed landed on background for this polarity

            area = float(stats[label, cv2.CC_STAT_AREA])
            if area < self._cfg.min_blob_area_px:
                continue
            if area > self._cfg.max_extent_area_fraction * window_area:
                continue   # this polarity selected the background

            bias = self._cfg.extent_bias_px
            candidate = Detection(
                u=float(centroids[label, 0]) + x0,
                v=float(centroids[label, 1]) + y0,
                width_px=max(float(stats[label, cv2.CC_STAT_WIDTH]) - bias, 1.0),
                height_px=max(float(stats[label, cv2.CC_STAT_HEIGHT]) - bias, 1.0),
                area_px=area,
                confidence=blob.confidence,
            )
            # Between two viable polarities the smaller component is the object;
            # the larger one has bled into surrounding structure.
            if best is None or candidate.area_px < best.area_px:
                best = candidate

        return best if best is not None else blob

    # -- Entry point --------------------------------------------------------

    def detect(self, gray_full: np.ndarray) -> Optional[Detection]:
        """Locate the target and return it in full-resolution pixel coordinates.

        Motion detection runs downscaled -- it is the expensive stage and does
        not need the resolution. Silhouette refinement then runs on the full
        frame, because it *does*: a one-pixel bounding-box error at half scale
        is two full-resolution pixels, which on a 20 px target is a 10% range
        error straight into the size-scale solution.
        """
        scale = self._cfg.detect_scale
        if abs(scale - 1.0) > 1e-6:
            gray = cv2.resize(gray_full, None, fx=scale, fy=scale,
                              interpolation=cv2.INTER_AREA)
        else:
            gray = gray_full

        reference = self._history[0] if len(self._history) == self._history.maxlen else None
        self._history.append(gray)
        if reference is None or reference.shape != gray.shape:
            return None
        previous = reference

        height, width = gray.shape[:2]
        frame_area = float(height * width)

        # Restrict the search to a window around the last hit while tracking.
        roi = self._region_of_interest(width, height)
        if roi is not None:
            x0, y0, x1, y1 = roi
            previous_roi, current_roi = previous[y0:y1, x0:x1], gray[y0:y1, x0:x1]
            offset = (x0, y0)
        else:
            previous_roi, current_roi = previous, gray
            offset = (0, 0)

        matrix = self._estimate_ego_motion(previous_roi, current_roi)
        if matrix is None:
            # Not enough texture to solve ego-motion. Differencing anyway would
            # produce garbage, so declare a miss and let the track coast.
            self._register_miss()
            return None

        warped = cv2.warpAffine(
            previous_roi, matrix,
            (current_roi.shape[1], current_roi.shape[0]),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE,
        )
        mask = self._residual_mask(warped, current_roi)
        detection = self._select_blob(mask, offset, frame_area)

        if detection is None:
            self._register_miss()
            return None

        # ROI tracking stays in detection space; the caller gets full resolution.
        self._last_detection = detection
        self._misses = 0

        inv = 1.0 / scale if abs(scale - 1.0) > 1e-6 else 1.0
        full = Detection(
            u=detection.u * inv,
            v=detection.v * inv,
            width_px=detection.width_px * inv,
            height_px=detection.height_px * inv,
            area_px=detection.area_px * inv * inv,
            confidence=detection.confidence,
        )
        if self._cfg.refine_extent:
            full = self._refine_extent(gray_full, full)
        return full

    def _region_of_interest(
        self, width: int, height: int
    ) -> Optional[Tuple[int, int, int, int]]:
        last = self._last_detection
        if last is None or self._misses > self._cfg.max_track_misses:
            return None

        span = self._cfg.roi_margin_factor * max(last.width_px, last.height_px)
        span = max(span, float(self._cfg.min_roi_px))
        x0 = int(max(0, math.floor(last.u - span)))
        y0 = int(max(0, math.floor(last.v - span)))
        x1 = int(min(width, math.ceil(last.u + span)))
        y1 = int(min(height, math.ceil(last.v + span)))
        if (x1 - x0) < 16 or (y1 - y0) < 16:
            return None
        return x0, y0, x1, y1

    def _register_miss(self) -> None:
        self._misses += 1
        if self._misses > self._cfg.max_track_misses:
            self._last_detection = None


# ---------------------------------------------------------------------------
# Detection -> polar measurement
# ---------------------------------------------------------------------------

class MonocularRangeModel:
    """Turns a pixel detection into a polar measurement with an honest covariance."""

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        target: TargetPrior,
        config: IntrinsicsConfig,
    ) -> None:
        self._intrinsics = intrinsics
        self._target = target
        self._cfg = config

    def range_from_extent(self, width_px: float) -> float:
        """Size-scale range: ``r = f * W_true / w_px``."""
        return self._intrinsics.focal_mean * self._target.width_m / max(width_px, EPS)

    def bearing_vector_body(self, u: float, v: float) -> np.ndarray:
        """Unit LOS vector in BODY FRD for a pixel."""
        return R_CAM_TO_BODY @ self._intrinsics.pixel_to_ray(u, v)

    def measurement_covariance(self, rng: float, width_px: float) -> np.ndarray:
        """Diagonal R for ``[azimuth, elevation, range]``.

        Bearing:  sigma_ang = sigma_centroid_px / f -- a pure pixel-to-angle
        conversion, and the reason the bearing channel is worth ~0.5 mrad.

        Range: differentiating r = fW/w gives two independent contributions,

            dr/dw = -r/w      -> sigma_r_pixel = r * sigma_w / w
                              (equivalently r^2 * sigma_w / (fW): the quadratic
                              growth the estimator's noise model assumes)
            dr/dW =  r/W      -> sigma_r_prior = r * sigma_W / W

        The second term is the one people forget. Monocular range is only as
        good as the assumed target size, and against a non-cooperative target
        that prior is soft. Folding it in keeps the filter from trusting a
        confidently wrong range; it also means range naturally de-weights itself
        at distance, leaving the bearing channel to carry the solution.
        """
        sigma_bearing = self._cfg.centroid_sigma_px / max(self._intrinsics.focal_mean, EPS)

        sigma_from_pixels = rng * self._cfg.extent_sigma_px / max(width_px, EPS)
        sigma_from_prior = rng * self._target.width_sigma_m / max(self._target.width_m, EPS)
        sigma_range = math.hypot(sigma_from_pixels, sigma_from_prior)

        return np.diag(
            [sigma_bearing ** 2, sigma_bearing ** 2, sigma_range ** 2]
        ).astype(np.float64)

    def to_measurement(
        self, detection: Detection, capture_time: float
    ) -> Tuple[Measurement, np.ndarray]:
        """Build the EKF measurement and the BODY-frame position it implies."""
        rng = self.range_from_extent(detection.width_px)
        los_body = self.bearing_vector_body(detection.u, detection.v)
        position_body = los_body * rng

        polar = cartesian_to_polar(position_body)
        # Range comes from the size model, not from the bearing reconstruction;
        # they agree to machine precision here, but assigning it explicitly
        # keeps the intent obvious.
        polar[2] = rng

        covariance = self.measurement_covariance(rng, detection.width_px)
        return Measurement(capture_time, polar, covariance), position_body


# ---------------------------------------------------------------------------
# Measurement hand-off
# ---------------------------------------------------------------------------

class MeasurementQueue:
    """Bounded, drop-oldest queue between detector threads and the EKF thread.

    ``drain()`` is the only consumer entry point and it never blocks: the
    control loop takes whatever is ready and moves on. Overflow drops the
    *oldest* entry, because under sustained overload the freshest observation is
    the one worth keeping.
    """

    def __init__(self, depth: int) -> None:
        self._lock = threading.Lock()
        self._items: Deque[Measurement] = deque(maxlen=max(1, depth))
        self.dropped = 0
        self.delivered = 0

    def put(self, measurement: Measurement) -> None:
        with self._lock:
            if len(self._items) == self._items.maxlen:
                self.dropped += 1
            self._items.append(measurement)

    def drain(self) -> List[Measurement]:
        """Remove and return everything queued, oldest first. Non-blocking."""
        with self._lock:
            items = list(self._items)
            self._items.clear()
            self.delivered += len(items)
        # Sort defensively: with more than one detector thread, completion order
        # is not capture order. The EKF tolerates either, but feeding it ordered
        # samples avoids needless rollbacks.
        items.sort(key=lambda m: m.capture_time)
        return items

    def __len__(self) -> int:
        with self._lock:
            return len(self._items)


# ---------------------------------------------------------------------------
# Detector worker
# ---------------------------------------------------------------------------

class DetectionWorker:
    """Thread: pull the newest frame, detect, publish a polar measurement."""

    def __init__(
        self,
        name: str,
        config: FrontendConfig,
        slot: LatestFrameSlot,
        detector: TargetDetector,
        range_model: MonocularRangeModel,
        output: MeasurementQueue,
        logger: logging.Logger,
    ) -> None:
        self._name = name
        self._cfg = config
        self._slot = slot
        self._detector = detector
        self._range_model = range_model
        self._output = output
        self._log = logger

        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()

        self._stats_lock = threading.Lock()
        self._processed = 0
        self._hits = 0
        self._range_rejects = 0
        self._latency_history: Deque[float] = deque(maxlen=240)
        self._compute_history: Deque[float] = deque(maxlen=240)
        self._last_position: Optional[np.ndarray] = None

    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name=self._name, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # -- Thread body --------------------------------------------------------

    def _prepare(self, image: np.ndarray) -> np.ndarray:
        """Grayscale conversion, the only preprocessing the detector needs.

        Downscaling belongs to the detector: it alone knows which stages want
        reduced resolution and which need the full frame.
        """
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image

    def _loop(self) -> None:
        timeout = 4.0 * self._cfg.camera.frame_period_s

        while self._running.is_set():
            frame = self._slot.take(timeout)
            if frame is None:
                continue

            started = time.monotonic()
            gray = self._prepare(frame.image)
            detection = self._detector.detect(gray)
            elapsed = time.monotonic() - started

            with self._stats_lock:
                self._processed += 1
                self._compute_history.append(elapsed)

            if detection is None:
                continue

            measurement, position = self._range_model.to_measurement(
                detection, frame.capture_time
            )
            rng = float(measurement.polar[2])
            if not (self._cfg.min_range_m <= rng <= self._cfg.max_range_m):
                with self._stats_lock:
                    self._range_rejects += 1
                continue

            self._output.put(measurement)
            with self._stats_lock:
                self._hits += 1
                self._last_position = position
                self._latency_history.append(time.monotonic() - frame.capture_time)

    # -- Telemetry ----------------------------------------------------------

    def statistics(self) -> dict:
        with self._stats_lock:
            processed, hits = self._processed, self._hits
            rejects = self._range_rejects
            latency = list(self._latency_history)
            compute = list(self._compute_history)
            position = None if self._last_position is None else self._last_position.copy()
        return {
            "processed": processed,
            "hits": hits,
            "range_rejects": rejects,
            "hit_rate": (100.0 * hits / processed) if processed else 0.0,
            "detect_ms": (float(np.mean(compute)) * 1e3) if compute else 0.0,
            "detect_ms_p99": (float(np.percentile(compute, 99)) * 1e3) if compute else 0.0,
            "end_to_end_ms": (float(np.mean(latency)) * 1e3) if latency else 0.0,
            "last_position_body": position,
        }


# ---------------------------------------------------------------------------
# Front-end orchestration
# ---------------------------------------------------------------------------

class CameraMeasurementSource:
    """Capture + detection pipeline exposing a non-blocking measurement drain.

    This is the whole front-end behind one object. Construct it, ``start()`` it,
    and call ``drain()`` from the control loop; nothing it does can stall the
    caller.
    """

    def __init__(
        self,
        config: FrontendConfig,
        logger: Optional[logging.Logger] = None,
        clock: Callable[[], float] = time.monotonic,
        source: Optional[FrameSource] = None,
        frame_callback: Optional[Callable[[], None]] = None,
    ) -> None:
        self._cfg = config
        self._log = logger or logging.getLogger("gnc.camera")
        self._clock = clock

        self.intrinsics = CameraIntrinsics(config.intrinsics)
        self._range_model = MonocularRangeModel(
            self.intrinsics, config.target, config.intrinsics
        )

        if source is None:
            source = self._build_source()
        self._capture = AsyncCameraCapture(
            config.camera, source, self._log, clock, frame_callback=frame_callback
        )

        self.measurements = MeasurementQueue(config.measurement_queue_depth)
        self._workers: List[DetectionWorker] = []
        for index in range(max(1, config.detector_threads)):
            self._workers.append(
                DetectionWorker(
                    name=f"cam-detect-{index}",
                    config=config,
                    slot=self._capture.slot,
                    detector=EgoMotionBlobDetector(config.detector, self._log),
                    range_model=self._range_model,
                    output=self.measurements,
                    logger=self._log,
                )
            )

    def _build_source(self) -> FrameSource:
        if self._cfg.camera.backend == "synthetic":
            return SyntheticFrameSource(
                self._cfg.camera, self._cfg.intrinsics, self._cfg.target
            )
        return GStreamerFrameSource(self._cfg.camera, self._log)

    # -- Lifecycle ----------------------------------------------------------

    def start(self) -> None:
        self._capture.start()
        for worker in self._workers:
            worker.start()
        self._log.info("Front-end running with %d detector thread(s)", len(self._workers))

    def stop(self) -> None:
        for worker in self._workers:
            worker.stop()
        self._capture.stop()
        self._log.info("Front-end stopped")

    # -- Consumer API -------------------------------------------------------

    def drain(self) -> List[Measurement]:
        """Every measurement produced since the last call, oldest first."""
        return self.measurements.drain()

    def is_healthy(self, now: Optional[float] = None) -> bool:
        """True while frames are still arriving from the sensor."""
        return self._capture.is_streaming(now)

    def statistics(self) -> dict:
        stats = {"capture": self._capture.statistics()}
        for worker in self._workers:
            stats[worker._name] = worker.statistics()
        stats["queue"] = {
            "pending": len(self.measurements),
            "delivered": self.measurements.delivered,
            "dropped": self.measurements.dropped,
        }
        return stats

    def format_telemetry(self) -> str:
        stats = self.statistics()
        capture = stats["capture"]
        detector = stats.get("cam-detect-0", {})
        return (
            f"capture {capture['measured_hz']:5.1f} Hz "
            f"(jitter {capture['interval_jitter_ms']:4.1f} ms, "
            f"worst {capture['worst_interval_ms']:5.1f} ms, "
            f"dropped {capture['slot_dropped']}) | "
            f"detect {detector.get('detect_ms', 0.0):5.2f} ms "
            f"(p99 {detector.get('detect_ms_p99', 0.0):5.2f}) | "
            f"hit {detector.get('hit_rate', 0.0):5.1f}% | "
            f"end-to-end {detector.get('end_to_end_ms', 0.0):5.1f} ms | "
            f"queue {stats['queue']['pending']} "
            f"(dropped {stats['queue']['dropped']})"
        )


# ---------------------------------------------------------------------------
# Direct in-process bridge to the EKF
# ---------------------------------------------------------------------------

@dataclass
class PumpResult:
    """Outcome of one :meth:`CameraEkfBridge.pump` call."""

    drained: int     # measurements taken off the queue
    fused: int       # of those, how many the filter accepted


class CameraEkfBridge:
    """Pumps camera measurements straight into the EKF's polar update.

    This is the zero-copy, zero-middleware path: no ROS serialisation, no
    inter-process hop, just the detector thread's output handed to
    ``LatencyCompensatedEKF.fuse`` on the control thread. Call :meth:`pump`
    once per control cycle, before the filter is stepped to the current epoch.
    """

    def __init__(
        self,
        source: CameraMeasurementSource,
        ekf: LatencyCompensatedEKF,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._source = source
        self._ekf = ekf
        self._log = logger or logging.getLogger("gnc.bridge")
        self.fused = 0
        self.rejected = 0

    def pump(self, body_rate: np.ndarray,
             own_accel: Optional[np.ndarray] = None) -> "PumpResult":
        """Fuse everything the camera produced since the last cycle.

        Reports arrivals and acceptances separately, because they are different
        health signals: no arrivals means the sensor or detector has stopped,
        while arrivals that are all rejected means the filter and the sensor
        disagree. Collapsing the two would report a disagreeing filter as a dead
        camera and send the operator hunting the wrong fault.

        Bounded work: the queue is shallow by construction, so this cannot run
        long regardless of how far behind the detector has fallen.
        """
        drained = 0
        accepted = 0
        for measurement in self._source.drain():
            drained += 1
            if self._ekf.fuse(measurement, body_rate, own_accel):
                accepted += 1
                self.fused += 1
            else:
                self.rejected += 1
        return PumpResult(drained=drained, fused=accepted)


# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Safety supervisor
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Fault taxonomy
# ---------------------------------------------------------------------------

class Fault(str, Enum):
    """Conditions that revoke guidance authority."""

    CAMERA_STALLED = "camera_stalled"
    TARGET_LOST = "target_lost"
    CONTROL_LOOP_STALLED = "control_loop_stalled"
    CONTROL_LOOP_OVERRUN = "control_loop_overrun"
    MAVLINK_HEARTBEAT_LOST = "mavlink_heartbeat_lost"
    FILTER_DIVERGED = "filter_diverged"
    FILTER_NOT_FINITE = "filter_not_finite"
    FILTER_STALE = "filter_stale"
    TRANSMIT_FAILURES = "transmit_failures"
    EXTERNAL_ABORT = "external_abort"


class SupervisorState(str, Enum):
    """Lifecycle of the supervisor itself."""

    IDLE = "idle"                    # constructed, not yet monitoring
    GRACE = "grace"                  # started, inside the startup window
    NOMINAL = "nominal"              # monitoring, no faults
    TRIPPED = "tripped"              # fault confirmed, handover performed
    HANDOVER_FAILED = "handover_failed"   # fault confirmed, handover refused

@dataclass
class FilterHealth:
    """Cheap summary of estimator health, computed by the control loop."""

    position_sigma_m: float
    velocity_sigma_mps: float
    epoch: float
    finite: bool
    positive_definite: bool


def filter_health_from_covariance(
    state: np.ndarray,
    covariance: np.ndarray,
    epoch: float,
    position_indices: List[int],
    velocity_indices: List[int],
) -> FilterHealth:
    """Summarise an EKF covariance into the handful of numbers the watchdog needs.

    Positive-definiteness is tested by Cholesky rather than by eigenvalues: it
    is the definitive test, it is several times cheaper on a 6x6, and it fails
    by exception exactly when the matrix has stopped being a covariance. A
    filter whose covariance has lost positive-definiteness is producing gains
    that are no longer a Kalman update, and every subsequent estimate is
    unsound -- so this is a trip condition, not a warning.
    """
    finite = bool(np.all(np.isfinite(state)) and np.all(np.isfinite(covariance)))

    if not finite:
        return FilterHealth(math.inf, math.inf, epoch, False, False)

    position_block = covariance[np.ix_(position_indices, position_indices)]
    velocity_block = covariance[np.ix_(velocity_indices, velocity_indices)]
    position_sigma = math.sqrt(max(float(np.trace(position_block)), 0.0))
    velocity_sigma = math.sqrt(max(float(np.trace(velocity_block)), 0.0))

    try:
        np.linalg.cholesky(covariance)
        positive_definite = True
    except np.linalg.LinAlgError:
        positive_definite = False

    return FilterHealth(
        position_sigma_m=position_sigma,
        velocity_sigma_mps=velocity_sigma,
        epoch=epoch,
        finite=True,
        positive_definite=positive_definite,
    )


# ---------------------------------------------------------------------------
# The supervisor
# ---------------------------------------------------------------------------

class SafetySupervisor:
    """Watchdog that revokes guidance authority and hands the vehicle back.

    Wiring:

        supervisor = SafetySupervisor(config, bridge)
        supervisor.start()
        ...
        # camera front-end, on every accepted frame
        supervisor.note_frame()
        # control loop, once per cycle
        supervisor.note_control_cycle(duration_s)
        supervisor.note_filter_health(health)
        if supervisor.is_control_authorised():
            bridge.send_velocity_setpoint(...)

    ``is_control_authorised`` is the gate. Once it returns False it keeps
    returning False (with ``latch``), and the control loop must stop
    transmitting -- that cessation is itself a safety action, because it lets
    the autopilot's own GUIDED timeout fire even if this process then dies.
    """

    def __init__(
        self,
        config: SupervisorConfig,
        bridge,
        logger: Optional[logging.Logger] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._cfg = config
        self._bridge = bridge
        self._log = logger or logging.getLogger("gnc.safety")
        self._clock = clock

        self._lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()

        # Liveness timestamps, all in the supervisor's own clock domain.
        self._start_time = 0.0
        self._last_frame_time = 0.0
        self._last_measurement_time = 0.0
        self._last_control_time = 0.0
        self._consecutive_overruns = 0
        self._consecutive_transmit_failures = 0
        self._filter_health: Optional[FilterHealth] = None

        self._state = SupervisorState.IDLE
        self._authorised = False
        self._fault_counts: Dict[Fault, int] = {}
        self._active_faults: List[Fault] = []
        self._trip_time: Optional[float] = None
        self._handover_mode: Optional[str] = None
        self._external_abort_reason: Optional[str] = None

        self._trip_callbacks: List[Callable[[List[Fault]], None]] = []
        self._ticks = 0

    # -- Lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """Begin monitoring. Authority is granted after the startup grace period."""
        with self._lock:
            if self._thread is not None:
                return
            now = self._clock()
            self._start_time = now
            # Seed the liveness stamps so the grace window is measured from
            # start rather than from the epoch.
            self._last_frame_time = now
            self._last_measurement_time = now
            self._last_control_time = now
            self._state = SupervisorState.GRACE
            self._authorised = True
            self._running.set()

        self._thread = threading.Thread(
            target=self._monitor_loop, name="gnc-supervisor", daemon=True
        )
        self._thread.start()
        self._log.info(
            "Safety supervisor armed: camera gap %.0f ms, control gap %.0f ms, "
            "heartbeat %.1f s, position sigma %.1f m, grace %.1f s",
            self._cfg.max_frame_gap_s * 1e3,
            self._cfg.max_control_gap_s * 1e3,
            self._cfg.max_heartbeat_age_s,
            self._cfg.max_position_sigma_m,
            self._cfg.startup_grace_s,
        )

    def stop(self) -> None:
        """Stop monitoring. Does not itself revoke authority or change mode."""
        self._running.clear()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                self._log.error("Supervisor thread did not exit within 2 s")
        self._thread = None
        self._log.info("Safety supervisor stopped after %d ticks", self._ticks)

    def reset(self) -> None:
        """Clear a latched trip. Deliberately manual -- see the module docstring."""
        with self._lock:
            now = self._clock()
            self._state = SupervisorState.GRACE
            self._authorised = True
            self._start_time = now
            self._last_frame_time = now
            self._last_measurement_time = now
            self._last_control_time = now
            self._consecutive_overruns = 0
            self._consecutive_transmit_failures = 0
            self._fault_counts.clear()
            self._active_faults = []
            self._trip_time = None
            self._handover_mode = None
            self._external_abort_reason = None
        self._log.warning("Safety supervisor reset; authority restored after grace")

    def register_trip_callback(self, callback: Callable[[List[Fault]], None]) -> None:
        """Register a callable invoked once, on the supervisor thread, at trip."""
        with self._lock:
            self._trip_callbacks.append(callback)

    # -- Liveness reporting (called from other threads; must stay cheap) -----

    def note_frame(self) -> None:
        """Report a frame arriving from the sensor.

        Called from the capture thread, so it must stay trivial: one lock
        acquisition and one float store, at the frame rate.
        """
        with self._lock:
            self._last_frame_time = self._clock()

    def note_measurement(self) -> None:
        """Report that a frame yielded a measurement the filter could use."""
        with self._lock:
            self._last_measurement_time = self._clock()

    def note_control_cycle(self, duration_s: float) -> None:
        """Report one completed control cycle and how long it took."""
        with self._lock:
            self._last_control_time = self._clock()
            if duration_s > self._cfg.max_control_cycle_s:
                self._consecutive_overruns += 1
            else:
                self._consecutive_overruns = 0

    def note_filter_health(self, health: FilterHealth) -> None:
        """Report the estimator's covariance summary for this cycle."""
        with self._lock:
            self._filter_health = health

    def note_transmit_result(self, ok: bool) -> None:
        """Report whether the last setpoint transmission succeeded."""
        with self._lock:
            if ok:
                self._consecutive_transmit_failures = 0
            else:
                self._consecutive_transmit_failures += 1

    def abort(self, reason: str) -> None:
        """Force a trip from outside -- an operator abort or a caller-detected fault."""
        with self._lock:
            self._external_abort_reason = reason
        self._log.error("External abort requested: %s", reason)

    # -- Authority gate -----------------------------------------------------

    def is_control_authorised(self) -> bool:
        """Whether the control loop may transmit setpoints this cycle."""
        with self._lock:
            return self._authorised

    @property
    def state(self) -> SupervisorState:
        with self._lock:
            return self._state

    @property
    def active_faults(self) -> List[Fault]:
        with self._lock:
            return list(self._active_faults)

    # -- Monitor thread -----------------------------------------------------

    def _apply_realtime_priority(self) -> None:
        priority = self._cfg.realtime_priority
        if priority is None:
            return
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
            self._log.info("Supervisor running SCHED_FIFO at priority %d", priority)
        except (PermissionError, OSError, AttributeError) as exc:
            # Expected without CAP_SYS_NICE. Worth a warning rather than silence:
            # a watchdog that can be starved by the workload it watches is a
            # weaker watchdog, and the operator should know which one they have.
            self._log.warning(
                "Supervisor could not obtain real-time priority (%s); it is "
                "subject to normal scheduling and may be delayed under load", exc
            )

    def _monitor_loop(self) -> None:
        self._apply_realtime_priority()
        period = 1.0 / max(self._cfg.check_rate_hz, 1e-6)
        next_tick = self._clock()

        while self._running.is_set():
            now = self._clock()
            self._ticks += 1

            try:
                self._tick(now)
            except Exception:
                # A crash in the watchdog must not silently leave the vehicle
                # under guidance. Revoke first, then report.
                self._log.critical(
                    "Supervisor tick raised; revoking authority", exc_info=True
                )
                with self._lock:
                    self._authorised = False
                    self._state = SupervisorState.TRIPPED

            next_tick += period
            sleep_for = next_tick - self._clock()
            if sleep_for > 0.0:
                time.sleep(sleep_for)
            else:
                next_tick = self._clock()

    def _tick(self, now: float) -> None:
        with self._lock:
            state = self._state

        if state in (SupervisorState.TRIPPED, SupervisorState.HANDOVER_FAILED):
            if self._cfg.latch:
                return
            # Unlatched operation: re-evaluate and recover if everything is
            # healthy again.
            if not self._evaluate(now):
                with self._lock:
                    self._state = SupervisorState.NOMINAL
                    self._authorised = True
                    self._active_faults = []
                self._log.warning("Faults cleared; authority restored (latch disabled)")
            return

        if state == SupervisorState.GRACE:
            if (now - self._start_time) < self._cfg.startup_grace_s:
                return
            with self._lock:
                self._state = SupervisorState.NOMINAL
            self._log.info("Startup grace elapsed; supervisor fully armed")

        confirmed = self._evaluate(now)
        if confirmed:
            self._trip(confirmed, now)

    # -- Fault evaluation ---------------------------------------------------

    def _evaluate(self, now: float) -> List[Fault]:
        """Return the faults that have persisted past the debounce window."""
        observed: List[Fault] = []

        with self._lock:
            frame_gap = now - self._last_frame_time
            measurement_gap = now - self._last_measurement_time
            control_gap = now - self._last_control_time
            overruns = self._consecutive_overruns
            transmit_failures = self._consecutive_transmit_failures
            health = self._filter_health
            abort_reason = self._external_abort_reason

        if abort_reason is not None:
            observed.append(Fault.EXTERNAL_ABORT)

        if frame_gap > self._cfg.max_frame_gap_s:
            observed.append(Fault.CAMERA_STALLED)

        if measurement_gap > self._cfg.max_measurement_gap_s:
            observed.append(Fault.TARGET_LOST)

        if control_gap > self._cfg.max_control_gap_s:
            observed.append(Fault.CONTROL_LOOP_STALLED)

        if overruns >= self._cfg.max_consecutive_overruns:
            observed.append(Fault.CONTROL_LOOP_OVERRUN)

        if transmit_failures >= self._cfg.max_consecutive_transmit_failures:
            observed.append(Fault.TRANSMIT_FAILURES)

        # The link is queried through the bridge's own state lock, which is held
        # only for microseconds and never across a serial write.
        try:
            heartbeat_age = self._bridge.heartbeat_age()
        except Exception:
            heartbeat_age = math.inf
        if heartbeat_age > self._cfg.max_heartbeat_age_s:
            observed.append(Fault.MAVLINK_HEARTBEAT_LOST)

        if health is not None:
            if not health.finite:
                observed.append(Fault.FILTER_NOT_FINITE)
            else:
                if not health.positive_definite:
                    observed.append(Fault.FILTER_DIVERGED)
                if health.position_sigma_m > self._cfg.max_position_sigma_m:
                    observed.append(Fault.FILTER_DIVERGED)
                if health.velocity_sigma_mps > self._cfg.max_velocity_sigma_mps:
                    observed.append(Fault.FILTER_DIVERGED)
                if (now - health.epoch) > self._cfg.max_filter_epoch_age_s:
                    observed.append(Fault.FILTER_STALE)

        # Debounce: a fault must be seen on consecutive ticks to count.
        confirmed: List[Fault] = []
        with self._lock:
            for fault in set(self._fault_counts) - set(observed):
                self._fault_counts.pop(fault, None)
            for fault in observed:
                count = self._fault_counts.get(fault, 0) + 1
                self._fault_counts[fault] = count
                if count >= self._cfg.trip_debounce_ticks:
                    confirmed.append(fault)
        return confirmed

    # -- Trip and handover --------------------------------------------------

    def _trip(self, faults: List[Fault], now: float) -> None:
        """Revoke authority, then hand the vehicle back to the autopilot."""
        with self._lock:
            self._authorised = False        # revoke first, always
            self._active_faults = list(faults)
            self._trip_time = now
            self._state = SupervisorState.TRIPPED

        names = ", ".join(fault.value for fault in faults)
        self._log.critical("SAFETY TRIP [%s] -- guidance authority revoked", names)

        link_lost = Fault.MAVLINK_HEARTBEAT_LOST in faults
        handed_over = self._execute_handover(link_lost)

        if not handed_over:
            with self._lock:
                self._state = SupervisorState.HANDOVER_FAILED

        for callback in list(self._trip_callbacks):
            try:
                callback(list(faults))
            except Exception:
                self._log.error("Trip callback raised", exc_info=True)

    def _viable_modes(self) -> List[str]:
        """Handover modes the vehicle can actually accept right now."""
        try:
            has_fix = self._bridge.has_position_fix()
        except Exception:
            has_fix = False

        viable = []
        for mode in self._cfg.handover_modes:
            if mode in self._cfg.position_dependent_modes and not has_fix:
                self._log.warning(
                    "Skipping %s: no 3-D position fix to support it", mode
                )
                continue
            viable.append(mode)
        return viable

    def _execute_handover(self, link_lost: bool) -> bool:
        """Stop commanding and put the vehicle into a pilot-recoverable mode.

        Order matters. The zero-velocity hold goes out first because it takes
        effect on the very next autopilot cycle, whereas a mode change has to be
        requested, accepted and confirmed. If the link itself is the fault there
        is nothing to send, and the correct action is to fall silent so that
        ArduPilot's GUIDED setpoint timeout fires on its own.
        """
        if link_lost:
            self._log.critical(
                "MAVLink heartbeat lost -- cannot command a mode change. Ceasing "
                "setpoint transmission so the autopilot's GUIDED timeout takes "
                "over. Verify GUID_TIMEOUT and the RC/GCS failsafe actions are "
                "configured on the airframe."
            )
            return False

        try:
            self._bridge.send_hold()
        except Exception:
            self._log.error("Zero-velocity hold failed to transmit", exc_info=True)

        for mode in self._viable_modes():
            for attempt in range(1, self._cfg.mode_attempts + 1):
                try:
                    requested = self._bridge.request_mode(mode)
                except Exception:
                    self._log.error("Mode request to %s raised", mode, exc_info=True)
                    requested = False

                if not requested:
                    continue

                try:
                    confirmed = self._bridge.confirm_mode(
                        mode, self._cfg.mode_confirm_timeout_s
                    )
                except Exception:
                    self._log.error("Mode confirmation raised", exc_info=True)
                    confirmed = False

                if confirmed:
                    with self._lock:
                        self._handover_mode = mode
                    self._log.critical(
                        "Flight authority handed back to ArduPilot in %s", mode
                    )
                    return True

                self._log.error(
                    "Mode %s not confirmed (attempt %d/%d)",
                    mode, attempt, self._cfg.mode_attempts,
                )

        self._log.critical(
            "HANDOVER FAILED -- no fallback mode was confirmed. Setpoints have "
            "stopped; the autopilot's GUIDED timeout and RC failsafe are now the "
            "only remaining protections. Pilot intervention required."
        )
        return False

    # -- Telemetry ----------------------------------------------------------

    def statistics(self) -> dict:
        with self._lock:
            now = self._clock()
            health = self._filter_health
            return {
                "state": self._state.value,
                "authorised": self._authorised,
                "ticks": self._ticks,
                "frame_gap_ms": (now - self._last_frame_time) * 1e3,
                "measurement_gap_ms": (now - self._last_measurement_time) * 1e3,
                "control_gap_ms": (now - self._last_control_time) * 1e3,
                "consecutive_overruns": self._consecutive_overruns,
                "position_sigma_m": (health.position_sigma_m if health else float("nan")),
                "velocity_sigma_mps": (health.velocity_sigma_mps if health else float("nan")),
                "active_faults": [fault.value for fault in self._active_faults],
                "handover_mode": self._handover_mode,
            }

    def format_telemetry(self) -> str:
        stats = self.statistics()
        return (
            f"supervisor {stats['state']:>8s} "
            f"(auth {'Y' if stats['authorised'] else 'N'}) | "
            f"frame gap {stats['frame_gap_ms']:5.1f} ms | "
            f"meas gap {stats['measurement_gap_ms']:6.1f} ms | "
            f"ctrl gap {stats['control_gap_ms']:5.1f} ms | "
            f"sigma_p {stats['position_sigma_m']:5.2f} m"
            + (f" | FAULTS: {','.join(stats['active_faults'])}"
               if stats["active_faults"] else "")
        )


# ---------------------------------------------------------------------------
# Integrated engine
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# The engine
# ---------------------------------------------------------------------------

class InspectionEngine:
    """Owns the control loop and the lifecycle of every subsystem."""

    def __init__(self, config: IntegrationConfig, logger: Optional[logging.Logger] = None) -> None:
        self._cfg = config
        self._log = logger or logging.getLogger("gnc.engine")

        self._bridge = MavlinkVelocityBridge(config.link, logging.getLogger("gnc.mavlink"))
        self._supervisor: SafetySupervisor  # set below; referenced by the hook
        self._source = CameraMeasurementSource(
            config.frontend,
            logger=logging.getLogger("gnc.camera"),
            # Camera liveness is reported from the capture thread, so the
            # 100 ms budget measures frames arriving from the sensor and
            # nothing else.
            frame_callback=lambda: self._supervisor.note_frame(),
        )
        self._ekf = LatencyCompensatedEKF(config.filt, logging.getLogger("gnc.ekf"))
        self._guidance = ProNavGuidance(config.guidance, logging.getLogger("gnc.pronav"))
        self._camera_bridge = CameraEkfBridge(self._source, self._ekf, self._log)
        self._supervisor = SafetySupervisor(
            config.supervisor, self._bridge, logging.getLogger("gnc.safety")
        )

        self._stop = threading.Event()
        self._cycles = 0
        self._overruns = 0
        self._transmitted = 0
        self._suppressed_not_guided = 0
        self._worst_cycle_s = 0.0
        self._last_telemetry = 0.0
        self._last_mode_warning = 0.0
        # Own acceleration handed to the estimator as a control input. The
        # ProNav command from the previous cycle is the best cheap estimate of
        # what the airframe is doing now; an IMU-derived acceleration (specific
        # force with gravity removed) would be better still and can be
        # substituted here without touching the filter.
        self._last_commanded_accel = np.zeros(3, dtype=np.float64)

    # -- Lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """Bring up the link, the camera and the watchdog, in that order."""
        self._bridge.connect(wait_for_heartbeat=self._cfg.wait_for_heartbeat)
        self._bridge.request_attitude_stream(rate_hz=50.0)

        status = self._bridge.link_status()
        self._log.info(
            "Autopilot: mode %s, armed %s, GPS fix %d (%d sats)",
            status["mode"], status["armed"], status["gps_fix_type"], status["satellites"],
        )

        self._source.start()

        # The supervisor comes up last so its startup grace covers a pipeline
        # that is already producing frames, and it registers a callback that
        # stops the camera-side work as soon as authority is revoked.
        self._supervisor.register_trip_callback(self._on_trip)
        self._supervisor.start()

        self._log.info(
            "Inspection engine running at %.0f Hz (require_guided=%s)",
            self._cfg.control_rate_hz, self._cfg.require_guided_mode,
        )

    def _on_trip(self, faults: List[Fault]) -> None:
        """Invoked on the supervisor thread once a trip is confirmed."""
        # The guidance integrator must not carry a stale command into any later
        # re-authorisation; the supervisor has already stopped transmission.
        self._guidance.reset()
        self._log.critical(
            "Engine notified of safety trip: %s", ", ".join(f.value for f in faults)
        )

    def stop(self) -> None:
        """Shut down in the reverse order, leaving the vehicle commanded to hold."""
        self._stop.set()
        self._supervisor.stop()
        self._source.stop()

        # A parting zero-velocity hold, but only if we still hold authority and
        # the vehicle is still listening to us. Commanding a vehicle a pilot has
        # already taken back would be exactly the wrong last act.
        try:
            if self._is_vehicle_listening():
                self._bridge.send_hold()
                self._log.info("Zero-velocity hold sent on shutdown")
        except Exception:
            self._log.error("Shutdown hold failed", exc_info=True)

        self._bridge.close()
        self._log.info(
            "Engine stopped: %d cycles, %d overruns, %d setpoints, worst cycle %.2f ms",
            self._cycles, self._overruns, self._transmitted, self._worst_cycle_s * 1e3,
        )

    # -- Authority ----------------------------------------------------------

    def _is_vehicle_listening(self) -> bool:
        """True when the autopilot is in a mode that accepts our setpoints."""
        if not self._cfg.require_guided_mode:
            return True
        return self._bridge.current_mode_name() == "GUIDED"

    # -- Control loop -------------------------------------------------------

    def run(self) -> int:
        """Run the control loop until stopped. Returns a process exit code."""
        period = 1.0 / max(self._cfg.control_rate_hz, 1e-6)
        next_tick = time.monotonic()
        body_rate = np.zeros(3, dtype=np.float64)

        while not self._stop.is_set():
            cycle_start = time.monotonic()

            try:
                self._control_cycle(cycle_start, body_rate)
            except Exception:
                # An exception here would otherwise leave the vehicle under a
                # stale setpoint. Tell the watchdog and let it hand over.
                self._log.critical("Control cycle raised", exc_info=True)
                self._supervisor.abort("control_cycle_exception")

            duration = time.monotonic() - cycle_start
            self._cycles += 1
            self._worst_cycle_s = max(self._worst_cycle_s, duration)
            if duration > period:
                self._overruns += 1
            self._supervisor.note_control_cycle(duration)

            self._maybe_report(cycle_start)

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0.0:
                self._stop.wait(sleep_for)
            else:
                # Resynchronise rather than trying to catch up: a burst of
                # back-to-back cycles after a stall would integrate guidance
                # with wrong timesteps.
                next_tick = time.monotonic()

        return 0 if self._supervisor.state != SupervisorState.HANDOVER_FAILED else 1

    def _control_cycle(self, now: float, body_rate_out: np.ndarray) -> None:
        body_rate = self._bridge.body_rate()
        body_rate_out[:] = body_rate

        # (1) Fuse whatever the camera produced since the last cycle. Bounded
        #     work: the measurement queue is shallow by construction.
        #
        #     Target liveness is reported on measurement *arrival*, not on
        #     acceptance. A measurement the filter gates out still proves the
        #     detector is finding something; if every one is rejected the
        #     covariance grows and FILTER_DIVERGED fires, which is the accurate
        #     description of that failure. Camera liveness is a separate signal
        #     fed from the capture thread (see the frame_callback above).
        own_accel = self._last_commanded_accel
        result = self._camera_bridge.pump(body_rate, own_accel)
        if result.drained > 0:
            self._supervisor.note_measurement()

        # (2) Bring the estimate to the current epoch.
        self._ekf.predict_to(now, body_rate, own_accel)

        # (3) Report estimator health to the watchdog. Only meaningful once a
        #     track exists; before that the covariance is the initial seed.
        if self._ekf.initialised:
            state, covariance, epoch = self._ekf.snapshot()
            self._supervisor.note_filter_health(
                filter_health_from_covariance(
                    state, covariance, epoch, IDX_POS, IDX_VEL
                )
            )

        # (4) Guidance.
        solution = None
        if self._ekf.is_track_fresh(now):
            solution = self._guidance.compute(
                self._ekf.relative_position(),
                self._ekf.relative_velocity(),
                1.0 / self._cfg.control_rate_hz,
            )

        if solution is None:
            velocity = self._guidance.relax(1.0 / self._cfg.control_rate_hz)
            acceleration = np.zeros(3, dtype=np.float64)
            yaw_rate = 0.0
        else:
            velocity = solution.velocity_body
            acceleration = solution.accel_body
            yaw_rate = solution.yaw_rate
        self._last_commanded_accel = np.asarray(acceleration, dtype=np.float64).copy()

        # (5) The authority gate. Both conditions, every cycle.
        if not self._supervisor.is_control_authorised():
            return

        if not self._is_vehicle_listening():
            self._suppressed_not_guided += 1
            if (now - self._last_mode_warning) > 1.0:
                self._last_mode_warning = now
                self._log.warning(
                    "Autopilot is in %s, not GUIDED -- withholding setpoints",
                    self._bridge.current_mode_name(),
                )
            # Guidance must not integrate while its output is going nowhere, or
            # it would hand the vehicle a large step command the moment GUIDED
            # is re-selected.
            self._guidance.reset()
            return

        ok = self._bridge.send_velocity_setpoint(velocity, yaw_rate, acceleration)
        self._supervisor.note_transmit_result(ok)
        if ok:
            self._transmitted += 1

    # -- Telemetry ----------------------------------------------------------

    def _maybe_report(self, now: float) -> None:
        if (now - self._last_telemetry) < self._cfg.telemetry_period_s:
            return
        self._last_telemetry = now

        if self._ekf.initialised:
            position = self._ekf.relative_position()
            velocity = self._ekf.relative_velocity()
            track = (
                f"range {float(np.linalg.norm(position)):6.2f} m, "
                f"closing {-float(np.dot(velocity, position / max(float(np.linalg.norm(position)), 1e-9))):6.2f} m/s"
            )
        else:
            track = "no track"

        link = self._bridge.link_status()
        self._log.info(
            "%s | %s | mode %s hb %.1fs | sent %d (withheld %d) | %s | "
            "cycles %d, overruns %d, worst %.2f ms",
            track,
            self._source.format_telemetry(),
            link["mode"],
            link["heartbeat_age_s"],
            self._transmitted,
            self._suppressed_not_guided,
            self._supervisor.format_telemetry(),
            self._cycles,
            self._overruns,
            self._worst_cycle_s * 1e3,
        )

    def request_stop(self) -> None:
        self._stop.set()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def build_config(args: argparse.Namespace) -> IntegrationConfig:
    config = IntegrationConfig()

    config.link.device = args.device
    config.link.baud = args.baud
    config.link.send_acceleration = args.send_acceleration

    config.frontend.camera.backend = args.backend
    config.frontend.camera.sensor_id = args.sensor_id
    config.frontend.camera.device = args.camera_device
    config.frontend.camera.width = args.width
    config.frontend.camera.height = args.height
    config.frontend.camera.frame_rate_hz = args.fps
    config.frontend.camera.sensor_latency_frames = args.latency_frames
    config.frontend.target.width_m = args.target_width
    config.frontend.target.width_sigma_m = args.target_width_sigma
    config.frontend.detector.detect_scale = args.detect_scale

    if args.intrinsics:
        config.frontend.intrinsics = CameraIntrinsics.from_file(args.intrinsics)._cfg
    else:
        config.frontend.intrinsics.cx = args.width / 2.0
        config.frontend.intrinsics.cy = args.height / 2.0

    config.guidance.nav_constant = args.nav_constant
    config.guidance.standoff_range_m = args.standoff

    config.control_rate_hz = args.control_rate
    config.require_guided_mode = not args.allow_any_mode
    config.wait_for_heartbeat = not args.no_wait_heartbeat

    config.supervisor.max_frame_gap_s = args.max_frame_gap_ms / 1e3
    config.supervisor.max_position_sigma_m = args.max_position_sigma
    config.supervisor.handover_modes = tuple(
        mode.strip().upper() for mode in args.handover_modes.split(",") if mode.strip()
    )
    return config


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Integrated relative-navigation inspection engine",
    )
    # Link
    parser.add_argument("--device", default="/dev/ttyTHS1",
                        help="MAVLink endpoint: serial path, or udpout:host:port for SITL")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--send-acceleration", action="store_true",
                        help="Include the ProNav acceleration as a feed-forward term "
                             "(verify your Copter firmware honours it first)")
    parser.add_argument("--no-wait-heartbeat", action="store_true")
    parser.add_argument("--allow-any-mode", action="store_true",
                        help="Transmit even when the autopilot is not in GUIDED. "
                             "This removes takeover-by-mode-switch; bench use only.")
    # Camera
    parser.add_argument("--backend", default="argus", choices=["argus", "v4l2", "synthetic"])
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--camera-device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--latency-frames", type=float, default=2.0)
    parser.add_argument("--detect-scale", type=float, default=0.5)
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--target-width", type=float, default=0.50)
    parser.add_argument("--target-width-sigma", type=float, default=0.10)
    # Guidance
    parser.add_argument("--nav-constant", type=float, default=4.0)
    parser.add_argument("--standoff", type=float, default=3.0)
    parser.add_argument("--control-rate", type=float, default=60.0)
    # Safety
    parser.add_argument("--max-frame-gap-ms", type=float, default=100.0)
    parser.add_argument("--max-position-sigma", type=float, default=12.0)
    parser.add_argument("--handover-modes", default="LOITER,BRAKE,RTL,ALT_HOLD",
                        help="Comma-separated fallback ladder, tried in order")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d %(name)-12s %(levelname)-8s %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )
    log = logging.getLogger("gnc.main")

    config = build_config(args)
    engine = InspectionEngine(config, log)

    def handle_signal(signum, _frame):
        log.info("Signal %d received; stopping", signum)
        engine.request_stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        engine.start()
    except Exception:
        log.exception("Startup failed")
        return 1

    try:
        return engine.run()
    finally:
        engine.stop()


if __name__ == "__main__":
    sys.exit(main())

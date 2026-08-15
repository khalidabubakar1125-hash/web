#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Asynchronous Arducam Global Shutter MIPI front-end for the relative-nav EKF.

Target platform
---------------
    Compute  : NVIDIA Jetson Orin Nano (Ubuntu 22.04 LTS, JetPack 6.x)
    Sensor   : Arducam Global Shutter MIPI CSI-2 module, 1080p @ 60 Hz
    Consumer : gnc/relative_nav_pronav.py -- LatencyCompensatedEKF

Threading model
---------------
The estimator must step on a metronome. If the 60 Hz control loop ever called
``VideoCapture.read()`` it would inherit every hiccup of the camera pipeline --
CSI-2 DMA stalls, ISP scheduling, USB/serdes retries -- and the EKF cycle time
would be governed by the worst frame, not the nominal one. So no stage in this
front-end is allowed to block the consumer:

    [capture thread]  cap.read() -> stamp -> LatestFrameSlot (overwrite)
             |                                    |
             |  (drops silently under back-pressure)
             v                                    v
    [detector thread(s)] slot.take() -> detect -> MeasurementQueue (drop-oldest)
                                                  |
                                                  v
    [EKF / control thread]  drain() -> ekf.fuse()   <-- never blocks, never waits

Both hand-offs are lossy by design. A frame that cannot be consumed in time is
worthless to a state estimator -- it is strictly better to drop it and keep the
newest one than to queue it and grow an unbounded latency tail. What is *not*
lost is time: every frame is stamped at grab, and that stamp rides all the way
into the ``Measurement``, so a detector that occasionally takes 40 ms produces a
correctly back-dated observation rather than a mis-timed one. The EKF's
retrodiction buffer then puts it exactly where it belongs on the timeline.

Measurement construction
------------------------
The EKF consumes polar observations ``[azimuth, elevation, range]`` in the BODY
FRD frame, so this module goes straight there rather than round-tripping through
Cartesian:

    bearing : pixel -> undistorted normalised ray -> optical frame -> BODY FRD
    range   : monocular size-scale, r = f * W_true / w_pixels

Both channels carry an analytically derived covariance instead of a hand-tuned
constant -- see :meth:`MonocularRangeModel.measurement_covariance`.
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
from typing import Callable, Deque, List, Optional, Tuple

import cv2
import numpy as np

# The estimator module is a sibling file. Support both "run as a script from the
# gnc/ directory" and "imported as part of a package" without duplicating code.
try:  # pragma: no cover - import shim
    from .relative_nav_pronav import (
        EPS,
        R_CAM_TO_BODY,
        LatencyCompensatedEKF,
        Measurement,
        cartesian_to_polar,
    )
except ImportError:  # pragma: no cover - import shim
    from relative_nav_pronav import (  # type: ignore[no-redef]
        EPS,
        R_CAM_TO_BODY,
        LatencyCompensatedEKF,
        Measurement,
        cartesian_to_polar,
    )


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

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

    def pump(self, body_rate: np.ndarray) -> "PumpResult":
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
            if self._ekf.fuse(measurement, body_rate):
                accepted += 1
                self.fused += 1
            else:
                self.rejected += 1
        return PumpResult(drained=drained, fused=accepted)


# ---------------------------------------------------------------------------
# ROS 2 publisher node (optional deployment shape)
# ---------------------------------------------------------------------------

def build_ros_node(config: FrontendConfig):
    """Construct the ROS 2 publisher node.

    Imported lazily so the front-end stays usable -- and testable -- on a
    machine without a ROS install. Publishes ``geometry_msgs/PointStamped`` in
    the BODY FRD frame on ``/perception/target_point``, which is exactly what
    ``RelativeNavigationNode`` subscribes to.

    IMPORTANT: the published header stamp is the *exposure epoch* -- sensor
    latency has already been removed here. Set ``pipeline_latency_frames = 0``
    on the consumer, or it will subtract the same 33.3 ms a second time and
    push the sample two frames into the past.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
    )
    from geometry_msgs.msg import PointStamped

    class CameraFrontendNode(Node):
        def __init__(self) -> None:
            super().__init__("camera_frontend")
            self._cfg = config

            # ROS time is the shared clock domain with the estimator node.
            self._source = CameraMeasurementSource(
                config,
                logger=logging.getLogger("gnc.camera"),
                clock=lambda: self.get_clock().now().nanoseconds * 1e-9,
            )

            sensor_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1,
            )
            self._publisher = self.create_publisher(
                PointStamped, "/perception/target_point", sensor_qos
            )

            self._source.start()
            # Poll faster than the frame rate so a measurement never waits a
            # full period for a publish slot.
            self._timer = self.create_timer(
                0.5 * config.camera.frame_period_s, self._on_tick
            )
            self._telemetry_timer = self.create_timer(2.0, self._on_telemetry)
            self.get_logger().info("Camera front-end publishing to /perception/target_point")

        def _on_tick(self) -> None:
            for measurement in self._source.drain():
                az, el, rng = measurement.polar
                # Republish as a Cartesian point in BODY FRD. The consumer's own
                # polar conversion is lossless, so nothing is given up here.
                position = np.array(
                    [rng * math.cos(el) * math.cos(az),
                     rng * math.cos(el) * math.sin(az),
                     -rng * math.sin(el)],
                    dtype=np.float64,
                )
                msg = PointStamped()
                msg.header.stamp.sec = int(measurement.capture_time)
                msg.header.stamp.nanosec = int(
                    (measurement.capture_time - int(measurement.capture_time)) * 1e9
                )
                msg.header.frame_id = "body_frd"
                msg.point.x = float(position[0])
                msg.point.y = float(position[1])
                msg.point.z = float(position[2])
                self._publisher.publish(msg)

        def _on_telemetry(self) -> None:
            if not self._source.is_healthy():
                self.get_logger().warning("Camera stream stalled: no recent frames")
            self.get_logger().info(self._source.format_telemetry())

        def shutdown(self) -> None:
            self._timer.cancel()
            self._telemetry_timer.cancel()
            self._source.stop()

    return rclpy, CameraFrontendNode


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def build_config(args: argparse.Namespace) -> FrontendConfig:
    config = FrontendConfig()
    config.camera.backend = args.backend
    config.camera.sensor_id = args.sensor_id
    config.camera.device = args.device
    config.camera.width = args.width
    config.camera.height = args.height
    config.camera.frame_rate_hz = args.fps
    config.camera.flip_method = args.flip_method
    config.camera.sensor_latency_frames = args.latency_frames
    config.detector.detect_scale = args.detect_scale
    config.detector_threads = args.detector_threads
    config.target.width_m = args.target_width
    config.target.width_sigma_m = args.target_width_sigma

    if args.intrinsics:
        loaded = CameraIntrinsics.from_file(args.intrinsics)
        config.intrinsics = loaded._cfg
    else:
        # Default principal point to the image centre for the chosen resolution.
        config.intrinsics.cx = args.width / 2.0
        config.intrinsics.cy = args.height / 2.0
    return config


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Asynchronous Arducam GS MIPI front-end for the relative-nav EKF",
    )
    parser.add_argument("--backend", default="argus", choices=["argus", "v4l2", "synthetic"])
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--device", default="/dev/video0", help="V4L2 node (v4l2 backend)")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--flip-method", type=int, default=0)
    parser.add_argument("--latency-frames", type=float, default=2.0,
                        help="Fixed sensor-to-userspace latency, in frames")
    parser.add_argument("--detect-scale", type=float, default=0.5)
    parser.add_argument("--detector-threads", type=int, default=1)
    parser.add_argument("--target-width", type=float, default=0.50,
                        help="Assumed target width [m] for the size-scale range")
    parser.add_argument("--target-width-sigma", type=float, default=0.10,
                        help="1-sigma uncertainty on the assumed target width [m]")
    parser.add_argument("--intrinsics", default=None,
                        help="camera_info YAML or JSON intrinsics file")
    parser.add_argument("--mode", default="standalone", choices=["standalone", "ros"],
                        help="standalone: drive an EKF in-process; ros: publish PointStamped")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Standalone run length in seconds (0 = until interrupted)")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args(argv)


def run_standalone(config: FrontendConfig, duration: float, log: logging.Logger) -> int:
    """Drive a real EKF from the camera, in-process, at 60 Hz.

    This is the deployment shape the ProNav engine wants: one process, no
    middleware between the detector and the filter.
    """
    from relative_nav_pronav import FilterConfig  # local import keeps the shim simple

    source = CameraMeasurementSource(config, logger=log)
    ekf = LatencyCompensatedEKF(FilterConfig(), logging.getLogger("gnc.ekf"))
    bridge = CameraEkfBridge(source, ekf, log)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    source.start()
    period = 1.0 / 60.0
    body_rate = np.zeros(3, dtype=np.float64)
    started = time.monotonic()
    next_tick = started
    cycle_overruns = 0
    cycles = 0
    worst_cycle_ms = 0.0

    try:
        while not stop.is_set():
            now = time.monotonic()
            if duration > 0.0 and (now - started) >= duration:
                break

            tick_start = now
            # The two lines the whole module exists to make safe: neither can
            # block on the camera.
            bridge.pump(body_rate)
            ekf.predict_to(now, body_rate)

            cycle_ms = (time.monotonic() - tick_start) * 1e3
            worst_cycle_ms = max(worst_cycle_ms, cycle_ms)
            cycles += 1
            if cycle_ms > period * 1e3:
                cycle_overruns += 1

            if cycles % 120 == 0:
                position = ekf.relative_position() if ekf.initialised else None
                where = (
                    f"range {float(np.linalg.norm(position)):6.2f} m"
                    if position is not None else "no track"
                )
                log.info("%s | %s | cycle worst %.2f ms, overruns %d/%d",
                         where, source.format_telemetry(), worst_cycle_ms,
                         cycle_overruns, cycles)

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0.0:
                stop.wait(sleep_for)
            else:
                next_tick = time.monotonic()   # resynchronise after a long stall
    finally:
        source.stop()

    log.info("Ran %d cycles, %d overruns, worst cycle %.2f ms",
             cycles, cycle_overruns, worst_cycle_ms)
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d %(name)-12s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )
    log = logging.getLogger("gnc.frontend")
    config = build_config(args)

    if args.mode == "standalone":
        return run_standalone(config, args.duration, log)

    rclpy, node_class = build_ros_node(config)
    rclpy.init(args=None)
    node = node_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - interactive path
        log.info("Interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

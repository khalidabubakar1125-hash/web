#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Single-process integration of the autonomous inspection GNC stack.

    gnc/camera_frontend.py      Arducam GS MIPI -> polar measurements  (async)
    gnc/relative_nav_pronav.py  latency-compensated EKF + True ProNav + MAVLink
    gnc/safety_supervisor.py    independent watchdog and authority handover

This module owns the 60 Hz control loop that ties them together and nothing
else. Everything it calls is non-blocking by construction: the camera hands over
through a drop-oldest queue, the estimator and guidance law are pure
computation, and the MAVLink write goes into a kernel buffer.

Control cycle
-------------
    1. drain camera measurements  -> EKF.fuse()      (retrodiction handles lag)
    2. EKF.predict_to(now)                            (bring estimate to epoch)
    3. summarise covariance       -> supervisor
    4. ProNav                     -> velocity + acceleration command
    5. authority gate             -> transmit, or stay silent
    6. report cycle timing        -> supervisor

Authority
---------
Two independent conditions must both hold before a setpoint is transmitted:

    * the supervisor has not tripped, and
    * the autopilot is in GUIDED.

The second is what keeps this process from fighting a pilot. The moment someone
flips the mode switch, ArduPilot leaves GUIDED and this loop stops transmitting
on the next cycle -- no handshake, no negotiation, no race.

Bench testing
-------------
Point ``--device`` at a SITL endpoint (``udpout:127.0.0.1:14550`` or
``tcp:127.0.0.1:5760``) to exercise the full stack without a flight controller
on the bench; ``--backend synthetic`` does the same for the camera. Both are
real endpoints, not simulations of this code's own behaviour.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from camera_frontend import (
    CameraEkfBridge,
    CameraMeasurementSource,
    CameraIntrinsics,
    FrontendConfig,
)
from relative_nav_pronav import (
    IDX_POS,
    IDX_VEL,
    FilterConfig,
    GuidanceConfig,
    LatencyCompensatedEKF,
    LinkConfig,
    MavlinkVelocityBridge,
    ProNavGuidance,
)
from safety_supervisor import (
    Fault,
    SafetySupervisor,
    SupervisorConfig,
    SupervisorState,
    filter_health_from_covariance,
)


# ---------------------------------------------------------------------------
# Configuration
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
        result = self._camera_bridge.pump(body_rate)
        if result.drained > 0:
            self._supervisor.note_measurement()

        # (2) Bring the estimate to the current epoch.
        self._ekf.predict_to(now, body_rate)

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

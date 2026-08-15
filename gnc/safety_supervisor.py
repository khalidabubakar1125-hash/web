#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Independent safety supervisor for the relative-navigation engine.

Purpose
-------
The guidance stack commands vehicle velocity. Everything that feeds it -- a
camera, a detector, an EKF, a serial link -- can fail, and several of those
failures are silent: a detector that stops finding the target does not raise,
an EKF whose covariance is diverging still returns numbers, and a serial link
that has died still accepts writes into a kernel buffer. This module watches for
those conditions and, when one is confirmed, stops commanding and hands flight
authority back to ArduPilot.

Design rules
------------
1. **Independent.** The supervisor runs on its own thread with its own clock. It
   never calls into the estimator, the camera or the guidance law, so a deadlock
   or an infinite loop in any of them cannot take the watchdog with it.

2. **Push-fed, not polling.** Components report liveness by calling ``note_*``;
   the supervisor only reads its own timestamps. A watchdog that has to ask a
   component whether it is alive can block on that component's lock. A watchdog
   that waits to be fed trips correctly when the feeder dies -- which is the
   entire point.

3. **Verify, don't assume.** A mode change is confirmed by reading the flight
   mode back out of HEARTBEAT. "Command sent" is not "authority handed over":
   LOITER is refused without a position fix, RTL without a home position.

4. **Latch by default.** Once tripped, the supervisor stays tripped until a
   human calls :meth:`reset`. Automatic recovery on a transient would hand
   control back and forth mid-flight, which is worse than staying in Loiter.

What this cannot do
-------------------
This is a companion-computer process on a general-purpose Linux kernel. It
cannot preempt a kernel stall, an OOM kill, or a power loss, and its own thread
is subject to the scheduler. It is a layer *above* the autopilot's own
failsafes, never a replacement for them. The autopilot-side protections --
GUIDED-mode setpoint timeout, RC failsafe, battery failsafe, geofence -- remain
the authority of last resort, and the most important single behaviour in this
module is that when it can no longer trust itself it *stops transmitting*, which
lets those protections fire.
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional

import numpy as np


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

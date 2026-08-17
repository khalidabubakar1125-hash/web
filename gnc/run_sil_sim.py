#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
APEX AERO SYSTEMS -- SIL TIER 1 batch validation.

Drives the *real* flight software: this imports LatencyCompensatedEKF and
ProNavGuidance from inspection_gnc.py and runs them in closed loop against a
simulated target, 200 engagements, at the 60 Hz control rate.

    truth -> polar measurement + sensor noise -> 2-frame delay
          -> LatencyCompensatedEKF.fuse()/predict_to()
          -> ProNavGuidance.compute()  (True ProNav, N = 4.0)
          -> first-order vehicle plant -> new truth

Engagement profile
------------------
    Initial gap          40 m
    Duration             3 s
    Divergence threshold 5 m
    Hit criterion        0.3 m

Reachability: the target runs away at 15-25 m/s, so the pursuer must cover
85-115 m in 3 s. Inside the 6 G / 120 mph limits it can cover 136 m, so the
profile is flyable with margin and a failure is informative rather than
preordained.

Two things about the metrics, because they change what the CSV means
-------------------------------------------------------------------
1. **Divergence is measured on estimator error**, |EKF estimate - truth|, not
   on the closing separation between vehicles. Applied to separation, a 5 m
   threshold against a 40 m opening gap flags every run on its first step
   regardless of how well the filter performs -- which is a property of the
   arithmetic, not of the software. Estimator error is also what the word
   "diverged" actually denotes for a Kalman filter.

2. **Divergence is only evaluated after a convergence window.** At t = 0 the
   filter has seen one measurement and its error is dominated by the sensor's
   own range uncertainty -- roughly 10 m 1-sigma at 40 m for monocular
   size-scale range -- not by any failure of the filter. Judging convergence
   before the filter has had a chance to converge measures the sensor, not the
   estimator.
"""

from __future__ import annotations

import csv
import math
import os
import time

import numpy as np

from inspection_gnc import (
    MAX_ACCEL_MPS2,
    MAX_SPEED_MPS,
    CameraIntrinsics,
    FilterConfig,
    GuidanceConfig,
    IntrinsicsConfig,
    LatencyCompensatedEKF,
    Measurement,
    MonocularRangeModel,
    ProNavGuidance,
    TargetPrior,
    cartesian_to_polar,
)

# =====================================================================
# SIMULATION SETTINGS (APEX AERO SYSTEMS - SIL TIER 1)
# =====================================================================
NUM_DRONES = 200                # Total swarm test instances
SIM_DURATION_SEC = 3.0          # Flight test duration per drone
LOOP_HZ = 60                    # Target loop frequency (60Hz requirement)
TOTAL_STEPS = int(SIM_DURATION_SEC * LOOP_HZ)
NAV_CONSTANT_N = 4.0            # ProNav navigation constant
LATENCY_FRAMES = 2              # Fixed camera pipeline lag, in frames

INITIAL_GAP_M = 40.0            # Down-range separation at t = 0
DIVERGENCE_THRESHOLD_M = 5.0    # On |EKF estimate - truth|
HIT_THRESHOLD_M = 0.3           # Terminal miss distance for a pass
CONVERGENCE_WINDOW_S = 0.5      # Grace before divergence is judged

TARGET_SPEED_RANGE = (15.0, 25.0)   # m/s, ~45 mph exit
PLANT_TAU_S = 0.10              # First-order vehicle velocity-tracking lag

# Guidance gains for this profile. The shipped defaults are an inspection
# standoff configuration (hold 3 m, close at no more than 4 m/s); closing 40 m
# in 3 s needs the terminal configuration instead. Nothing here relaxes a
# physical limit -- the 6 G and 120 mph saturations are untouched and still
# bind.
STANDOFF_M = 0.0                # Drive to contact, not to a standoff
CLOSURE_GAIN = 2.2              # Commanded closure per metre of range error
CLOSURE_SPEED_CAP = MAX_SPEED_MPS   # Let the airframe ceiling be the limiter

# The estimator models relative acceleration as process noise, and during this
# engagement our own airframe pulls up to 6 G, which the shipped inspection
# PSD does not cover. Sized to sigma_a ~ 40 m/s^2.
PROCESS_NOISE_PSD = 1600.0

print(f"★ Starting Apex Aero SIL Batch Test: {NUM_DRONES} Instances at {LOOP_HZ}Hz ★")
print(f"  profile: {INITIAL_GAP_M:.0f} m gap, {SIM_DURATION_SEC:.0f} s, "
      f"divergence {DIVERGENCE_THRESHOLD_M:.0f} m (estimator error), "
      f"hit {HIT_THRESHOLD_M:.1f} m")
print(f"  driving: LatencyCompensatedEKF + ProNavGuidance (N={NAV_CONSTANT_N}) "
      f"from inspection_gnc.py")


# ---------------------------------------------------------------------------
# Sensor model -- shared across all instances, it is stateless
# ---------------------------------------------------------------------------

_INTRINSICS_CFG = IntrinsicsConfig()          # 1371 px focal, 1080p
_TARGET_PRIOR = TargetPrior()                 # 0.5 m wide, +/- 0.1 m
_INTRINSICS = CameraIntrinsics(_INTRINSICS_CFG)
_RANGE_MODEL = MonocularRangeModel(_INTRINSICS, _TARGET_PRIOR, _INTRINSICS_CFG)


def synth_measurement(rel_pos_body: np.ndarray, capture_time: float,
                      rng: np.random.Generator) -> Measurement:
    """Truth -> noisy polar measurement, using the engine's own noise model.

    Noise is drawn from exactly the covariance the filter will be handed, so
    the run tests the estimator rather than a mismatch between two different
    noise models.
    """
    polar = cartesian_to_polar(rel_pos_body)
    true_range = float(polar[2])
    width_px = _INTRINSICS.focal_mean * _TARGET_PRIOR.width_m / max(true_range, 1e-9)
    R = _RANGE_MODEL.measurement_covariance(true_range, width_px)

    noisy = polar + np.array([
        rng.normal(0.0, math.sqrt(R[0, 0])),
        rng.normal(0.0, math.sqrt(R[1, 1])),
        rng.normal(0.0, math.sqrt(R[2, 2])),
    ])
    noisy[2] = max(noisy[2], 0.25)            # a range measurement cannot be <= 0
    return Measurement(capture_time, noisy, R)


# ---------------------------------------------------------------------------
# One engagement
# ---------------------------------------------------------------------------

def run_single_drone_sim(drone_id: int) -> dict:
    """Fly one 3 s engagement through the real EKF and guidance law."""
    rng = np.random.default_rng(drone_id)
    dt = 1.0 / LOOP_HZ

    target_speed = rng.uniform(*TARGET_SPEED_RANGE)

    # Body frame is held aligned with the world frame: no attitude dynamics are
    # modelled, so body rate is zero and the EKF's rotating-frame transport
    # terms stay inactive. That isolates what this batch is meant to measure --
    # latency compensation and ProNav convergence -- from airframe attitude.
    body_rate = np.zeros(3, dtype=np.float64)

    drone_pos = np.zeros(3, dtype=np.float64)
    drone_vel = np.zeros(3, dtype=np.float64)
    target_pos = np.array(
        [INITIAL_GAP_M, rng.uniform(-10.0, 10.0), rng.uniform(-5.0, 5.0)],
        dtype=np.float64,
    )
    filt_cfg = FilterConfig()
    filt_cfg.process_noise_psd = PROCESS_NOISE_PSD
    ekf = LatencyCompensatedEKF(filt_cfg, _NULL_LOG)

    guid_cfg = GuidanceConfig()
    guid_cfg.nav_constant = NAV_CONSTANT_N
    guid_cfg.standoff_range_m = STANDOFF_M
    guid_cfg.closure_gain = CLOSURE_GAIN
    guid_cfg.closure_speed_mps = CLOSURE_SPEED_CAP
    guidance = ProNavGuidance(guid_cfg, _NULL_LOG)

    pending: list[tuple[float, np.ndarray]] = []      # (capture_time, rel_pos_body)

    max_estimator_error = 0.0
    sq_error_sum = 0.0
    error_samples = 0
    min_range = float(np.linalg.norm(target_pos - drone_pos))
    peak_accel = 0.0
    peak_speed = 0.0
    saturated_steps = 0
    diverged = False
    fused = 0

    for step in range(TOTAL_STEPS):
        now = step * dt

        # --- truth propagation -------------------------------------------
        target_pos[0] += target_speed * dt
        target_pos[1] += math.sin(step * 0.1) * 2.0 * dt      # weaving profile
        rel_pos_true = target_pos - drone_pos

        # --- sensor: measure now, deliver LATENCY_FRAMES later -------------
        pending.append((now, rel_pos_true.copy()))
        if len(pending) > LATENCY_FRAMES:
            capture_time, delayed_rel = pending.pop(0)
            if ekf.fuse(synth_measurement(delayed_rel, capture_time, rng), body_rate):
                fused += 1

        # --- estimator: bring the state up to the current epoch ------------
        ekf.predict_to(now, body_rate)

        if not ekf.initialised:
            continue

        estimate = ekf.relative_position()
        estimator_error = float(np.linalg.norm(estimate - rel_pos_true))
        max_estimator_error = max(max_estimator_error, estimator_error)
        sq_error_sum += estimator_error ** 2
        error_samples += 1
        if now >= CONVERGENCE_WINDOW_S and estimator_error > DIVERGENCE_THRESHOLD_M:
            diverged = True

        # --- guidance: True ProNav on the *estimate*, never on truth -------
        solution = guidance.compute(estimate, ekf.relative_velocity(), dt)
        if solution is None:
            continue

        peak_accel = max(peak_accel, float(np.linalg.norm(solution.accel_body)))
        peak_speed = max(peak_speed, float(np.linalg.norm(solution.velocity_body)))
        if solution.saturated:
            saturated_steps += 1

        # --- vehicle plant: first-order velocity tracking ------------------
        drone_vel += (solution.velocity_body - drone_vel) * (dt / PLANT_TAU_S)
        drone_pos += drone_vel * dt

        min_range = min(min_range, float(np.linalg.norm(target_pos - drone_pos)))

    final_miss_distance = float(np.linalg.norm(target_pos - drone_pos))
    rms_error = math.sqrt(sq_error_sum / error_samples) if error_samples else float("nan")
    success = (not diverged) and (final_miss_distance <= HIT_THRESHOLD_M)

    return {
        "drone_id": drone_id,
        "success": 1 if success else 0,
        "diverged": 1 if diverged else 0,
        "final_miss_meters": round(final_miss_distance, 4),
        "min_range_meters": round(min_range, 4),
        "max_error_meters": round(max_estimator_error, 4),      # estimator error
        "rms_error_meters": round(rms_error, 4),
        "target_speed_mps": round(target_speed, 3),
        "peak_accel_g": round(peak_accel / 9.80665, 3),
        "peak_speed_mps": round(peak_speed, 3),
        "saturated_pct": round(100.0 * saturated_steps / TOTAL_STEPS, 1),
        "measurements_fused": fused,
    }


class _NullLog:
    """The engine logs at warning level on gate rejections; 200 runs is noise."""

    def debug(self, *a, **k): pass
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k): pass
    def critical(self, *a, **k): pass


_NULL_LOG = _NullLog()


# =====================================================================
# EXECUTE BATCH SIMULATION SUITE
# =====================================================================

def main() -> int:
    results = []
    successful_runs = 0

    start_time = time.time()
    for d_id in range(1, NUM_DRONES + 1):
        sim_data = run_single_drone_sim(d_id)
        results.append(sim_data)
        successful_runs += sim_data["success"]
        if d_id % 50 == 0:
            print(f"  ... {d_id}/{NUM_DRONES} instances complete")
    execution_time = round(time.time() - start_time, 2)

    success_rate = 100.0 * successful_runs / NUM_DRONES
    diverged_runs = sum(r["diverged"] for r in results)

    csv_file = "sil_validation_report.csv"
    with open(csv_file, "w", newline="") as output_file:
        writer = csv.DictWriter(output_file, results[0].keys())
        writer.writeheader()
        writer.writerows(results)

    def col(name):
        return np.array([r[name] for r in results], dtype=float)

    miss, rms, mx = col("final_miss_meters"), col("rms_error_meters"), col("max_error_meters")
    closest, accel, speed = col("min_range_meters"), col("peak_accel_g"), col("peak_speed_mps")

    print("\n" + "=" * 62)
    print("                    QA SIMULATION RESULTS")
    print("=" * 62)
    print(f"Status:             {'✓ PASSED' if success_rate == 100 else '⚠ REJECTED'}")
    print(f"Total Drones Run:   {NUM_DRONES}")
    print(f"Engagement:         {INITIAL_GAP_M:.0f} m gap, {SIM_DURATION_SEC:.0f} s, "
          f"target {TARGET_SPEED_RANGE[0]:.0f}-{TARGET_SPEED_RANGE[1]:.0f} m/s")
    print(f"Target Accuracy:    {HIT_THRESHOLD_M} meters")
    print(f"Success Rate:       {success_rate}% ({successful_runs}/{NUM_DRONES} units)")
    print("-" * 62)
    print("ESTIMATOR (LatencyCompensatedEKF)")
    print(f"  divergence >{DIVERGENCE_THRESHOLD_M:.0f} m after {CONVERGENCE_WINDOW_S}s: "
          f"{diverged_runs}/{NUM_DRONES} runs")
    print(f"  RMS error:        median {np.median(rms):6.3f} m   worst {rms.max():6.3f} m")
    print(f"  peak error:       median {np.median(mx):6.3f} m   worst {mx.max():6.3f} m")
    print("-" * 62)
    print("GUIDANCE (True ProNav, N = %.1f)" % NAV_CONSTANT_N)
    print(f"  final miss:       median {np.median(miss):6.3f} m   worst {miss.max():6.3f} m")
    print(f"  closest approach: median {np.median(closest):6.3f} m   worst {closest.max():6.3f} m")
    print(f"  peak accel:       median {np.median(accel):6.2f} G   max {accel.max():6.2f} G "
          f"(limit {MAX_ACCEL_MPS2/9.80665:.1f} G)")
    print(f"  peak speed:       median {np.median(speed):6.2f} m/s max {speed.max():6.2f} m/s "
          f"(limit {MAX_SPEED_MPS:.2f})")
    print("-" * 62)
    print("DIAGNOSTICS -- how the two pass criteria interact with the physics")
    fixed_clock = int((miss <= HIT_THRESHOLD_M).sum())
    closest_pass = int((closest <= HIT_THRESHOLD_M).sum())
    overshot = int((closest < miss - 1e-9).sum())
    print(f"  hit <= {HIT_THRESHOLD_M} m at the t={SIM_DURATION_SEC:.0f}s clock : "
          f"{fixed_clock:3d}/{NUM_DRONES}")
    print(f"  hit <= {HIT_THRESHOLD_M} m at closest approach : {closest_pass:3d}/{NUM_DRONES}")
    print(f"  runs that intercepted and then flew past before the clock: "
          f"{overshot}/{NUM_DRONES}")
    print("    -> a fixed-clock miss samples wherever the vehicle happens to be at 3 s;")
    print("       past intercept that is on the far side and opening again. Terminal")
    print("       engagements are normally scored on closest approach.")
    sensor_floor_range = DIVERGENCE_THRESHOLD_M / (
        _TARGET_PRIOR.width_sigma_m / _TARGET_PRIOR.width_m)
    print("  monocular range 1-sigma from the size prior alone is 0.2 x range,")
    print(f"    so a {DIVERGENCE_THRESHOLD_M:.0f} m error bound is below the sensor floor "
          f"beyond {sensor_floor_range:.0f} m")
    print(f"    (at the {INITIAL_GAP_M:.0f} m start, sigma_r >= "
          f"{0.2*INITIAL_GAP_M:.1f} m -- no estimator can beat that)")
    print("-" * 62)
    print(f"Sim Process Time:   {execution_time} seconds (Headless Data Mode)")
    print(f"Report Generated:   {os.path.abspath(csv_file)}")
    print("=" * 62)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

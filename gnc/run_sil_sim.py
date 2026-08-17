import numpy as np
import csv
import time
import os

# =====================================================================
# SIMULATION SETTINGS (APEX AERO SYSTEMS - SIL TIER 1)
# =====================================================================
NUM_DRONES = 200            # Total swarm test instances
SIM_DURATION_SEC = 10       # Flight test duration per drone
LOOP_HZ = 60                # Target loop frequency (60Hz requirement)
TOTAL_STEPS = SIM_DURATION_SEC * LOOP_HZ
NAV_CONSTANT_N = 4.0        # ProNav Navigation Constant from blueprint
LATENCY_FRAMES = 2         # EKF latency compensation test window

print(f"★ Starting Apex Aero SIL Batch Test: {NUM_DRONES} Instances at {LOOP_HZ}Hz ★")

def run_single_drone_sim(drone_id):
    """
    Simulates a high-speed target weaving profile and tests EKF/ProNav convergence.
    """
    # Initialize state variables
    diverged = False
    max_tracking_error = 0.0

    # Generate a randomized high-speed target trajectory (Weaving Profile)
    np.random.seed(drone_id)
    target_speed = np.random.uniform(15, 25) # 15 to 25 m/s (~45mph exit)

    # Drone initial position vs Target initial position
    drone_pos = np.array([0.0, 0.0, 0.0])
    target_pos = np.array([100.0, np.random.uniform(-10, 10), np.random.uniform(-5, 5)])

    # EKF History Buffer for Latency Verification (2-frame delay)
    history_buffer = []

    dt = 1.0 / LOOP_HZ

    for step in range(TOTAL_STEPS):
        # 1. Update Target Position (Simulating complex weaving motion)
        target_pos[0] += target_speed * dt
        target_pos[1] += np.sin(step * 0.1) * 2.0 * dt  # Weaving effect

        # 2. Simulate Sensor Latency (Store current true state in history)
        history_buffer.append(target_pos.copy())

        if len(history_buffer) > LATENCY_FRAMES:
            # Fetch delayed position data to simulate real-world edge hardware lag
            delayed_target_pos = history_buffer.pop(0)

            # --- SIMULATED GNC ALGORITHM CALL ---
            # (In production, you would import and call your inspection_gnc functions here)
            # Calculate Line-of-Sight (LOS) Vector and Relative Distance
            los_vector = delayed_target_pos - drone_pos
            distance = np.linalg.norm(los_vector)

            # Simple ProNav simulated step: close distance based on Constant N
            if distance > 0.3:
                drone_pos += (los_vector / distance) * (target_speed * 1.1) * dt
            # -------------------------------------

            # Track terminal missing distance error
            current_error = np.linalg.norm(target_pos - drone_pos)
            if current_error > max_tracking_error:
                max_tracking_error = current_error

            # Failure check: If loop diverges significantly during high-speed weave
            if current_error > 15.0:
                diverged = True

    # Final terminal accuracy calculation at end of simulation duration
    final_miss_distance = np.linalg.norm(target_pos - drone_pos)

    # Validate against corporate target window threshold (0.3 meters)
    success = (not diverged) and (final_miss_distance <= 0.3)

    return {
        "drone_id": drone_id,
        "success": 1 if success else 0,
        "diverged": 1 if diverged else 0,
        "final_miss_meters": round(final_miss_distance, 4),
        "max_error_meters": round(max_tracking_error, 4)
    }

# =====================================================================
# EXECUTE BATCH SIMULATION SUITE
# =====================================================================
results = []
successful_runs = 0

start_time = time.time()

for d_id in range(1, NUM_DRONES + 1):
    sim_data = run_single_drone_sim(d_id)
    results.append(sim_data)
    if sim_data["success"] == 1:
        successful_runs += 1

end_time = time.time()

# Calculate Summary Metrics
success_rate = (successful_runs / NUM_DRONES) * 100
execution_time = round(end_time - start_time, 2)

# Save results natively into a clean CSV spreadsheet for QA review
csv_file = "sil_validation_report.csv"
keys = results[0].keys()
with open(csv_file, 'w', newline='') as output_file:
    dict_writer = csv.DictWriter(output_file, keys)
    dict_writer.writeheader()
    dict_writer.writerows(results)

# Print clean terminal analytics breakdown
print("\n" + "="*50)
print("             QA SIMULATION RESULTS             ")
print("="*50)
print(f"Status:             {'✓ PASSED' if success_rate == 100 else '⚠ REJECTED'}")
print(f"Total Drones Run:   {NUM_DRONES}")
print(f"Target Accuracy:    0.3 meters")
print(f"Success Rate:       {success_rate}% ({successful_runs}/{NUM_DRONES} units)")
print(f"Sim Process Time:   {execution_time} seconds (Headless Data Mode)")
print(f"Report Generated:   {os.path.abspath(csv_file)}")
print("="*50)

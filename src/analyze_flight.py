import pandas as pd
import numpy as np
import glob
import os
import matplotlib.pyplot as plt

# 1. Automatically detect the latest flight files in your folder
pos_files = glob.glob('*_vehicle_local_position_0.csv')
traj_files = glob.glob('*_trajectory_setpoint_0.csv')

if not pos_files or not traj_files:
    print("Error: Missing CSV files. Make sure you copied them here using 'cp'.")
    exit()

# Pick the newest files based on modification time
pos_files.sort(key=os.path.getmtime)
traj_files.sort(key=os.path.getmtime)
actual_file = pos_files[-1]
target_file = traj_files[-1]

print(f"Analyzing actual flight data from: {actual_file}")
print(f"Analyzing target trajectory data from: {target_file}")

# 2. Load the data
actual_df = pd.read_csv(actual_file)
target_df = pd.read_csv(target_file)

actual_time = actual_df['timestamp'].values
actual_x = actual_df['x'].values
actual_y = actual_df['y'].values
actual_z = actual_df['z'].values

target_time = target_df['timestamp'].values
target_x = target_df['position[0]'].values
target_y = target_df['position[1]'].values
target_z = target_df['position[2]'].values

# 3. Synchronize timelines
print("Synchronizing timelines...")
sync_target_x = np.interp(actual_time, target_time, target_x)
sync_target_y = np.interp(actual_time, target_time, target_y)
sync_target_z = np.interp(actual_time, target_time, target_z)

# 4. Calculate Root Mean Square Error (RMSE)
print("Calculating Error Metrics...")
rmse_x = np.sqrt(np.nanmean((actual_x - sync_target_x)**2))
rmse_y = np.sqrt(np.nanmean((actual_y - sync_target_y)**2))
rmse_z = np.sqrt(np.nanmean((actual_z - sync_target_z)**2))

# 5. Print Results
print("\n" + "="*40)
print("FLIGHT CONTROL PERFORMANCE SUMMARY")
print("="*40)
print(f"X-Axis Tracking Error (RMSE): {rmse_x:.4f} meters")
print(f"Y-Axis Tracking Error (RMSE): {rmse_y:.4f} meters")
print(f"Z-Axis Tracking Error (RMSE): {rmse_z:.4f} meters")
print("="*40)

# 6. Generate 3D Flight Plot
print("Generating 3D Flight Plot...")
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ax.plot(target_x, target_y, target_z, 'r--', label='Target Path (ROS 2 Setpoint)', alpha=0.7)
ax.plot(actual_x, actual_y, actual_z, 'b-', label='Actual Flight (PX4 Telemetry)', linewidth=2)

ax.set_title("Autonomous Lawnmower Search Grid: Target vs Actual")
ax.set_xlabel("X Position (meters)")
ax.set_ylabel("Y Position (meters)")
ax.set_zlabel("Altitude/Z (meters)")
ax.legend()

plt.savefig("thesis_flight_plot.png", dpi=300)
print("Saved dynamic thesis_flight_plot.png to your folder!")

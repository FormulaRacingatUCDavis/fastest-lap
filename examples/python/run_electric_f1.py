import os
import sys
import matplotlib.pyplot as plt
import matplotlib.collections as mcoll
import numpy as np

# Add current path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import fastest_lap

# ==== CONFIGURATION ====
TRACK_NAME = "catalunya"  # Easily switch tracks here (e.g., "laguna_seca", "zolder")
VEHICLE_XML = "limebeer-2014-f1-electric.xml"
# ======================

# Setup paths (assuming we're in examples/python)
base_path = "../../"
vehicle_db = os.path.join(base_path, f"database/vehicles/f1/{VEHICLE_XML}")
track_db   = os.path.join(base_path, f"database/tracks/{TRACK_NAME}/{TRACK_NAME}.xml")

print(f"Initializing Fastest Lap for {TRACK_NAME}...")
fastest_lap.set_print_level(0)

# Create the electric vehicle
print("Loading electric vehicle model...")
fastest_lap.create_vehicle_from_xml("elec_car", vehicle_db)

# Create the track
print(f"Loading track: {TRACK_NAME}...")
fastest_lap.create_track_from_xml(TRACK_NAME, track_db)

# ==== FORMULA E PHYSICS CONFIGURATION ====
# Set motor to 350kW FE power and 1500Nm torque
# Using the full internal path with 'vehicle/' prefix
fastest_lap.vehicle_set_parameter("elec_car", "vehicle/rear-axle/electric-motor/maximum-torque", 1500.0)
fastest_lap.vehicle_set_parameter("elec_car", "vehicle/rear-axle/electric-motor/maximum-power", 350.0)

# Set aerodynamics to FE specs (low drag, low downforce, heavier battery mass)
fastest_lap.vehicle_set_parameter("elec_car", "vehicle/chassis/aerodynamics/cd", 0.4)
fastest_lap.vehicle_set_parameter("elec_car", "vehicle/chassis/aerodynamics/cl", 1.0)
fastest_lap.vehicle_set_parameter("elec_car", "vehicle/chassis/mass", 900.0)

# ==== SIMULATE A FULL CLOSED LAP ====
# Get the full length of the track
track_len = fastest_lap.track_download_length(TRACK_NAME)
s_lap = [i * 10.0 for i in range(int(track_len/10.0))] 

# Compute optimal lap using the <is_closed> true option
options  = "<options>"
options += "    <is_closed>true</is_closed>"
options += "    <output_variables>"
options += "        <prefix>run/</prefix>"
options += "    </output_variables>"
options += "</options>"

print(f"Computing optimal FULL CLOSED lap for {TRACK_NAME}... (This will take a few minutes)")
prefix, variable_list = fastest_lap.optimal_laptime("elec_car", TRACK_NAME, s_lap, options)

# Download telemetry data
print("Downloading lap telemetry...")
varmap = fastest_lap.download_variables(prefix, variable_list)

# Download track boundaries for visualization
print("Downloading track geometry...")
l_x = fastest_lap.track_download_data(TRACK_NAME, "left.x")
l_y = fastest_lap.track_download_data(TRACK_NAME, "left.y")
r_x = fastest_lap.track_download_data(TRACK_NAME, "right.x")
r_y = fastest_lap.track_download_data(TRACK_NAME, "right.y")

# Extract simulation results
s_dist    = np.array(varmap['road.arclength'])
speed_kmh = np.array(varmap['chassis.velocity.x']) * 3.6
throttle  = np.array(varmap['chassis.throttle'])
car_x     = np.array(varmap['chassis.position.x'])
car_y     = np.array(varmap['chassis.position.y'])

# ==== GENERATE PLOTS ====
print("Generating comprehensive plot...")
fig = plt.figure(figsize=(15, 12))
gs = fig.add_gridspec(3, 2)

# Subplot 1: Speed Profile
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(s_dist, speed_kmh, color="purple", lw=2)
ax1.set_ylabel("Speed [km/h]")
ax1.set_title(f"Speed Profile - {TRACK_NAME}")
ax1.grid(True, alpha=0.3)

# Subplot 2: Throttle/Brake
ax2 = fig.add_subplot(gs[1, 0])
ax2.plot(s_dist, throttle, color="green", lw=1.5)
ax2.set_xlabel("Distance [m]")
ax2.set_ylabel("Throttle [-1...1]")
ax2.grid(True, alpha=0.3)

# Subplot 3: Track Map with Racing Line
ax3 = fig.add_subplot(gs[:, 1])
# Plot boundaries
ax3.plot(l_x, l_y, color="gray", alpha=0.5, lw=1)
ax3.plot(r_x, r_y, color="gray", alpha=0.5, lw=1)
# Plot color-coded racing line
points = np.array([car_x, car_y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = plt.Normalize(speed_kmh.min(), speed_kmh.max())
lc = mcoll.LineCollection(segments, cmap='jet', norm=norm)
lc.set_array(speed_kmh)
lc.set_linewidth(3)
line = ax3.add_collection(lc)
fig.colorbar(line, ax=ax3, label="Speed [km/h]")

ax3.set_aspect('equal')
ax3.set_title(f"Racing Line Overlay - {TRACK_NAME}")
ax3.set_xlabel("X [m]")
ax3.set_ylabel("Y [m]")
ax3.grid(True, alpha=0.1)

save_path = "electric_lap_results.png"
plt.tight_layout()
plt.savefig(save_path, dpi=150)
print(f"Success! Full results saved to: {save_path}")

# Display plot if running interactively
plt.show()

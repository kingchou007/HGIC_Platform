import numpy as np
import matplotlib.pyplot as plt

# Load the data from the CSV
name = "cc_"
velocity_data = np.loadtxt("cc.csv", delimiter=",")

# Extract velocities for X and Y directions
velocities_x = velocity_data[:, 0::3]
velocities_y = velocity_data[:, 1::3]

# Calculate average, max, and min for X and Y directions
avg_vel_x = np.mean(velocities_x, axis=1)
avg_vel_y = np.mean(velocities_y, axis=1)

max_vel_x = np.max(velocities_x, axis=1)
max_vel_y = np.max(velocities_y, axis=1)

min_vel_x = np.min(velocities_x, axis=1)
min_vel_y = np.min(velocities_y, axis=1)

# Plotting the average velocities for X and Y directions in one figure
plt.figure(figsize=(12, 5))

# X velocities
plt.fill_between(
    range(avg_vel_x.shape[0]), min_vel_x, max_vel_x, color="lightcoral", alpha=0.3
)
plt.plot(avg_vel_x, color="red", label="X")

# Y velocities
plt.fill_between(
    range(avg_vel_y.shape[0]), min_vel_y, max_vel_y, color="lightgreen", alpha=0.3
)
plt.plot(avg_vel_y, color="green", label="Y")

plt.legend(loc="upper right")
plt.tight_layout()
plt.grid(True, which="both", linestyle="--", linewidth=1)
plt.savefig(name + "velocity.png", dpi=600)
plt.show()

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Constants
num_uavs = 9
group_center = np.array([0, 0, 0])
z_height = 5  # The height at which the UAVs are flying

# Function to generate UAV positions for each formation type
def generate_positions(formation_type):
    if formation_type == 'circle':
        formation_angle_offset = 2 * np.pi / num_uavs
        for i in range(num_uavs):
            formation_angle = formation_angle_offset * i
            yield group_center + spacing * np.array([np.cos(formation_angle), np.sin(formation_angle), z_height])
                
    elif formation_type == 'line':
        for i in range(num_uavs):
            yield np.array([i * spacing, group_center[1], z_height])
                     
    elif formation_type == 'diagonal':
        for i in range(num_uavs):
            row = col = i  # For a diagonal, row index equals column index
            yield np.array([group_center[0] + (col * spacing), group_center[1] + row * spacing, z_height])
                         
    elif formation_type == 'V':
        for i in range(num_uavs):
            if i == num_uavs // 2:  # For the head of the V
                yield group_center + np.array([0, 0, z_height])
            else:
                yield group_center + np.array([abs((i - num_uavs // 2)) * spacing, (i - num_uavs // 2) * spacing, z_height])

# Formation types
formation_types = ['circle', 'line', 'diagonal', 'V']
colors = ['blue', 'green', 'orange', 'purple']
# Modify the spacing constant
spacing = 15

# Create a figure with a single subplot for all formations
fig = plt.figure(figsize=(16, 16))
ax = fig.add_subplot(111, projection='3d')

for j, formation_type in enumerate(formation_types):
    # Generate and plot the UAV positions
    positions = list(generate_positions(formation_type))
    for i, position in enumerate(positions):
        # Color the head of the V formation red
        if formation_type == 'V' and i == num_uavs // 2:
            ax.scatter(*position, color='red')
        else:
            ax.scatter(*position, color=colors[j])

    # Connect the UAVs with lines
    xs, ys, zs = zip(*positions)
    
    # If the formation type is 'circle', connect the first and last UAVs and add a center point
    if formation_type == 'circle':
        xs = xs + (xs[0],)
        ys = ys + (ys[0],)
        zs = zs + (zs[0],)
        ax.scatter(*group_center, color='red')  # Center point
        
    ax.plot(xs, ys, zs, color=colors[j])

# Set the title of the plot
ax.set_title("UAV Formations")

# Remove the specific values on the x, y, and z axes
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])

# Set the aspect of the plot to be equal
ax.set_box_aspect([1,1,1])

# Set the x, y and z axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Increase the x and y limits
ax.set_xlim([-100, 100])
ax.set_ylim([-100, 100])

# Show the plot
plt.show()

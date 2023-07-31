# from mpl_toolkits.mplot3d import Axes3D
# import numpy as np
# from matplotlib import pyplot as plt

# # # Constants
# # num_uavs = 9
# # spacing = 1
# # group_center = np.array([0, 0, 0])
# # z_height = 5  # The height at which the UAVs are flying

# # def generate_positions(formation_type):
# #     if formation_type == 'circle':
# #         formation_angle_offset = 2 * np.pi / num_uavs
# #         for i in range(num_uavs):
# #             formation_angle = formation_angle_offset * i
# #             yield group_center + spacing * np.array([np.cos(formation_angle), np.sin(formation_angle), 0])  # All at the same height
                
# #     elif formation_type == 'line':
# #         for i in range(num_uavs):
# #             yield np.array([i * spacing, group_center[1], z_height])
                     
# #     elif formation_type == 'diagonal':
# #         for i in range(num_uavs):
# #             row = col = i  # For a diagonal, row index equals column index
# #             yield np.array([group_center[0] + (col * spacing), group_center[1] + row * spacing, z_height])
                         
# #     elif formation_type == 'V':
# #         for i in range(num_uavs):
# #             if i == num_uavs // 2:  # For the head of the V
# #                 yield group_center + np.array([0, 0, z_height])
# #             else:
# #                 yield group_center + np.array([abs((i - num_uavs // 2)) * spacing, (i - num_uavs // 2) * spacing, z_height])
# # # Formation types
# # formation_types = ['circle', 'line', 'dia']gonal', 'V

# # # Create a 3D plot for each formation
# # for formation_type in formation_types:
# #     # Create a new figure
# #     fig = plt.figure(figsize=(8, 8))
# #     ax = fig.add_subplot(111, projection='3d')

# #     # Generate and plot the UAV positions
# #     positions = list(generate_positions(formation_type))
# #     for i, position in enumerate(positions):
# #         # Color the head of the V formation red
# #         if formation_type == 'V' and i == num_uavs // 2:
# #             ax.scatter(*position, color='red', marker='*')
# #         else:
# #             ax.scatter(*position, color='blue')

# #     # Connect the UAVs with lines
# #     xs, ys, zs = zip(*positions)
    
# #     # If the formation type is 'circle', connect the first and last UAVs and add a center point
# #     if formation_type == 'circle':
# #         xs = xs + (xs[0],)
# #         ys = ys + (ys[0],)
# #         zs = zs + (zs[0],)
# #         ax.scatter(*group_center, color='red')
        
# #     ax.plot(xs, ys, zs, color='black')

# #     # Remove the specific values on the x, y, and z axes
# #     ax.set_xticklabels([])
# #     ax.set_yticklabels([])
# #     ax.set_zticklabels([])

# #     # Set the aspect of the plot to be equal
# #     ax.set_box_aspect([1,1,1])

# #     # Set the x, y and z axis labels
# #     ax.set_xlabel('X')
# #     ax.set_ylabel('Y')
# #     ax.set_zlabel('Z')

# # # Show the plots
# # plt.show()


# # Constants
# num_uavs = 9
# spacing = 12
# group_center = np.array([0, 0, 0])

# # Function to generate UAV positions for each formation type with different heights for each formation
# def generate_positions(formation_type):
#     if formation_type == 'circle':
#         formation_angle_offset = 2 * np.pi / num_uavs
#         for i in range(num_uavs):
#             formation_angle = formation_angle_offset * i
#             yield group_center + spacing * np.array([np.cos(formation_angle), np.sin(formation_angle), 20])  # All at the same height
               
#     elif formation_type == 'line':
#         for i in range(num_uavs):
#             yield np.array([i * spacing, group_center[1], 20])  # Set height to 20
                     
#     elif formation_type == 'diagonal':
#         for i in range(num_uavs):
#             row = col = i  # For a diagonal, row index equals column index
#             yield np.array([group_center[0] + (col * spacing), group_center[1] + row * spacing, 30])  # Set height to 30
                         
#     elif formation_type == 'V':
#         for i in range(num_uavs):
#             if i == num_uavs // 2:  # For the head of the V
#                 yield group_center + np.array([0, 0, 40])  # Set height to 40 for head of V
#             else:
#                 yield group_center + np.array([abs((i - num_uavs // 2)) * spacing, (i - num_uavs // 2) * spacing, 40])  # Set height to 40 for others

# # Create a figure with a single subplot for all formations
# fig = plt.figure(figsize=(16, 16))
# ax = fig.add_subplot(111, projection='3d')


# ax = fig.add_subplot(111, projection='3d')
# formation_types = ['circle', 'line', 'diagonal', 'V']
# # Define colors for each formation type
# colors = ['blue', 'green', 'orange', 'purple']

# for j, formation_type in enumerate(formation_types):
#     # Generate and plot the UAV positions
#     positions = list(generate_positions(formation_type))
#     for i, position in enumerate(positions):
#         # Color the head of the V formation red
#         if formation_type == 'V' and i == num_uavs // 2:
#             ax.scatter(*position, color='red')
#         else:
#             ax.scatter(*position, color=colors[j])

#     # Connect the UAVs with lines
#     xs, ys, zs = zip(*positions)
    
#     # If the formation type is 'circle', connect the first and last UAVs and add a center point
#     if formation_type == 'circle':
#         xs = xs + (xs[0],)
#         ys = ys + (ys[0],)
#         zs = zs + (zs[0],)
#         ax.scatter(*group_center, color='red')  # Center point
        
#     ax.plot(xs, ys, zs, color=colors[j])

# # Set the title of the plot
# ax.set_title("UAV Formations")

# # Remove the specific values on the x, y, and z axes
# ax.set_xticklabels([])
# ax.set_yticklabels([])
# ax.set_zticklabels([])

# # Set the aspect of the plot to be equal
# ax.set_box_aspect([1,1,1])

# # Set the x, y and z axis labels
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # Increase the x and y limits
# ax.set_xlim([-50, 50])
# ax.set_ylim([-50, 50])

# # Show the plot
# plt.show()

import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
 
# # Set the target point
# target_point = np.array([0, 0])  

# # Define the boundaries
# y_max = target_point[1] - 50
# y_min = target_point[1] + 50
# x_max = target_point[0] - 50
# x_min = target_point[0] + 50

# # Number of drones
# n_drones = 9

#             # Randomly generating drone positions within the specified area
#             # np.random.seed(0)  # for consistency
#             # drone_positions = np.random.rand(n_drones, 2) * [x_max - x_min, y_max - y_min] + [x_min, y_min]
#             # Compute the number of drones per row/column (assuming a square grid)
# n_per_row = int(np.sqrt(n_drones))

#             # Compute the spacing between drones
# x_spacing = (x_max - x_min) / (n_per_row - 1)
# y_spacing = (y_max - y_min) / (n_per_row - 1)
#             # Generate drone positions on the grid
# drone_positions = np.zeros((n_drones, 2))
# for i in range(n_drones):
#     row = i // n_per_row
#     col = i % n_per_row
#     drone_positions[i] = [x_min + col * x_spacing, y_min + row * y_spacing]

# vor = Voronoi(drone_positions)  # Compute Voronoi diagram
# # Plot Voronoi diagram
# fig, ax = plt.subplots()
# voronoi_plot_2d(vor, show_vertices=False, ax=ax)
# # ax.scatter(drone_positions[:,0], drone_positions[:,1], color='red')  # Plot drone positions

# # Compute and plot the centroids
# for i in range(n_drones):
#     region = vor.point_region[i]
#     if not -1 in vor.regions[region]:  # Ensure the Voronoi cell is a valid polygon
#         polygon = [vor.vertices[j] for j in vor.regions[region]]
#         centroid = np.mean(polygon, axis=0)
#         ax.scatter(centroid[0], centroid[1], color='green', marker="x")  # Plot centroid

# ax.set_xlim([x_min, x_max])
# ax.set_ylim([y_min, y_max])
# plt.show()

import numpy as np
import matplotlib.pyplot as plt

num_uavs = 10  # total number of UAVs
spacing = 8  # spacing between each UAV
group_center_radius = 70  # radius of the center of the group

for i in range(num_uavs):
    ts = np.linspace(0, 600, 600)  # let's say t varies from 0 to 600
    points = []
    
    for t in ts:
        angle = 2 * np.pi * t / 600
        group_center = group_center_radius * np.array([np.cos(angle), np.sin(angle)])

        # calculating the point for each UAV
        point = np.array([abs((i - num_uavs // 2)) * spacing, 
                          (i - num_uavs // 2) * spacing])
        formation_angle = angle - np.pi * i / 10  # rotate the formation by 90 degrees
        rotation_matrix = np.array([[np.cos( formation_angle), -np.sin( formation_angle)], 
                                             [np.sin( formation_angle), np.cos( formation_angle)]])

        rotated_point = np.dot(rotation_matrix, point) + group_center

        points.append(rotated_point)

    points = np.array(points).T  # transpose to get x and y coordinates
    plt.plot(points[0], points[1])  # plot the trajectory for each UAV

plt.show()

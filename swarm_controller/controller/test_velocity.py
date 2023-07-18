# import airsim
# import time
# import numpy as np

# class velocityComputation23:
#     def __init__(self):
#         # Build a connection with AirSim
#         self.client = airsim.MultirotorClient()
#         self.client.confirmConnection()

#         # Define the origin position of the swarm
#         self.origin = [[0, 0], [2, 0], [4, 0], 
#                        [0, -3], [2, -2], [4, -3], 
#                        [0, 3], [2, 2], [4, 3]]
        
#         self.origin_x = [0, 2, 4, 0, 2, 4, 0, 2, 4]
#         self.origin_y = [0, 0, 0, -3, -2, -3, 3, 2, 3]
#         self.origin_z = [] # most action will be in 2D plane
        
#         self.num_uavs = len(self.origin) # can be selected from 1 to 9
#         self.pos_mig = np.array([[0], [0]])
        
#     def set_parameters(self, v_max=0, r_max=0, k_sep=0, 
#                        k_coh=0, k_mig=0, k_rep=0, r_repulsion=0, 
#                        d_desired=0):
#         self.v_max = v_max
#         self.r_max = r_max
#         self.k_sep = k_sep
#         self.k_coh = k_coh
#         self.k_mig = k_mig
#         self.r_repulsion = r_repulsion
#         self.d_desired = d_desired
#         self.k_rep = k_rep
        
#         self.v_cmd = np.zeros([2, self.num_uavs])
#         self.v_rep = np.zeros([2, 1])
#         self.v_coh = np.zeros([2, 1])
#         self.v_sep = np.zeros([2, 1])
#         self.z_cmd = self.get_avg_altitude()
        
#     def get_UAV_pos(self, vehicle_name="SimpleFlight"):
#         state = self.client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
#         x = state.position.x_val
#         y = state.position.y_val
#         i = int(vehicle_name[3])
#         x += self.origin_x[i - 1]
#         y += self.origin_y[i - 1]
#         pos = np.array([[x], [y]]) # Return a 2D array
#         return pos

#     def take_off(self):
#         for i in range(self.num_uavs):
#             name_i = "UAV" + str(i + 1)
#             self.client.enableApiControl(True, name_i)
#             self.client.armDisarm(True, name_i)
#             if i != self.num_uavs - 1:
#                 self.client.takeoffAsync(vehicle_name=name_i)
#             else:
#                 self.client.takeoffAsync(vehicle_name=name_i).join()
        
#         time.sleep(0.5)
        
#         # fly to same altitude
#         for i in range(self.num_uavs):
#             name = "UAV" + str(i + 1)
#             if i != self.num_uavs - 1:
#                 self.client.moveToZAsync(-3, 1, vehicle_name=name)
#             else:
#                 self.client.moveToZAsync(-3, 1, vehicle_name=name).join()
                
#     def land(self):
#         for i in range(9):
#             name_i = "UAV"+str(i+1)
#             if i != self.num_uavs - 1: 
#                 self.client.landAsync(vehicle_name=name_i)
#             else:
#                 self.client.landAsync(vehicle_name=name_i).join()
            
#     def get_avg_altitude(self):
#         # get current z position of all UAVs
#         swam_altitude = [self.client.getMultirotorState(vehicle_name="UAV" + str(i + 1)).kinematics_estimated.position.z_val for i in range(self.num_uavs)]
#         z_cmd = np.mean(swam_altitude)
#         return z_cmd
    
#     ### general velocity without repulsion
#     def compute_velocity_command(self):
#         v_cmd = np.zeros([2, self.num_uavs])
#         z_cmd = self.get_avg_altitude()

#         for i in range(self.num_uavs):
#             name_i = "UAV"+str(i+1)
#             pos_i = self.get_UAV_pos(vehicle_name=name_i)
#             pos_mig = pos_i + self.shift
#             r_mig = pos_mig - pos_i
#             v_mig = self.k_mig * r_mig / np.linalg.norm(r_mig)
#             v_sep = np.zeros([2, 1])
#             v_coh = np.zeros([2, 1]) 
#             N_i = 0

#             for j in range(self.num_uavs):
#                 if j != i:
#                     N_i += 1
#                     name_j = "UAV"+str(j+1)
#                     pos_j = self.get_UAV_pos(vehicle_name=name_j)
#                     if np.linalg.norm(pos_j - pos_i) < self.r_max:
#                         r_ij = pos_j - pos_i
#                         v_sep += -self.k_sep * r_ij / np.linalg.norm(r_ij)
#                         v_coh += self.k_coh * r_ij 

#             if N_i > 0:
#                 v_sep = v_sep / N_i
#                 v_coh = v_coh / N_i
                
#             v_cmd[:, i] = (v_sep + v_coh + v_mig).flatten()
#             if v_cmd[0, i] > self.v_max:
#                 v_cmd[0, i] = self.v_max

#         return v_cmd, z_cmd

#     def get_vehicle_name(self, i):
#         return "UAV" + str(i + 1)
    
#     def execute_for_all_uavs(self, func):
#         for i in range(self.num_uavs):
#             name_i = self.get_vehicle_name(i)
#             func(name_i)
    
#     # used for left, right, forward, backward
#     def move_in_direction(self, shift, t=100):
#         self.shift = shift
#         for _ in range(t):
#             v_cmd, z_cmd = self.compute_velocity_command()
#             for i in range(self.num_uavs):
#                 name_i = self.get_vehicle_name(i)
#                 self.client.moveByVelocityZAsync(v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i)
    
#     # Compute Merge and spread velocity
#     def get_swarm_center(self):
#         # get current position of all UAVs
#         temp_pos = []
#         for i in range(self.num_uavs):
#             name_i = "UAV"+str(i+1)
#             temp_pos.append(self.get_UAV_pos(vehicle_name=name_i))
            
#         return np.mean(temp_pos, axis=0)
    
#     def _compute_force_between_uavs(self, i, pos_i, force_rule):
#         # Initialize the total force and number of neighboring UAVs
#         v = np.zeros([2, 1])
#         num_neighbors = 0

#         for j in range(self.num_uavs):
#             # Skip the current UAV
#             if j != i:
#                 # Get the position of the j-th UAV
#                 pos_j = self.get_UAV_pos(f"UAV{j + 1}")
#                 # Compute the vector from the i-th to the j-th UAV
#                 r_ij = pos_j - pos_i
#                 # Compute the distance between the i-th and the j-th UAV
#                 dist = np.linalg.norm(r_ij)
#                 # Calculate the force between the i-th and the j-th UAV and whether they are neighbors
#                 force_ij, count = force_rule(dist, r_ij)
#                 # Add the force to the total force
#                 v += force_ij
#                 # Increase the number of neighbors
#                 num_neighbors += count

#         # If there are any neighbors, average the total force
#         if num_neighbors > 0:
#             v /= num_neighbors

#         return v

#     ### calculate merge formation ####
#     def _cohesion_force_rule(self, dist, r_ij):
#         # If the UAVs are further apart than the desired distance but within the maximum radius
#         if self.d_desired < dist < self.r_max:
#             return self.k_coh * (dist - self.d_desired) * r_ij / dist, 1
#         else:
#             return np.zeros([2, 1]), 0

#     def _separation_force_rule(self, dist, r_ij):
#         # If the UAVs are closer together than the desired distance, should move apart
#         if dist < self.d_desired:
#             return -self.k_sep * (self.d_desired - dist) * r_ij / dist, 1
#         else:
#             return np.zeros([2, 1]), 0
        
#     def _repulsion_force_rule(self, dist, r_ij):
#         # If the UAVs are closer together than the repulsion distance, should strongly move apart
#         if dist < self.r_repulsion:
#             return -self.k_rep * (self.r_repulsion - dist) * r_ij / dist, 1
#         else:
#             return np.zeros([2, 1]), 0

#     ### compute every uavs ###
#     def compute_cohesion_force(self, i, pos_i):
#         return self._compute_force_between_uavs(i, pos_i, self._cohesion_force_rule)

#     def compute_separation_force(self, i, pos_i):
#         return self._compute_force_between_uavs(i, pos_i, self._separation_force_rule)

#     def compute_repulsion_force(self, i, pos_i):
#         return self._compute_force_between_uavs(i, pos_i, self._repulsion_force_rule)
    
#     # Limit the maximum velocity of the UAVs  
#     def limit_velocity(self, v_cmd):
#         v_cmd_norm = np.linalg.norm(v_cmd, axis=0)
#         v_cmd_norm[v_cmd_norm > self.v_max] = self.v_max
#         v_cmd = v_cmd * (v_cmd_norm / np.maximum(v_cmd_norm, 1e-7))
#         return v_cmd
    
#     def compute_forces(self):
#         v_cmd = np.zeros([2, self.num_uavs])
        
#         for i in range(self.num_uavs):
#             name_i = "UAV" + str(i + 1)
#             pos_i = self.get_UAV_pos(vehicle_name=name_i)
#             v_sep = self.compute_separation_force(i, pos_i)
#             v_coh = self.compute_cohesion_force(i, pos_i)
#             v_rep = self.compute_repulsion_force(i, pos_i)
#             r_mig = self.get_swarm_center() - pos_i
#             v_mig = self.k_mig * r_mig / np.linalg.norm(r_mig)
#             v_cmd[:, i] = (v_sep + v_coh + v_rep + v_mig).flatten()
    
#     def stop_move(self):
#         z_cmd = self.get_avg_altitude()
#         for i in range(self.num_uavs):
#             name_i = self.get_vehicle_name(i)
#             self.client.moveByVelocityZAsync(0, 0, z_cmd, 0.1, vehicle_name=name_i)
            
#     # calculate merge formation (spread and merge) and compute velocity
#     def compute_merge_forces(self):
#         v_cmd = np.zeros([2, self.num_uavs])
#         for i in range(self.num_uavs):
#             name_i = "UAV" + str(i + 1)
#             pos_i = self.get_UAV_pos(vehicle_name=name_i)
#             v_sep = self.compute_separation_force(i, pos_i)
#             v_coh = self.compute_cohesion_force(i, pos_i)
#             v_rep = self.compute_repulsion_force(i, pos_i)
#             r_mig = self.get_swarm_center() - pos_i
#             v_mig = self.k_mig * r_mig / np.linalg.norm(r_mig)
#             v_cmd[:, i] = (v_sep + v_coh + v_rep + v_mig).flatten()  
#         return v_cmd
    
#     def compute_spread_force(self):
#         self.shift = self.get_swarm_center()
#         for _ in range(self.num_uavs):
#             v_cmd, _ = self.compute_velocity_command()
            
#         return v_cmd
    
    
#     ### compute circle formation ###
#     def form_circle(self):
#     # Set the desired formation parameters
  
#         formation_radius = 5
#         safe_distance = 2.5
#         repulsion_distance = 1.5
#         formation_angle_offset = 2 * np.pi / 9
   
#         group_center = self.get_swarm_center()
        

#         # Main loop to control UAVs
#         for t in range(500): # assuming 200 time steps are enough to form the circle
#             for i in range(9):
#                 # Define the name and position of the current UAV
#                 name_i = "UAV" + str(i + 1)
#                 pos_i = self.get_UAV_pos (vehicle_name=name_i)

#                 # Calculate the formation point for the current UAV
#                 formation_angle = formation_angle_offset * i
#                 formation_point = group_center + formation_radius * np.array(
#                     [[np.cos(formation_angle)], [np.sin(formation_angle)]])

#                 # Compute the desired velocity for the current UAV
#                 v_mig = self.k_mig * (formation_point - pos_i)

#                 # Perform collision avoidance
#                 v_rep = np.zeros([2, 1])
#                 for j in range(9):
#                     if j != i:
#                         # Define the name and position of the other UAV
#                         name_j = "UAV" + str(j + 1)
#                         pos_j = self.get_UAV_pos (vehicle_name=name_j)

#                         # Compute the repulsion vector if the other UAV is too close
#                         distance = np.linalg.norm(pos_j - pos_i)
#                         if distance < safe_distance:
#                             repulsion_vector = pos_i - pos_j
#                             v_rep += self.k_rep * (repulsion_vector / np.linalg.norm(repulsion_vector)) * (
#                                         safe_distance - distance)
#                             if distance < repulsion_distance:
#                                 v_rep *= 2

#                 # Store the computed velocity command for the current UAV
#                 self.v_cmd[:, i:i + 1] = v_mig + v_rep
                
                
        
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
#     def compute_velocity(self, k_sep, k_coh, add_rep):

#         for i in range(9):
#             name_i = "UAV" + str(i + 1)
#             pos_i = self.get_UAV_pos(vehicle_name=name_i)
#             r_mig = self.pos_mig - pos_i
#             v_mig = self.k_mig * r_mig / np.linalg.norm(r_mig)
#             v_sep, v_coh, N_i = self.compute_forces(pos_i, i, k_sep, k_coh, add_rep)
#             v_sep /= N_i
#             v_coh /= N_i
#             self.v_cmd[:, i:i + 1] = v_sep + v_coh + v_mig

#     def compute_forces(self, pos_i, i, k_sep, k_coh, add_rep):
#         v_sep = np.zeros([2, 1])
#         v_coh = np.zeros([2, 1])
#         N_i = 0
#         for j in range(9):
#             if j != i:
#                 N_i += 1
#                 name_j = "UAV" + str(j + 1)
#                 pos_j = self.get_UAV_pos(vehicle_name=name_j)
#                 r_ij = pos_j - pos_i
#                 if np.linalg.norm(r_ij) < self.r_max:
#                     v_sep += -k_sep * r_ij / np.linalg.norm(r_ij)
#                     if not add_rep:
#                         if np.linalg.norm(r_ij) < self.r_repulsion:
#                             v_sep += -self.k_rep * (self.r_repulsion - np.linalg.norm(r_ij)) * r_ij / np.linalg.norm(r_ij)
#                         elif np.linalg.norm(r_ij) < self.d_desired:
#                             v_sep += -k_sep * (self.d_desired - np.linalg.norm(r_ij)) * r_ij / np.linalg.norm(r_ij)
#                     v_coh += k_coh * r_ij
#         return v_sep, v_coh, N_i
    
#     def move_UAVs(self, z_cmd):
#         for i in range(self.num_uavs):
#             name_i = "UAV" + str(i + 1)
#             self.client.moveByVelocityZAsync(self.v_cmd[0, i], self.v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i)

    
    
     
#     ### compute circle formation ###
#     def spread(self):
#         self.set_parameters(v_max=10, r_max=20, k_sep=30, k_coh=1, k_mig=1)
#         self.pos_mig = self.get_swarm_center()
#         z_cmd = self.get_avg_altitude()
#         for t in range(300):
#             self.compute_velocity(30, 1, True)
#             self.move_UAVs(z_cmd)
            
#     def merge(self):
#         self.set_parameters(v_max=3, r_max=20, k_sep= 1.5, k_coh=0.15, k_mig=1, k_rep= 2,
#                                        r_repulsion=4 , d_desired=5)
#         z_cmd = self.get_avg_altitude()
#         for t in range(500):
#             self.compute_velocity(self.k_sep, self.k_coh, False)
#             self.move_UAVs(z_cmd)
    
# def main():
#     swarm = velocityComputation()
#     swarm.spread
    
# if __name__ == "__main__":
#     main()
    
    
               
            


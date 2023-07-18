import airsim
import numpy as np
from numba import jit

class VelocityComputation():
    def __init__(self):
        # Build a connection with AirSim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        # Define the origin position of the swarm
        self.origin = [[0, 0], [2, 0], [4, 0], 
                       [0, -3], [2, -2], [4, -3], 
                       [0, 3], [2, 2], [4, 3]]
        
        self.origin_x = [0, 2, 4, 0, 2, 4, 0, 2, 4]
        self.origin_y = [0, 0, 0, -3, -2, -3, 3, 2, 3]
        self.origin_z = [] # most action will be in 2D plane
        
        self.num_uavs = len(self.origin) # can be selected from 1 to 9
          
    def set_parameters(self, v_max=0, r_max=20, k_sep=0, k_coh=0, 
                       k_mig=1, k_rep=0, r_repulsion=0, d_desired=0):
        self.v_max = v_max
        self.r_max = r_max
        self.k_sep = k_sep
        self.k_coh = k_coh
        self.k_mig = k_mig
        self.r_repulsion = r_repulsion
        self.d_desired = d_desired
        self.k_rep = k_rep
        
        self.v_cmd = np.zeros([2, self.num_uavs])
        self.v_rep = np.zeros([2, 1])
        self.v_coh = np.zeros([2, 1])
        self.v_sep = np.zeros([2, 1])
        self.z_cmd = self.get_avg_altitude()
        self.pos_mig = np.array([[0], [0]])
        
    def get_UAV_pos(self, vehicle_name="SimpleFlight"):
        state = self.client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
        x = state.position.x_val
        y = state.position.y_val
        i = int(vehicle_name[3])
        x += self.origin_x[i - 1]
        y += self.origin_y[i - 1]
        pos = np.array([[x], [y]]) # Return a 2D array
        return pos

    def get_avg_altitude(self):
        # get current z position of all UAVs
        swarm_altitude = [
            self.client.getMultirotorState(vehicle_name="UAV" + str(i + 1))
            .kinematics_estimated.position.z_val for i in range(self.num_uavs)
        ]
        z_cmd = np.mean(swarm_altitude)
        
        return z_cmd   
    
    def get_swarm_center(self):
        # get center position of all UAVs
        temp_pos = []
        for i in range(self.num_uavs):
            name_i = "UAV"+str(i+1)
            temp_pos.append(self.get_UAV_pos(vehicle_name=name_i))
            
        return np.mean(temp_pos, axis=0)
    
    def compute_separation_force(self, pos_i, pos_j):
        r_ij = pos_j - pos_i
        distance = np.linalg.norm(r_ij)
        if distance != 0:
            return -self.k_sep * r_ij / distance
        else:
            return np.zeros(2)
    
    def compute_cohesion_force(self, pos_i, pos_j):
        r_ij = pos_j - pos_i
        return self.k_coh * r_ij
        
    def compute_repulsion_force(self, pos_i, pos_j, repulsion_distance, safe_distance_uav, K_rep):
        r_ij = pos_j - pos_i
        distance = np.linalg.norm(r_ij)

        if distance < repulsion_distance:
            repulsion_vector = -K_rep * r_ij / distance
            if distance < safe_distance_uav:
                repulsion_vector *= 2
        else:
            repulsion_vector = np.zeros([2, 1])  # No repulsion force if distance >= repulsion_distance

        return repulsion_vector

    def compute_forces(self, pos_i, pos_j, rep_dis, safe_dis, add_rep):
        # Calculate the norm
        norm = np.linalg.norm(pos_j - pos_i)
        # Initialize forces
        v_sep = v_coh = v_rep = 0

        if norm < self.r_max:
            v_sep = self.compute_separation_force(pos_i, pos_j)
            v_coh = self.compute_cohesion_force(pos_i, pos_j)
            if add_rep:
                v_rep = self.compute_repulsion_force(pos_i, pos_j, rep_dis, safe_dis, self.k_rep)
                
        return v_sep, v_coh, v_rep
     
    def compute_velocity(self, rep_dis, safe_dis, add_rep):
        # Loop through i-th UAV
        for i in range(self.num_uavs):
            name_i = "UAV" + str(i + 1)
            pos_i = self.get_UAV_pos(vehicle_name=name_i)
             # vector from UAV to migration point
            r_mig = self.pos_mig - pos_i
            N_i = 0
            
            if np.linalg.norm(r_mig) != 0:
                v_mig = self.k_mig * r_mig / np.linalg.norm(r_mig)
            else:
                v_mig = np.zeros([2, 1])
            
            # Loop through j-th UAV
            for j in range(self.num_uavs):
                if j != i:
                    N_i += 1
                    name_j = "UAV" + str(j + 1)
                    pos_j = self.get_UAV_pos(vehicle_name=name_j)
                    
                    if np.linalg.norm(pos_j - pos_i) < self.r_max:
                        v_sep, v_coh, v_rep = self.compute_forces(pos_i, pos_j, rep_dis, safe_dis, add_rep)
                        self.v_sep += v_sep
                        self.v_coh += v_coh
                        self.v_rep += v_rep
                        
            self.v_sep /= N_i
            self.v_coh /= N_i
            self.v_rep /= N_i
            
            # print("v_sep shape:", self.v_sep.shape)
            # print("v_coh shape:", self.v_coh.shape)
            # print("v_rep shape:", self.v_rep.shape)
            # print("v_mig shape:", v_mig.shape)
                        
            self.v_cmd[:, i:i + 1] = self.v_sep + self.v_coh + self.v_rep + v_mig
            
            # # limit the velocity
            # if self.v_cmd[0, i] > self.v_max:
            #     self.v_cmd[0, i] = self.v_max
    
    def move_UAVs(self, z_cmd):
        for i in range(self.num_uavs):
            name_i = "UAV" + str(i + 1)
            self.client.moveByVelocityZAsync(self.v_cmd[0, i], self.v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i)

    # define the generator function for the formation points
    def point_generator(self, type, spacing):
        group_center = np.array([[0], [0]]) #self.get_swarm_center()
        

        if type == 'circle':
            formation_angle_offset = 2 * np.pi / self.num_uavs
            for i in range(self.num_uavs):
                formation_angle = formation_angle_offset * i
                yield group_center + spacing * np.array([[np.cos(formation_angle)], [np.sin(formation_angle)]])
                
        elif type == 'line':
            for i in range(self.num_uavs):
                yield np.array([[i * spacing], [group_center[1, 0]]])
                     
        elif type == 'diagonal':
            for i in range(self.num_uavs):
                row = col = i  # For a diagonal, row index equals column index
                yield np.array([[group_center[0, 0] + (col * spacing)], 
                                [group_center[1, 0] + row * spacing]])
                         
        elif type == 'V':
            for i in range(self.num_uavs):
                yield group_center + np.array([[abs((i - self.num_uavs // 2)) * spacing], 
                                            [(i - self.num_uavs // 2) * spacing]])
        else:
            raise ValueError(f'Unknown formation type: {type}')

    #Define the calculate_velocity function
    def calculate_formation_velocity(self, rep_dis, safe_dis, formation_point_gen):
        v_sep, v_rep, v_coh, N_i = 0, 0, 0, 0
        for i, formation_point in enumerate(formation_point_gen):
            # Define the name and position of the current UAV
            name_i = "UAV" + str(i + 1)
            pos_i = self.get_UAV_pos(vehicle_name=name_i)

            # Compute the desired velocity for the current UAV
            v_mig = self.k_mig * (formation_point - pos_i)
            N_i += 1

            # Compute the repulsion vector if the other UAVs are within the repulsion distance
            for j in range(self.num_uavs):
                if j != i:
                    # Define the name and position of the other UAV
                    name_j = "UAV" + str(j + 1)
                    pos_j = self.get_UAV_pos(vehicle_name=name_j)
                    dist_ij = np.linalg.norm(pos_j - pos_i)
                    
                    if dist_ij < self.r_max:
                        # Compute the repulsion force
                        repulsion_force = self.compute_repulsion_force(pos_i, pos_j, rep_dis, safe_dis, self.k_rep)
                        # Add repulsion force to the repulsion velocit
                        v_rep += repulsion_force
                        v_sep += self.compute_separation_force(pos_i, pos_j)
                        v_coh += self.compute_cohesion_force(pos_i, pos_j)
                           

            v_rep /= N_i
            v_sep /= N_i
            v_coh /= N_i
            # Calculate the final desired velocity
            v_desired = v_mig + v_rep + v_sep + v_coh
        
            # Limit the velocity to the maximum allowed speed
            v_desired_norm = np.linalg.norm(v_desired)
            if v_desired_norm > self.v_max:
                v_desired = self.v_max * v_desired / v_desired_norm

            self.v_cmd[:, i:i + 1] = v_desired
    
    # define form functions
    def form_circle(self, rep_dis, safe_dis, spacing=15):
        formation_points = self.point_generator('circle', spacing)
        self.calculate_formation_velocity(rep_dis, safe_dis, formation_points)

    def form_line(self, rep_dis, safe_dis, spacing=12):
        formation_points = self.point_generator('line', spacing)
        self.calculate_formation_velocity(rep_dis, safe_dis, formation_points)

    def form_V(self, rep_dis, safe_dis, spacing=12):
        formation_points = self.point_generator('V', spacing)
        self.calculate_formation_velocity(rep_dis, safe_dis, formation_points)

    def form_diagonal(self, rep_dis, safe_dis, spacing=10):
        formation_points = self.point_generator('diagonal', spacing)
        self.calculate_formation_velocity(rep_dis, safe_dis, formation_points)
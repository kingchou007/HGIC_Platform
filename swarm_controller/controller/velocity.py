import airsim
import time
import numpy as np

class velocityComputation:
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
        
    def set_parameters(self, v_max, r_max, k_sep, k_coh, k_mig, k_rep, r_repulsion, d_desired):
        self.v_max = v_max
        self.r_max = r_max
        self.k_sep = k_sep
        self.k_coh = k_coh
        self.k_mig = k_mig
        self.r_repulsion = r_repulsion
        self.d_desired = d_desired
        self.k_rep = k_rep
        
    def get_UAV_pos(self, vehicle_name="SimpleFlight"):
        state = self.client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
        x = state.position.x_val
        y = state.position.y_val
        i = int(vehicle_name[3])
        x += self.origin_x[i - 1]
        y += self.origin_y[i - 1]
        pos = np.array([[x], [y]]) # Return a 2D array
        return pos

    def take_off(self):
        for i in range(self.num_uavs):
            name_i = "UAV" + str(i + 1)
            self.client.enableApiControl(True, name_i)
            self.client.armDisarm(True, name_i)
            if i != self.num_uavs - 1:
                self.client.takeoffAsync(vehicle_name=name_i)
            else:
                self.client.takeoffAsync(vehicle_name=name_i).join()
        
        time.sleep(0.5)
        
        # fly to same altitude
        for i in range(self.num_uavs):
            name = "UAV" + str(i + 1)
            if i != self.num_uavs - 1:
                self.client.moveToZAsync(-3, 1, vehicle_name=name)
            else:
                self.client.moveToZAsync(-3, 1, vehicle_name=name).join()
                
    def land(self):
        for i in range(9):
            name_i = "UAV"+str(i+1)
            if i != self.num_uavs - 1: 
                self.client.landAsync(vehicle_name=name_i)
            else:
                self.client.landAsync(vehicle_name=name_i).join()
            
    def get_avg_altitude(self):
        # get current z position of all UAVs
        swam_altitude = [self.client.getMultirotorState(vehicle_name="UAV" + str(i + 1)).kinematics_estimated.position.z_val for i in range(self.num_uavs)]
        z_cmd = np.mean(swam_altitude)
        return z_cmd
    
    def compute_velocity_command(self):
        v_cmd = np.zeros([2, self.num_uavs])
        z_cmd = self.get_avg_altitude()

        for i in range(self.num_uavs):
            name_i = "UAV"+str(i+1)
            pos_i = self.get_UAV_pos(vehicle_name=name_i)
            pos_mig = pos_i + self.shift
            r_mig = pos_mig - pos_i
            v_mig = self.k_mig * r_mig / np.linalg.norm(r_mig)
            v_sep = np.zeros([2, 1])
            v_coh = np.zeros([2, 1]) 
            N_i = 0

            for j in range(self.num_uavs):
                if j != i:
                    N_i += 1
                    name_j = "UAV"+str(j+1)
                    pos_j = self.get_UAV_pos(vehicle_name=name_j)
                    if np.linalg.norm(pos_j - pos_i) < self.r_max:
                        r_ij = pos_j - pos_i
                        v_sep += -self.k_sep * r_ij / np.linalg.norm(r_ij)
                        v_coh += self.k_coh * r_ij 

            if N_i > 0:
                v_sep = v_sep / N_i
                v_coh = v_coh / N_i
                
            v_cmd[:, i] = (v_sep + v_coh + v_mig).flatten()
            if v_cmd[0, i] > self.v_max:
                v_cmd[0, i] = self.v_max

        return v_cmd, z_cmd

    
    def get_vehicle_name(self, i):
        return "UAV" + str(i + 1)
    
    def execute_for_all_uavs(self, func):
        for i in range(self.num_uavs):
            name_i = self.get_vehicle_name(i)
            func(name_i)
    
    # used for left, right, forward, backward
    def move_in_direction(self, shift, t=100):
        self.shift = shift
        for _ in range(t):
            v_cmd, z_cmd = self.compute_velocity_command()
            for i in range(self.num_uavs):
                name_i = self.get_vehicle_name(i)
                self.client.moveByVelocityZAsync(v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i)

    # Compute Merge and spread velocity
    def get_swarm_center(self):
        # get current position of all UAVs
        temp_pos = []
        for i in range(self.num_uavs):
            name_i = "UAV"+str(i+1)
            temp_pos.append(self.get_UAV_pos(vehicle_name=name_i))
            
        return np.mean(temp_pos, axis=0)
            
    def compute_formation_velocities(self):
        v_cmd = np.zeros([2, self.num_uavs])
        z_cmd = self.get_avg_altitude()
        
        # find the center of swarm
        pos_mig = self.get_swarm_center()

        for i in range(self.num_uavs):
            name_i = "UAV" + str(i + 1)
            pos_i = self.get_UAV_pos(vehicle_name=name_i)
            r_mig = pos_mig - pos_i   # 计算从当前无人机的位置到蜂群中心的向量
            
            # 其实逻辑很简单，就是增加了蜂群移动的目标点，然后计算迁移速度，这是使无人机向蜂群中心移动的速度
            v_mig = self.k_mig * r_mig / np.linalg.norm(r_mig) #  计算迁移速度，这是使无人机向蜂群中心移动的速度
            v_sep = np.zeros([2, 1])
            v_coh = np.zeros([2, 1])
            v_rep = np.zeros([2, 1])
            N_i = 0
            
            for j in range(self.num_uavs): # 计算排斥速度，这是使无人机避免与其他无人机碰撞的速度
                if j != i:
                    N_i += 1
                    name_j = "UAV" + str(j + 1)
                    pos_j = self.get_UAV_pos(vehicle_name=name_j)
                    r_ij = pos_j - pos_i
                    dist = np.linalg.norm(r_ij)
                    if dist < self.r_max:  #  检查距离是否小于最大感知距离。
                        if dist < self.r_repulsion: #  如果距离小于排斥半径，则计算排斥速度。
                            v_rep += - self.k_rep * (self.r_repulsion - dist) * r_ij / dist
                        elif dist < self.d_desired: # 如果距离小于期望距离，则计算排斥速度。
                            v_sep += -self.k_sep * (self.d_desired - dist) * r_ij / dist
                        else: # 如果距离大于期望距离，则计算聚集速度。
                            v_coh += self.k_coh * (dist - self.d_desired) * r_ij / dist
            v_sep = v_sep / N_i 
            v_coh = v_coh / N_i  
            v_cmd[:, i] = (v_sep + v_coh + v_rep + v_mig ).flatten() 
        
        return v_cmd, z_cmd
    
    


    
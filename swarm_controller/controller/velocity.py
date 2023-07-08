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
        
        self.num_uavs = len(self.origin)
      
        # Navigation speed
        self.v_max = 10    # maximum velocity
        self.r_max = 5   # radius of the circle
        self.k_sep = 2    # separation control coefficient
        self.k_coh = 0.2  # cohesion control coefficient
        self.k_mig = 5    # migration control coefficient

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
    
    def compute_velocity_command(self, shift):
        v_cmd = np.zeros([2, self.num_uavs])
        z_cmd = self.get_avg_altitude()

        for i in range(self.num_uavs):
            name_i = "UAV"+str(i+1)
            pos_i = self.get_UAV_pos(vehicle_name=name_i)
            pos_mig = pos_i + shift
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

        return v_cmd, z_cmd
    
    def get_vehicle_name(self, i):
        return "UAV" + str(i + 1)
    
    def execute_for_all_uavs(self, func):
        for i in range(self.num_uavs):
            name_i = self.get_vehicle_name(i)
            func(name_i)
    
    # used for left, right, forward, backward
    def move_in_direction(self, shift, t=100):
        for _ in range(t):
            v_cmd, z_cmd = self.compute_velocity_command(shift)
            for i in range(self.num_uavs):
                name_i = self.get_vehicle_name(i)
                self.client.moveByVelocityZAsync(v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i)

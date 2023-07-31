# from velocity import VelocityComputation

class Configuration:
    def __init__(self):
        self.max = 9
        self.num_uavs = self.max  # Default number of UAVs in the swarm
        
    def split_three(self):
        self.num_uavs = self.num_uavs // 3
    
    def add(self):
        if self.num_uavs < max:
            self.num_uavs += 1
        
    def delete(self):
        if self.num_uavs > 0:
            self.num_uavs -= 1
            
    def split(slef, num_groups):
        # 3 groups of 3 UAVs
        slef.num_uavs = slef.num_uavs // num_groups
        
    def adjust_max_velocity(self, v_max):
        # get the v_max in the velocity computation
        # adjust the v_max
        v_max += 1
        return v_max
    
    def selet_all(self):
        self.num_uavs = max
        return self.num_uavs
        
    
    

        
        
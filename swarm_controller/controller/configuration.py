class Configuration(object):
    def __init__(self):
        # Define the origin position of the swarm
        self.origin = [[0, 0], [2, 0], [4, 0], 
                       [0, -3], [2, -2], [4, -3], 
                       [0, 3], [2, 2], [4, 3]]
        
        self.origin_x = [0, 2, 4, 0, 2, 4, 0, 2, 4]
        self.origin_y = [0, 0, 0, -3, -2, -3, 3, 2, 3]
        self.origin_z = [] # we only consider 2-D plane
        
        self.num_uavs = len(self.origin) # can be selected from 1 to 9
        # self.num_uavs = 5
        
    def split_three(self):
        self.num_uavs = self.num_uavs // 3
    
    def add(self):
        if self.num_uavs < len(self.origin):
            self.num_uavs += 1
            print(self.num_uavs)
        else:
            print("The maximum number of UAVs is reached!")
        
    def delete(self):
        if self.num_uavs > 0:
            self.num_uavs -= 1
        else:
            print("No UAVs in the swarm!")
            
    def split(slef, num_groups):
        # 3 groups of 3 UAVs
        slef.num_uavs = slef.num_uavs // num_groups    
        
    def increase_max_velocity(self):
        self.adjust_v_max += 1
        return self.adjust_v_max
    
    def decrease_max_velocity(self):
        self.adjust_v_max -= 1
        return self.adjust_v_max
    
    def select_all(self):
        self.num_uavs = len(self.origin)
        return self.num_uavs
class Configuration(object):
    def __init__(self):
        # List of origin positions for each UAV in the swarm.
        self.origin = [
            [0, 0],
            [2, 0],
            [4, 0],
            [0, -3],
            [2, -2],
            [4, -3],
            [0, 3],
            [2, 2],
            [4, 3],
        ]

        # Separate lists for x and y coordinates of the origins.
        self.origin_x = [0, 2, 4, 0, 2, 4, 0, 2, 4]
        self.origin_y = [0, 0, 0, -3, -2, -3, 3, 2, 3]

        # Placeholder for z-coordinates, currently unused as the scenario is 2D.
        self.origin_z = []

        # Initial count of UAVs based on the number of origins defined.
        self.num_uavs = len(self.origin)

    def split_three(self):
        """Divide the number of UAVs into three equal groups."""
        self.num_uavs = self.num_uavs // 3

    def add(self):
        """Increment the count of UAVs by one if not at max capacity."""
        if self.num_uavs < len(self.origin):
            self.num_uavs += 1
            print(self.num_uavs)
        else:
            print("The maximum number of UAVs is reached!")

    def delete(self):
        """Decrement the count of UAVs by one if there are any present."""
        if self.num_uavs > 0:
            self.num_uavs -= 1
        else:
            print("No UAVs in the swarm!")

    def split(self, num_groups):
        """Divide the number of UAVs into specified number of groups."""
        self.num_uavs = self.num_uavs // num_groups

    def increase_max_velocity(self):
        """Increase the maximum velocity of the UAVs by one."""
        self.adjust_v_max += 1
        return self.adjust_v_max

    def decrease_max_velocity(self):
        """Decrease the maximum velocity of the UAVs by one."""
        self.adjust_v_max -= 1
        return self.adjust_v_max

    def select_all(self):
        """Select all available UAVs in the origin list."""
        self.num_uavs = len(self.origin)
        return self.num_uavs

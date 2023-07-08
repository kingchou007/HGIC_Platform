from velocity import velocityComputation
import numpy as np


class Navigation:
    def __init__(self):
        self.controller = velocityComputation()
    
    # Navigation methods   
    def forward(self, desired_distance = 10):
        self.controller.move_in_direction(np.array([[desired_distance], [0]]))

    def backward(self, desired_distance = 10):
        self.controller.move_in_direction(np.array([[-desired_distance], [0]]))

    def left(self, desired_distance = 10):
        self.controller.move_in_direction(np.array([[0], [-desired_distance]]))

    def right(self, desired_distance = 10):
        self.controller.move_in_direction(np.array([[0], [desired_distance]]))
        
    def up(self, desired_height = 30, speed = 10):
        z_cmd = self.controller.get_avg_altitude() - desired_height
        self.controller.execute_for_all_uavs(lambda name: self.controller.client.moveToZAsync(z_cmd, speed, vehicle_name=name))

    def down(self, desired_height = 20, speed = 10):
        z_cmd = self.controller.get_avg_altitude() + desired_height
        self.controller.execute_for_all_uavs(lambda name: self.controller.client.moveToZAsync(z_cmd, speed, vehicle_name=name))
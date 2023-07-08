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
        
        # Navigation methods
    # def up(self, desired_height = 30, speed = 10):
    #     # update z_cmd to be the average of all UAVs' z position
    #     z_cmd = self.controller.get_avg_altitude() - desired_height
        
    #     for i in range(self.num_uavs):
    #         name_i = "UAV" + str(i + 1)
    #         if i != self.num_uavs - 1:
    #             self.client.moveToZAsync(z_cmd, speed, vehicle_name=name_i)
    #         else:
    #             self.client.moveToZAsync(z_cmd, speed, vehicle_name=name_i).join()  
                
    # def down(self, desired_height = 20, speed = 10):
    #     z_cmd = self.controller.get_avg_altitude() + desired_height
        
    #     for i in range(self.num_uavs):
    #         name_i = "UAV" + str(i + 1)
    #         if i != self.num_uavs - 1:
    #             self.client.moveToZAsync(z_cmd, speed, vehicle_name=name_i)
    #         else:
    #             self.client.moveToZAsync(z_cmd, speed, vehicle_name=name_i).join()
    def up(self, desired_height = 30, speed = 10):
        z_cmd = self.controller.get_avg_altitude() - desired_height
        self.controller.execute_for_all_uavs(lambda name: self.controller.client.moveToZAsync(z_cmd, speed, vehicle_name=name))

    def down(self, desired_height = 20, speed = 10):
        z_cmd = self.controller.get_avg_altitude() + desired_height
        self.controller.execute_for_all_uavs(lambda name: self.controller.client.moveToZAsync(z_cmd, speed, vehicle_name=name))

                
def main():
    nav = Navigation()
    nav.up()
    # nav.down()

                    

if __name__ == "__main__":
    main()
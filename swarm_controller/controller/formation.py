from velocity import velocityComputation
import numpy as np

class FormationController(object):
    def __init__(self):
        self.controller = velocityComputation()

    def merge(self):
        self.controller.set_parameters(v_max=2, r_max=20, k_sep=20, k_coh=0.5, k_mig=1, k_rep=20,
                                       r_repulsion=2, d_desired=5)
        
        for _ in range(300):
            v_cmd, z_cmd = self.controller.compute_formation_velocities()
            for i in range(9):
                name_i = self.controller.get_vehicle_name(i)
                self.controller.client.moveByVelocityZAsync(v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i)

    
    def spread():
        pass
    
    def circle():
        pass
    
    def grid():
        pass
    

fc = FormationController()
fc.merge()
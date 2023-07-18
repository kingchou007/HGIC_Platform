from velocity import VelocityComputation
import numpy as np

class FormationController(object):
    def __init__(self):
        self.control = VelocityComputation()
        self.pos_mig = self.control.get_swarm_center() 
        self.z_cmd = self.control.get_avg_altitude()
        self.compute_velocity = self.control.compute_velocity
        self.move_UAVs = self.control.move_UAVs

    def merge(self):
        self.control.set_parameters(v_max=3, r_max=20, k_sep=1.7, k_coh=0.3, k_mig=1, k_rep=7,
                                      r_repulsion=8)
        self.control.pos_mig = self.control.get_swarm_center()
        self.run_loop(True, 8, 5, 500)
        
    def spread(self):
        self.control.set_parameters(v_max=15, r_max=20, k_sep=35, k_coh=1.3, k_mig=1)
        self.control.pos_mig = self.control.get_swarm_center()
        self.run_loop(False, 0, 0, 300)
        
    def circle(self):
        self.control.set_parameters(v_max=7, r_max=30, k_mig=1.5, k_rep=20, k_sep=0.7, k_coh=0.05)
        self.control.pos_mig = self.control.get_swarm_center()
        for _ in range(800):
            self.control.form_circle(15, 8)
            self.move_UAVs(self.z_cmd)
    
    def line(self):
        self.control.set_parameters(v_max=3, r_max=30, k_mig=1.5, k_rep=20, k_sep=1, k_coh=0.03)
        self.control.pos_mig = self.control.get_swarm_center()
        for _ in range(800):
            self.control.form_line(10, 4)
            self.move_UAVs(self.z_cmd)
            
    def V_formation(self):
        self.control.set_parameters(v_max=5, r_max=20, k_mig=1.5, k_rep=20, k_sep=1, k_coh=0.05)
        for _ in range(800):
            self.control.form_V(10, 8)
            self.move_UAVs(self.z_cmd)
    
    def diagonal(self):
        self.control.set_parameters(v_max=5, r_max=20, k_mig=1.5, k_rep=15, k_sep=0.5, k_coh=0.05)
        for _ in range(800):
            self.control.form_diagonal(10, 5)
            self.move_UAVs(self.z_cmd)
                    
    def run_loop(self, add_rep, rep_dis, safe_dis, t=0):
        for _ in range(t):
            self.compute_velocity(rep_dis, safe_dis, add_rep) 
            self.move_UAVs(self.z_cmd)
            
def main():
    swarm = FormationController()
    
    swarm.circle()
    swarm.line()
    swarm.V_formation()
    swarm.diagonal()
    swarm.spread()
    swarm.merge()

if __name__ == "__main__":
    main()

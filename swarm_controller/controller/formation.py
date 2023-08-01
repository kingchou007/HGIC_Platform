from .velocity import VelocityComputation
from .configuration import Configuration

class FormationController(object):
    def __init__(self):
        self.control = VelocityComputation()
        self.config = Configuration()
        self.pos_mig = self.control.get_swarm_center() 
        self.z_cmd = self.control.get_avg_altitude()
        self.compute_velocity = self.control.compute_velocity
        self.move_UAVs = self.control.move_UAVs
        self.adjust_v_max = self.control.v_max
        
    def change_velocity(self):
        self.adjust_v_max += self.config.increase_max_velocity()
        print("The maximum velocity is set to: ", self.adjust_v_max)

    def merge(self):
        self.control.set_parameters(v_max=3, r_max=20, k_sep=1.7, k_coh=0.5, k_mig=1, k_rep=9, r_repulsion=8)
        self.control.pos_mig = self.control.get_swarm_center()
        self.run_loop(True, 9, 5, 800)
        
    def spread(self):
        self.control.set_parameters(v_max=15, r_max=20, k_sep=35, k_coh=1.3, k_mig=1)
        self.control.pos_mig = self.control.get_swarm_center()
        self.run_loop(False, 0, 0, 300)
        
    def circle(self):
        self.control.set_parameters(v_max=16, r_max=30, k_mig=1, k_rep=25, k_sep=0.3, k_coh=0.02)
        self.control.pos_mig = self.control.get_swarm_center()
        for _ in range(500):
            self.control.form_circle(7, 7)
            self.move_UAVs(self.z_cmd)
            self.degbug_info()

    def line(self):
        self.control.set_parameters(v_max=5, r_max=22, k_mig=1, k_rep=25, k_sep=0.4, k_coh=0.01)
        self.control.pos_mig = self.control.get_swarm_center()
        for _ in range(500):
            self.control.form_line(13, 8)
            self.move_UAVs(self.z_cmd)
            self.degbug_info()
            
    def V_formation(self):
        self.control.set_parameters(v_max=6, r_max=25, k_mig=1, k_rep=25, k_sep=0.3, k_coh=0.02)
        for _ in range(500):
            self.control.form_V(8, 5)
            self.move_UAVs(self.z_cmd)
            self.degbug_info()
    
    def diagonal(self):
        self.control.set_parameters(v_max=12, r_max=25, k_mig=1, k_rep=25, k_sep=0.3, k_coh=0.02)
        for _ in range(500):
            self.control.form_diagonal(13, 8)
            self.move_UAVs(self.z_cmd)
            self.degbug_info()
                    
    def run_loop(self, add_rep, rep_dis, safe_dis, t=0):
        for _ in range(t):
            self.compute_velocity(rep_dis, safe_dis, add_rep) 
            self.move_UAVs(self.z_cmd)
    
    def degbug_info(self):
        info = self.control.compute_density()
        print("Density: ", info)

# def main():
#     swarm = FormationController()
    
#     # swarm.spread()
    
    
#     # swarm.line()
#     # swarm.V_formation()
    
#     # swarm.circle()
#     # swarm.diagonal()
#     # swarm.merge()
#     # swarm.change_velocity()

# if __name__ == "__main__":
#     main()

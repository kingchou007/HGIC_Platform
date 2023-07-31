from velocity import VelocityComputation
import numpy as np

class TaskControl(object):
    def __init__(self):
        self.control = VelocityComputation()

    def circle_search(self):
        self.control.set_parameters(v_max=10, r_max=20, k_mig=1, k_rep=25, k_sep=0.3, k_coh=0.02)
        self.control.circle_move_circle()
        
    def circle_v_search(self):
        self.control.set_parameters(v_max=5, r_max=30, k_mig=0.4, k_rep=30, k_sep=1, k_coh=0)
        self.control.V_move_circle()
        
    def line_search(self):
        self.control.set_parameters(v_max=10, r_max=20, k_mig=0.5, k_rep=15, k_sep=0.5, k_coh=1)
        self.control.line_search()
        
    
    def cover(self):
        self.control.set_parameters(v_max=10, r_max=30, k_mig=3, k_rep=25, k_sep=2, k_coh=0)
        self.control.space_ccupation()

def main():
    task = TaskControl()
    #task.circle_search()
    #task.circle_v_search()
    task.cover()
    # task.line_search()
    
if __name__ == '__main__':
    main()
        
        
        
import timeit
import airsim
from velocity import VelocityComputation


class FormationController(object):
    def __init__(self):
        # Initialize the velocity computation object
        self.control = VelocityComputation()
        # Get the center of the swarm
        self.pos_mig = self.control.get_swarm_center()
        # Get the average altitude of the swarm
        self.z_cmd = self.control.get_avg_altitude()
        # Reference to the velocity computation methods for easier access
        self.compute_velocity = self.control.compute_velocity
        self.move_UAVs = self.control.move_UAVs

    def change_velocity(self):
        """Change the maximum velocity by increasing it."""
        self.adjust_v_max += self.config.increase_max_velocity()
        print("The maximum velocity is set to: ", self.adjust_v_max)

    def merge(self):
        """Merge the formation."""
        self.control.set_parameters(
            v_max=3, r_max=20, k_sep=1.7, k_coh=0.5, k_mig=1, k_rep=9, r_repulsion=8)
        self.control.pos_mig = self.control.get_swarm_center()
        self.run_loop(True, 8, 5, 500)

    def spread(self):
        """Spread the formation."""
        self.control.set_parameters(
            v_max=15, r_max=20, k_sep=35, k_coh=1.3, k_mig=1)
        self.control.pos_mig = self.control.get_swarm_center()
        self.run_loop(False, 0, 0, 300)

    def circle(self):
        """Make the drones form a circle."""
        self.control.set_parameters(
            v_max=20, r_max=25, k_mig=2, k_rep=25, k_sep=15, k_coh=0.1)
        self.control.pos_mig = self.control.get_swarm_center()
        self.control.form_circle(10, 10)

    def line(self):
        """Make the drones form a line."""
        self.control.set_parameters(
            v_max=20, r_max=25, k_mig=2, k_rep=25, k_sep=15, k_coh=0.1)
        self.control.pos_mig = self.control.get_swarm_center()
        self.control.form_line(13, 7)

    def V_formation(self):
        """Make the drones form a V-formation."""
        self.control.set_parameters(
            v_max=20, r_max=25, k_mig=2, k_rep=25, k_sep=15, k_coh=0.1)
        self.control.form_V(10, 7)

    def diagonal(self):
        """Make the drones form a diagonal."""
        self.control.set_parameters(
            v_max=12, r_max=25, k_mig=1, k_rep=25, k_sep=0.3, k_coh=0.02)
        for _ in range(600):
            self.control.form_diagonal(13, 8)
            self.move_UAVs(self.z_cmd)
            self.degbug_info()

    def run_loop(self, add_rep, rep_dis, safe_dis, t=0):
        """Main loop to compute velocity and move the drones."""
        for _ in range(t):
            self.compute_velocity(rep_dis, safe_dis, add_rep)
            self.move_UAVs(self.z_cmd)

    def degbug_info(self):
        """Print density information for debugging."""
        info = self.control.compute_density()
        print("Density: ", info)

# Uncomment below lines to test and time consuming specific formations
# def time_function(func):
#     """Time the execution of a function."""
#     start_time = timeit.default_timer()
#     func()
#     elapsed_time = timeit.default_timer() - start_time
#     print(f"{func.__name__} took {elapsed_time:.5f} seconds.")

#!/usr/bin/python
# -*- coding: utf-8 -*-
from swarm_controller.velocity1 import VelocityComputation
import timeit


class TaskControl(object):
    def __init__(self):
        # Initialize the velocity computation object for task control

        self.control = VelocityComputation()

    def circle_search(self):
        """Configure parameters and perform circle search using drones."""

        self.control.set_parameters(
            v_max=10,
            r_max=20,
            k_mig=0.5,
            k_rep=25,
            k_sep=0.3,
            k_coh=0.02,
        )
        self.control.circle_move_circle()

    def circle_v_search(self):
        """Configure parameters and perform a V-patterned circle search using drones."""

        self.control.set_parameters(
            v_max=5,
            r_max=30,
            k_mig=0.4,
            k_rep=30,
            k_sep=1,
            k_coh=0,
        )
        self.control.V_move_circle()

    def line_search(self):
        """Configure parameters and perform a linear search using drones."""

        self.control.set_parameters(
            v_max=20,
            r_max=25,
            k_mig=0.08,
            k_rep=25,
            k_sep=1,
            k_coh=0.1,
        )
        self.control.line_search()

    def cover(self):
        """Configure parameters and make drones occupy space effectively."""

        self.control.set_parameters(
            v_max=10,
            r_max=30,
            k_mig=3,
            k_rep=25,
            k_sep=2,
            k_coh=0,
        )
        self.control.space_ccupation()


# Uncomment below to test and time specific tasks

# def time_function(func):
#     """Time the execution of a function."""
#     start_time = timeit.default_timer()
#     func()
#     elapsed_time = timeit.default_timer() - start_time
#     print(f"{func.__name__} took {elapsed_time:.5f} seconds.")

import socket
import collections
import threading
import time
import numpy as np
import keyboard
import sys
import airsim

# Import the functions from the controller package
from configuration import Configuration as config
from task import TaskControl as task

# from controller.navigation import NavigationController as nav
from formation import FormationController
from swarm_controller.velocity1 import VelocityComputation

# from control_api import ControlAPI
import test as SwarmControl


fm = FormationController()

# Define valid commands and map them to corresponding functions
COMMANDS = {
    "take off": SwarmControl.take_off,
    "land": SwarmControl.land,
    "spread": fm.spread,
    "merge": fm.merge,
    "V": fm.V_formation,
    "up": SwarmControl.up,
    "down": SwarmControl.down,
    "forward": SwarmControl.forward,
    "backward": SwarmControl.backward,
    "left": SwarmControl.left,
    "right": SwarmControl.right,
    "chase": SwarmControl.chasing,
    "cover": task.cover,
    "circle search": task.circle_search,
    "v_search": task.circle_v_search,
    "search": task.line_search,
    "circle": fm.circle,
    "v": task.circle_v_search,
    "grid": fm.line,
    "split": SwarmControl.test2,
}


def time_weighted_freqs(command_queue):
    """
    Calculate the frequency of each command in the queue with more recent
    commands being given a higher weight.
    """
    now = time.time()
    command_freqs = collections.defaultdict(int)
    for command, timestamp in command_queue:
        # Weight decreases exponentially with time difference
        weight = np.exp(-(now - timestamp))
        command_freqs[command] += weight
    return command_freqs


def receive_sendback_message(sock, command_queue):
    """
    Function to receive messages over the UDP socket and
    add valid commands to the command queue.
    """
    while True:
        command, addr = sock.recvfrom(1024)
        command = command.decode("utf8")  # Convert bytes to string
        if command in COMMANDS:
            command_queue.append((command, time.time()))
            print(f"{command} is received")

        # # Send a response to the client
        # response = "Command received: " + command
        # #sock.sendto(response.encode(), addr)


def main():
    """
    Main function to set up UDP communication, process commands, and control drones.
    """
    # #get collision information
    # swarm = VelocityComputation()
    # collision_count = 0

    UDP_IP = "127.0.0.1"
    UDP_PORT = 5000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    command_queue = []
    COMMAND_THRESHOLD = 10
    threading.Thread(
        target=receive_sendback_message, args=(sock, command_queue)
    ).start()
    last_command = ""
    # # show velocity_max
    # v_max = 0

    # Main Loop
    while True:
        # status = swarm.get_collision_info()
        # if status == True:
        #     collision_count += 1 # send this back to Inteaction system
        #     print(collision_count)

        if len(command_queue) >= COMMAND_THRESHOLD:
            command_freqs = time_weighted_freqs(command_queue)
            most_common_command = max(command_freqs, key=command_freqs.get)

            print("Most common command:", most_common_command)
            if most_common_command != last_command:
                COMMANDS[most_common_command]()
                last_command = most_common_command
                command_queue.clear()


if __name__ == "__main__":
    main()

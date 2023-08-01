import socket
import collections
import threading
import time
import numpy as np
import keyboard

# Import the functions from the controller package 
from controller.configuration import Configuration as config
from controller.task import TaskControl as task
from controller.navigation import NavigationController as nav
from controller.formation import FormationController as formation


# Define valid commands and map them to corresponding functions
COMMANDS = {
    "take off": nav.take_off,
    "land": nav.land,
    "spread": formation.spread,
    "merge": formation.merge,
    "up": nav.up,
    "down": nav.down,
    "forward": nav.forward,
    "backward": nav.backward,
    "left": nav.left,
    "right": nav.right,
    "chase": task.chasing,
    "cover": task.cover,
    "circle search": task.circle_search,
    "v)search": task.circle_v_search,
    "search": task.line_search,
    "circle": formation.circle,
    "v": formation.circle_v,
    "line": formation.line,
    
}

# Function to calculate time-weighted command frequencies
def time_weighted_freqs(command_queue):
    now = time.time()
    command_freqs = collections.defaultdict(int)
    for command, timestamp in command_queue:
        # Weight decreases exponentially with time difference
        weight = np.exp(-(now - timestamp))
        command_freqs[command] += weight
    return command_freqs

# Build a UDP socket to receive commands from the user side  
def receive_message(sock, command_queue):
    while True:
        command, addr = sock.recvfrom(1024)
        command = command.decode('utf8')  # Convert bytes to string
        if command in COMMANDS:
            command_queue.append((command, time.time()))
            print(f"{command} is received")
        
        # Send a response to the client
        response = "Command received: " + command
        #sock.sendto(response.encode(), addr)


def main():
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    command_queue = []
    COMMAND_THRESHOLD = 20  # Perform command after every 25 received commands

    # Start receiving messages asynchronously
    threading.Thread(target=receive_message, args=(sock, command_queue)).start()

    last_command = ""
    
    while True:
        if len(command_queue) >= COMMAND_THRESHOLD:
            command_freqs = time_weighted_freqs(command_queue)
            most_common_command = max(command_freqs, key=command_freqs.get)
            
            print("Most common command:", most_common_command)
            if most_common_command != last_command: # Avoid sending the same command twice
                COMMANDS[most_common_command]()
                last_command = most_common_command
                command_queue.clear()
                
        if keyboard.is_pressed('q'):
            break


if __name__ == "__main__":
    main()

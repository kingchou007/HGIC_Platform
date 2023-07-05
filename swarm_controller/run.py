import socket
import control_api as SwarmControl
import collections
import threading
import time
import numpy as np
import keyboard

# Define valid commands and map them to corresponding functions
COMMANDS = {
    "start": SwarmControl.take_off,
    "land": SwarmControl.land,
    "spread": SwarmControl.spread,
    "merge": SwarmControl.merge,
    "up": SwarmControl.up,
    "down": SwarmControl.down,
    "forward": SwarmControl.forward,
    "backward": SwarmControl.backward,
    "left": SwarmControl.left,
    "right": SwarmControl.right,
    "take_off": SwarmControl.take_off
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
        command = command.decode()  # Convert bytes to string
        if command in COMMANDS:
            command_queue.append((command, time.time()))
            # print(f"{command} is received")
        
        # Send a response to the client
        # response = "Command received: " + command
        #sock.sendto(response.encode(), addr)


def main():
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    command_queue = []
    COMMAND_THRESHOLD = 25  # Perform command after every 25 received commands

    # Start receiving messages asynchronously
    threading.Thread(target=receive_message, args=(sock, command_queue)).start()

    last_command = ""
    
    while True:
        if keyboard.is_pressed('q'):  # if 'esc' is pressed, break the loop
            print('End now!')
            break
        
        if len(command_queue) >= COMMAND_THRESHOLD:
            command_freqs = time_weighted_freqs(command_queue)
            most_common_command = max(command_freqs, key=command_freqs.get) # type: ignore
            
            print("Most common command:", most_common_command)
            if most_common_command != last_command:
                COMMANDS[most_common_command]()
                last_command = most_common_command
                command_queue.clear()
                
        # time.sleep(5)  # Wait for 1 second before checking the command queue again


if __name__ == "__main__":
    main()

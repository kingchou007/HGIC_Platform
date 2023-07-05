# import socket
# import control_api as SwarmControl
# # import airsim
# import collections
# import keyboard
# import time


# # build a UDP socket to receive commands from the user side
# def receive_Message():
#     UDP_IP = "127.0.0.1"
#     UDP_PORT = 5000
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock.bind((UDP_IP, UDP_PORT))
#     command, _ = sock.recvfrom(1024)
#     command = command.decode()  # Convert bytes to string
#     return command
   

# def main():
#     command_queue = []
#     COMMAND_THRESHOLD = 25  # Perform command after every 10 received commands
#     SwarmControl.take_off()
    
    
#     while True:
#         last_command = ""
#         command = receive_Message()
#         print(command + " is received")
        
#         command_queue.append(command)
#         if len(command_queue) >= COMMAND_THRESHOLD:
#             most_common_command = collections.Counter(command_queue).most_common(1)[0][0]
#             print("Most common command:", most_common_command)
#             if most_common_command == "spread" and last_command != "spread":
#                 last_command = most_common_command
#                 SwarmControl.spread()
#                 command_queue.clear()
#             elif most_common_command == "merge" and last_command != "merge":
#                 last_command = most_common_command
#                 SwarmControl.test2()
#                 command_queue.clear()
#             elif most_common_command == "up" and last_command != "up":
#                 last_command = most_common_command
#                 command_queue.clear()
#                 SwarmControl.up()
#             elif most_common_command == "down" and last_command != "down":
#                 last_command = most_common_command
#                 command_queue.clear()
#                 SwarmControl.down()
#             elif most_common_command == "forward" and last_command != "forward":
#                 last_command = most_common_command
#                 command_queue.clear()
#                 SwarmControl.forward()
#             elif most_common_command == "backwrod" and last_command != "backwrod":
#                 last_command = most_common_command
#                 command_queue.clear()
#                 SwarmControl.backward()
#             elif most_common_command == "left" and last_command != "left":
#                 last_command = most_common_command
#                 command_queue.clear()
#                 SwarmControl.left()
#             elif most_common_command == "right" and last_command != "right":
#                 last_command = most_common_command
#                 command_queue.clear()
#                 SwarmControl.right()
                
                
        

# if __name__ == "__main__":
#     main()


import socket
import control_api as SwarmControl
import collections
import threading
import time

# # Define valid commands and map them to corresponding functions
# COMMANDS = {
#     "spread": SwarmControl.spread,
#     "merge": SwarmControl.merge,
#     "up": SwarmControl.up,
#     "down": SwarmControl.down,
#     "forward": SwarmControl.forward,
#     "backward": SwarmControl.backward,
#     "left": SwarmControl.left,
#     "right": SwarmControl.right,
#     "take_off": SwarmControl.take_off
# }

# # Build a UDP socket to receive commands from the user side  
# def receive_message(sock, command_queue):
#     while True:
#         command, addr = sock.recvfrom(1024)
#         command = command.decode()  # Convert bytes to string
#         if command in COMMANDS:
#             command_queue.append(command)
#             print(f"{command} is received")
        
#         # Send a response to the client
#         response = "Command received: " + command
#         sock.sendto(response.encode(), addr)


# def main():
#     UDP_IP = "127.0.0.1"
#     UDP_PORT = 5000
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock.bind((UDP_IP, UDP_PORT))

#     command_queue = []
#     COMMAND_THRESHOLD = 25  # Perform command after every 25 received commands

#     # Start receiving messages asynchronously
#     threading.Thread(target=receive_message, args=(sock, command_queue)).start()

#     last_command = ""
#     while True:
#         if len(command_queue) >= COMMAND_THRESHOLD:
#             most_common_command = collections.Counter(command_queue).most_common(1)[0][0]
#             print("Most common command:", most_common_command)
#             if most_common_command != last_command:
#                 COMMANDS[most_common_command]()
#                 last_command = most_common_command
#                 command_queue.clear()
#         time.sleep(5)  # Wait for 1 second before checking the command queue again


# if __name__ == "__main__":
#     main()



###################################################################################################
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

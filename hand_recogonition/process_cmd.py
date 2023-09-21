import json
import os
from collections import deque, Counter
from timingdecorator.timeit import timeit
import time

class GestureCommandProcessor:
    def __init__(self):
        # Get the directory that this script is in
        script_dir = os.path.dirname(os.path.realpath(__file__))

        # Construct the path to the command file
        command_file = os.path.join(script_dir, "hand_settings.json")

        with open(command_file, 'r') as f:
            self.command_dict = json.load(f)

        self.modes = [mode for mode in self.command_dict.keys() if mode not in ["Emergency", "Universal"]]
        self.current_mode_index = 0  # default mode index
        self.current_mode = "None"  # Set the current mode to "None" by default
        self.switch_buffer = deque(maxlen=25)
        self.frames_since_last_switch = 0  # Counter for frames since last mode switch

    #@timeit
    def switch_mode(self, hand_sign_id):
        switch_mode_sign_id = 3  # the sign_id for switching modes
        # Add the current hand sign to the buffer
        self.switch_buffer.append(hand_sign_id)

        # Get the most common hand sign in the buffer
        most_common_sign, count = Counter(self.switch_buffer).most_common(1)[0]

        if (
            most_common_sign == switch_mode_sign_id
            and count > 15
            and self.frames_since_last_switch >= 50
        ):
            self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)
            self.current_mode = self.modes[self.current_mode_index]
            print(f"Mode switched to '{self.current_mode}'")
            # Clear the buffer after a successful switch to prevent immediate subsequent switches
            self.switch_buffer.clear()
            self.frames_since_last_switch = 0  # Reset the counter

        self.frames_since_last_switch += 1

    # Execute the corresponding command based on the hand_sign_id and current mode.
    #@timeit
    def execute_command(self, hand_sign_id):
        # Check for emergency commands first since they have the highest priority
        start_time = time.perf_counter()
        emergency_command = self.command_dict.get("Emergency", {}).get(str(hand_sign_id))
        if emergency_command is not None:
            print(f"Executing emergency command: '{emergency_command}'")
            return emergency_command
      

        # Check for universal commands
        universal_command = self.command_dict.get("Universal", {}).get(str(hand_sign_id))
        if universal_command is not None:
            print(f"Executing universal command: '{universal_command}'")
            return universal_command

        current_mode_command = self.command_dict.get(self.current_mode, {}).get(str(hand_sign_id), "None")
        return current_mode_command
    
        elapsed_time = (time.perf_counter() - start_time) * 1000
        print(f"Inference time: {elapsed_time:.5f} ms")  # print

    def get_current_mode(self):
        return self.current_mode

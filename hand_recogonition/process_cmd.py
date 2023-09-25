import json
import os
from collections import deque, Counter
from timingdecorator.timeit import timeit
import time


class GestureCommandProcessor:
    def __init__(self):
        # Determine the directory of the current script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        # Form the path to the hand settings JSON file
        command_file = os.path.join(script_dir, "hand_settings.json")

        # Load the hand gesture-command mappings from the JSON file
        with open(command_file, "r") as f:
            self.command_dict = json.load(f)

        # Extract all the available modes from the command dictionary
        self.modes = [
            mode
            for mode in self.command_dict.keys()
            if mode not in ["Emergency", "Universal"]
        ]
        self.current_mode_index = 0
        self.current_mode = "None"
        # Set up a buffer for switch mode gesture detection
        self.switch_buffer = deque(maxlen=25)
        self.frames_since_last_switch = 0

    def switch_mode(self, hand_sign_id):
        """Switches the mode if the given hand gesture indicates a mode switch."""
        switch_mode_sign_id = 3  # the sign_id for switching modes
        self.switch_buffer.append(hand_sign_id)
        most_common_sign, count = Counter(self.switch_buffer).most_common(1)[0]

        # Check if the buffer contains enough 'switch' gestures and a switch hasn't happened recently
        if (
            most_common_sign == switch_mode_sign_id
            and count > 15
            and self.frames_since_last_switch >= 50
        ):
            # Cycle through the available modes
            self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)
            self.current_mode = self.modes[self.current_mode_index]
            print(f"Mode switched to '{self.current_mode}'")
            self.switch_buffer.clear()  # Reset the buffer
            self.frames_since_last_switch = 0  # Reset the counter

        self.frames_since_last_switch += 1

    @timeit
    def execute_command(self, hand_sign_id):
        """Determines and returns the command associated with the given hand gesture."""
        # Check for emergency commands as they have the highest priority
        emergency_command = self.command_dict.get("Emergency", {}).get(
            str(hand_sign_id)
        )
        if emergency_command is not None:
            print(f"Executing emergency command: '{emergency_command}'")
            return emergency_command

        # Check for universal commands
        universal_command = self.command_dict.get("Universal", {}).get(
            str(hand_sign_id)
        )
        if universal_command is not None:
            print(f"Executing universal command: '{universal_command}'")
            return universal_command

        # Get the command for the current mode
        current_mode_command = self.command_dict.get(self.current_mode, {}).get(
            str(hand_sign_id), "None"
        )
        return current_mode_command

        # The following print statement seems misplaced; it will never execute due to the return statement above
        elapsed_time = (time.perf_counter() - start_time) * 1000
        print(f"Inference time: {elapsed_time:.5f} ms")

    def get_current_mode(self):
        """Returns the current operating mode."""
        return self.current_mode

# command_utils.py

import json

json_filepath = "hand_recognition/utils/commands.json"

def load_commands(json_filepath):
    """
    Load commands from a JSON file.
    """
    with open(json_filepath, 'r') as file:
        commands = json.load(file)
    return commands

def execute_command(json_filepath, mode, hand_sign_id, emergency=False):
    """
    Execute the corresponding command based on the mode and hand_sign_id.

    Parameters:
    json_filepath (str): Path to the JSON file containing command dictionary.
    mode (str): The operation mode ("Navigation", "Formation Control", etc.).
    hand_sign_id (int or str): The ID of the recognized hand sign.
    emergency (bool): If True, execute the corresponding emergency command, regardless of the mode.

    Returns:
    str: The executed command. Returns 'none' if no match found.
    """
    commands = load_commands(json_filepath)
    if emergency:
        command = commands.get("Emergency", {}).get(str(hand_sign_id), "none")
    else:
        command = commands.get("Universal", {}).get(str(hand_sign_id), "none")
        if command == "none":
            command = commands.get(mode, {}).get(str(hand_sign_id), "none")
    
    return command

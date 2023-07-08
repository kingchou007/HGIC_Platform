import control_api as SwarmControl

if __name__ == "__main__":
    # take_off()    
    while True:
        command = input("Please input the command to control swarm: ")
        if command == "start":
            SwarmControl.take_off()
        elif command == "merge":
            SwarmControl.merge()
        elif command == "spread":
            SwarmControl.spread()
        elif command == "left":
            SwarmControl.left()
        elif command == "right":
            SwarmControl.right()
        elif command == "up":
            SwarmControl.up()
        elif command == "down":
            SwarmControl.down()
        elif command == "forward":
            SwarmControl.forward()
        elif command == "backward":
            SwarmControl.backward()
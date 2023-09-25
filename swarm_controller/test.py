import airsim
import time
import numpy as np
import os

# Build a connection with AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Initialize the UAVs position
origin_x = [0, 2, 4, 0, 2, 4, 0, 2, 4]
origin_y = [0, 0, 0, -3, -2, -3, 3, 2, 3]
origin_z = []


def get_UAV_pos(client, vehicle_name="SimpleFlight"):
    global origin_x
    global origin_y
    # global origin_z
    state = client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
    x = state.position.x_val
    y = state.position.y_val
    i = int(vehicle_name[3])
    x += origin_x[i - 1]
    y += origin_y[i - 1]
    pos = np.array([[x], [y]])
    return pos


client = airsim.MultirotorClient()


def take_off():
    for i in range(9):  # adjust the number based on the number of UAVs
        name = "UAV" + str(i + 1)
        client.enableApiControl(True, name)
        client.armDisarm(True, name)
        if i != 8:
            client.takeoffAsync(vehicle_name=name)
        else:
            client.takeoffAsync(vehicle_name=name).join()

    for i in range(9):  # adjust the number based on the number of UAVs
        name = "UAV" + str(i + 1)
        if i != 8:
            client.moveToZAsync(-3, 1, vehicle_name=name)
        else:
            client.moveToZAsync(-3, 1, vehicle_name=name).join()


def merge():
    v_max = 2
    r_max = 20
    r_repulsion = 2
    k_sep = 20
    k_coh = 0.5
    k_rep = 12
    k_mig = 1
    d_desired = 3.5

    pos_mig = np.array([[5], [0]])
    v_cmd = np.zeros([2, 9])
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    for t in range(500):
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)
            r_mig = pos_mig - pos_i
            v_mig = k_mig * r_mig / np.linalg.norm(r_mig)
            v_sep = np.zeros([2, 1])
            v_coh = np.zeros([2, 1])
            v_rep = np.zeros([2, 1])
            N_i = 0

            for j in range(9):
                if j != i:
                    N_i += 1
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)
                    r_ij = pos_j - pos_i
                    dist = np.linalg.norm(r_ij)
                    if dist < r_max:
                        if dist < r_repulsion:
                            v_rep += -k_rep * (r_repulsion - dist) * r_ij / dist
                        elif dist < d_desired:
                            v_sep += -k_sep * (d_desired - dist) * r_ij / dist
                        else:
                            v_coh += k_coh * (dist - d_desired) * r_ij / dist
            v_sep = v_sep / N_i
            v_coh = v_coh / N_i
            v_cmd[:, i : i + 1] = v_sep + v_coh + v_rep + v_mig

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )  # 通过API发送速度指令


def get_swarm_center():
    for i in range(9):
        # get position of each UAV
        name = "UAV" + str(i + 1)
        pos = get_UAV_pos(client, vehicle_name=name)
        if i == 0:
            pos_sum = pos
        else:
            pos_sum += pos
    pos_mig = pos_sum / 9
    return pos_mig


def spread():
    v_max = 2
    r_max = 20
    k_sep = 30
    k_coh = 1
    k_mig = 1
    pos_mig = get_swarm_center()
    v_cmd = np.zeros([2, 9])

    # get average height of all UAVs
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    # Main loop to control UAVs
    for t in range(300):
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)
            r_mig = pos_mig - pos_i
            v_mig = k_mig * r_mig / np.linalg.norm(r_mig)
            v_sep = np.zeros([2, 1])
            v_coh = np.zeros([2, 1])
            N_i = 0
            for j in range(9):
                if j != i:
                    N_i += 1
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)
                    if np.linalg.norm(pos_j - pos_i) < r_max:
                        r_ij = pos_j - pos_i
                        v_sep += -k_sep * r_ij / np.linalg.norm(r_ij)
                        v_coh += k_coh * r_ij
            v_sep = v_sep / N_i
            v_coh = v_coh / N_i
            v_cmd[:, i : i + 1] = v_sep + v_coh + v_mig

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def flocking():
    pass


# basic movement
def left():
    v_max = 5  # adjust maximum velocity to the left
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    # Main loop to control UAVs
    for t in range(300):
        v_cmd = np.zeros([2, 9])  # x, y, z velocity

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)
            # calculate x-axis velocity component based on current position
            v_cmd[0, i] = 0
            # keep y-axis velocity component at 0
            v_cmd[1, i] = -v_max

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def right():
    v_max = 5
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    for t in range(300):
        v_cmd = np.zeros([2, 9])
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)
            v_cmd[0, i] = 0
            v_cmd[1, i] = v_max
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def up():
    v_max = 60  # adjust maximum velocity upwards
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd) - v_max

    # Main loop to control UAVs
    for t in range(300):
        v_cmd = np.zeros([2, 9])

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)

            # calculate y-axis velocity component based on current position
            v_cmd[1, i] = 0

            # keep x-axis and z-axis velocity components at 0
            v_cmd[0, i] = 0

        # move all UAVs with the calculated velocity commands
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def down():
    v_max = 5  # adjust maximum velocity upwards
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]

    # Main loop to control UAVs
    for t in range(200):
        v_cmd = np.zeros([2, 9])
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)
            v_cmd[1, i] = 0
            v_cmd[0, i] = 0
            z_cmd[i] = v_max
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd[i], 0.1, vehicle_name=name_i
            )


def forward():
    v_max = 5
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    for t in range(300):
        v_cmd = np.zeros([2, 9])
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)
            v_cmd[0, i] = v_max
            v_cmd[1, i] = 0
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def backward():
    v_max = 5
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    for t in range(300):
        v_cmd = np.zeros([2, 9])
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)
            v_cmd[0, i] = -v_max
            v_cmd[1, i] = 0
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def fly_circle():
    v_max = 2
    r_max = 5
    k_sep = 3  # Increase separation coefficient

    k_coh = 3  # Decrease cohesion coefficient
    k_mig = 1
    k_rep = 10
    pos_mig = np.array([[25], [0]])
    v_cmd = np.zeros([2, 9])

    # get average height of all UAVs
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    # Main loop to control UAVs
    for t in range(500):
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)
            r_mig = pos_mig - pos_i
            angle = 2 * np.pi * t / 500  # Angle based on the current time step
            circle_radius = 20  # Radius of the circular pattern
            circle_center = np.array([[0], [0]])  # Center point of the circular pattern

            if t < 50:
                # Temporary separation behavior with collision avoidance and repulsion zone
                separation_radius = 5  # Radius for temporary separation
                separation_center = np.array(
                    [[0], [10]]
                )  # Center point for temporary separation
                repulsion_radius = 2  # Radius of the repulsion zone

                v_sep_temp = np.zeros([2, 1])  # Temporary separation velocity
                v_avoid = np.zeros([2, 1])  # Collision avoidance velocity

                for j in range(9):
                    if j != i:
                        name_j = "UAV" + str(j + 1)
                        pos_j = get_UAV_pos(client, vehicle_name=name_j)
                        r_ij = pos_j - pos_i
                        separation_distance = np.linalg.norm(r_ij)

                        if separation_distance < separation_radius:
                            v_avoid += (
                                k_sep
                                * (separation_radius / separation_distance)
                                * (r_ij / separation_distance)
                            )

                        if separation_distance < repulsion_radius:
                            v_avoid += (
                                k_rep
                                * (repulsion_radius / separation_distance) ** 2
                                * (r_ij / separation_distance)
                            )

                v_sep_temp -= v_avoid
                v_mig = v_sep_temp

            elif t < 450:
                # Fixed triangular formation and circular pattern with collision avoidance
                formation_radius = 20  # Radius of the triangular formation
                formation_center = np.array(
                    [[0], [15]]
                )  # Center point of the triangular formation
                angle_offset = (
                    2 * np.pi / 3
                )  # Angular offset between UAVs in the triangular formation

                desired_angle = angle_offset * i + angle
                desired_position = formation_center + formation_radius * np.array(
                    [[np.cos(desired_angle)], [np.sin(desired_angle)]]
                )
                v_mig = k_mig * (desired_position - pos_i)

                # Add circular motion
                # v_mig += k_mig * np.array([[np.cos(angle)], [np.sin(angle)]]) * circle_radius

                v_avoid = np.zeros([2, 1])  # Collision avoidance velocity

                for j in range(9):
                    if j != i:
                        name_j = "UAV" + str(j + 1)
                        pos_j = get_UAV_pos(client, vehicle_name=name_j)
                        r_ij = pos_j - pos_i
                        separation_distance = np.linalg.norm(r_ij)

                        if separation_distance < formation_radius:
                            v_avoid += (
                                k_sep
                                * (formation_radius / separation_distance)
                                * (r_ij / separation_distance)
                            )

                v_mig += v_avoid

            else:
                # Gradually bring the group closer together
                target_center = np.array(
                    [[0], [0]]
                )  # Target center point for convergence
                convergence_speed = 0.1  # Speed of convergence
                v_mig = k_mig * (target_center - pos_i) * convergence_speed

            v_sep = np.zeros([2, 1])
            v_coh = np.zeros([2, 1])
            N_i = 0
            for j in range(9):
                if j != i:
                    N_i += 1
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)
                    if np.linalg.norm(pos_j - pos_i) < r_max:
                        r_ij = pos_j - pos_i
                        v_sep += -k_sep * r_ij / np.linalg.norm(r_ij)
                        v_coh += k_coh * r_ij
            v_sep = v_sep / N_i
            v_coh = v_coh / N_i
            v_cmd[:, i : i + 1] = v_sep + v_coh + v_mig

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def test():
    k_mig = 1
    pos_mig = np.array([[25], [0]])
    v_cmd = np.zeros([2, 9])

    # get average height of all UAVs
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    # Main loop to control UAVs
    for t in range(600):
        angle = 2 * np.pi * t / 600  # Angle based on the current time step
        circle_radius = 20  # Radius of the circular pattern

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)

            # Define the subgroups
            subgroup = (i + 1) // 3

            # Define the subgroup's center point
            subgroup_center_angle = 2 * np.pi * subgroup / 3
            subgroup_center = circle_radius * np.array(
                [
                    [np.cos(subgroup_center_angle + angle)],
                    [np.sin(subgroup_center_angle + angle)],
                ]
            )

            # Define the formation points for each UAV within the subgroup
            formation_radius = (
                5  # Distance between the UAVs in the triangular formation
            )
            formation_angle_offset = (
                2 * np.pi / 3
            )  # Angular offset between UAVs in the triangular formation
            formation_angle = formation_angle_offset * (i % 3)
            formation_point = subgroup_center + formation_radius * np.array(
                [[np.cos(formation_angle)], [np.sin(formation_angle)]]
            )

            # Calculate the desired velocity for each UAV to reach its formation point
            v_mig = k_mig * (formation_point - pos_i)
            v_cmd[:, i : i + 1] = v_mig

        # Set the velocity for each UAV
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def test2():
    k_mig = 1
    k_rep = 10  # Repulsion coefficient
    pos_mig = np.array([[25], [0]])
    v_cmd = np.zeros([2, 9])

    safe_distance = 5  # Safe distance between UAVs
    repulsion_distance = 3  # Distance at which UAVs start repelling each other

    # get average height of all UAVs
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    # Main loop to control UAVs
    for t in range(600):
        angle = 2 * np.pi * t / 600  # Angle based on the current time step
        circle_radius = 15  # Radius of the circular pattern

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)

            # Define the subgroups
            subgroup = i // 3

            # Define the subgroup's center point
            subgroup_center_angle = 2 * np.pi * subgroup / 3
            subgroup_center = circle_radius * np.array(
                [
                    [np.cos(subgroup_center_angle + angle)],
                    [np.sin(subgroup_center_angle + angle)],
                ]
            )

            # Define the formation points for each UAV within the subgroup
            formation_radius = (
                5  # Distance between the UAVs in the triangular formation
            )
            formation_angle_offset = (
                2 * np.pi / 3
            )  # Angular offset between UAVs in the triangular formation
            formation_angle = formation_angle_offset * (i % 3)
            formation_point = subgroup_center + formation_radius * np.array(
                [[np.cos(formation_angle)], [np.sin(formation_angle)]]
            )

            # Calculate the desired velocity for each UAV to reach its formation point
            v_mig = k_mig * (formation_point - pos_i)

            # Collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        # Calculate a repulsion vector
                        repulsion_vector = pos_i - pos_j
                        # Normalize and scale by the repulsion coefficient
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        # If the distance is below the repulsion distance, scale the repulsion vector further
                        if distance < repulsion_distance:
                            v_rep *= 2

            v_cmd[:, i : i + 1] = v_mig + v_rep

        # Set the velocity for each UAV
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def circle_move():
    k_mig = 1
    k_rep = 10  # Repulsion coefficient
    pos_mig = np.array([[25], [0]])
    v_cmd = np.zeros([2, 9])

    safe_distance = 2.5  # Safe distance between UAVs
    repulsion_distance = 1.5  # Distance at which UAVs start repelling each other

    # get average height of all UAVs
    # for easy to control, we set the height of all UAVs to be the same
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)
    v_rep = np.zeros([2, 1])

    # Main loop to control UAVs
    for t in range(600):
        angle = (
            2 * np.pi * t / 600
        )  # Angle based on the current time step（we hope have a circle movement）
        group_center_radius = 20  # Set the radius of the circle for the group center

        # Calculate the swarm center point
        group_center = group_center_radius * np.array(
            [[np.cos(angle)], [np.sin(angle)]]
        )

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(
                client, vehicle_name=name_i
            )  # store the position of the current UAV

            # Define the formation points for each UAV within the group
            formation_radius = 5  # Distance between the UAVs in the formation
            formation_angle_offset = (
                2 * np.pi / 9
            )  # Angular offset between UAVs in the formation
            formation_angle = (
                formation_angle_offset * i
            )  # calculates the specific angle for the current UAV based on its index
            formation_point = group_center + formation_radius * np.array(
                [[np.cos(formation_angle)], [np.sin(formation_angle)]]
            )

            # Calculate the desired velocity for each UAV to reach its formation point
            v_mig = k_mig * (formation_point - pos_i)

            # Collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        # Calculate a repulsion vector
                        repulsion_vector = pos_i - pos_j
                        # Normalize and scale by the repulsion coefficient
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        # If the distance is below the repulsion distance, scale the repulsion vector further
                        if distance < repulsion_distance:
                            v_rep *= 2

            v_cmd[:, i : i + 1] = v_mig + v_rep

        # Set the velocity for each UAV
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def test_obs():
    k_mig = 1
    k_rep = 10
    max_speed = 15  # Maximum speed limit for the drones
    safe_distance = 2.5
    repulsion_distance = 2
    avoidance_distance = 10
    group_center_radius = 20
    formation_radius = 5
    k_obs = 5  # Strength of the repulsive field around obstacles
    obs_dist = 5  # Distance at which the drone starts to "feel" an obstacle

    obstacles = [
        ((15, 30), (40, 60)),  # Obstacle 1
        ((45, 60), (0, 10)),  # Obstacle 2
        ((-15, -40), (15, 40)),  # Obstacle 3
        ((-15, -40), (-20, -40)),  # Obstacle 4
        ((10, 30), (-70, -50)),
    ]  # Obstacle 5

    z_cmd = np.mean(
        [
            client.getMultirotorState(
                vehicle_name="UAV" + str(i + 1)
            ).kinematics_estimated.position.z_val
            for i in range(9)
        ]
    )

    for t in range(600):
        angle = 2 * np.pi * t / 900
        group_center = group_center_radius * np.array(
            [[np.cos(angle)], [np.sin(angle)]]
        )
        team_positions = [
            get_UAV_pos(client, vehicle_name="UAV" + str(i + 1)) for i in range(9)
        ]

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = team_positions[i]
            formation_angle = 2 * np.pi * i / 9
            formation_point = group_center + formation_radius * np.array(
                [[np.cos(formation_angle)], [np.sin(formation_angle)]]
            )
            v_mig = k_mig * (formation_point - pos_i)

            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    pos_j = team_positions[j]
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        repulsion_vector = pos_i - pos_j
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        if distance < repulsion_distance:
                            v_rep *= 2

            v_avoid = np.zeros([2, 1])
            for obstacle in obstacles:
                (x_min, x_max), (y_min, y_max) = obstacle

                # Calculate the closest point on the obstacle's bounding box to the drone's position
                closest_x = max(x_min, min(pos_i[0, 0], x_max))
                closest_y = max(y_min, min(pos_i[1, 0], y_max))

                obstacle_vector = pos_i - np.array([[closest_x], [closest_y]])
                obstacle_distance = np.linalg.norm(obstacle_vector)

                if obstacle_distance < obs_dist:
                    avoidance_vector = obstacle_vector / obstacle_distance

                    # Calculate the avoidance direction based on drone's velocity
                    avoidance_direction = np.sign(
                        np.dot(v_cmd.flatten(), avoidance_vector.flatten())
                    )

                    # Gradual transition for smoother avoidance behavior
                    transition_factor = 1 / (
                        1 + np.exp((obstacle_distance - obs_dist) / 2)
                    )

                    # Smoothly adjust avoidance direction
                    avoidance_vector = (
                        (1 - transition_factor) * avoidance_vector
                        + transition_factor * avoidance_direction * v_cmd
                    )

                    # Apply weight based on obstacle distance
                    avoidance_weight = 1 / obstacle_distance

                    # Update v_avoid based on avoidance force
                    v_avoid += k_obs * avoidance_weight * avoidance_vector
                v_cmd = v_mig + v_rep + v_avoid

            # Apply speed limit
            # if np.linalg.norm(v_cmd) > max_speed:
            #     v_cmd = max_speed * (v_cmd / np.linalg.norm(v_cmd))

            client.moveByVelocityZAsync(
                v_cmd[0, 0], v_cmd[1, 0], z_cmd, 0.1, vehicle_name=name_i
            )


def circle_move_with_obstacles():  ### this one is working
    k_mig = 0.5
    k_rep = 10  # Repulsion coefficient
    pos_mig = np.array([[25], [0]])
    v_cmd = np.zeros([2, 9])

    safe_distance = 2.5  # Safe distance between UAVs
    repulsion_distance = 1.5  # Distance at which UAVs start repelling each other
    feel_distance = 5  # Distance at which UAVs start sensing obstacles

    obstacles = [
        ((15, 30), (40, 60)),  # Obstacle 1
        ((45, 60), (0, 10)),  # Obstacle 2
        ((-15, -40), (15, 40)),  # Obstacle 3
        ((-15, -40), (-20, -40)),  # Obstacle 4
        ((10, 30), (-70, -50)),
    ]  # Obstacle 5

    # get average height of all UAVs
    # for easy to control, we set the height of all UAVs to be the same
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)
    v_rep = np.zeros([2, 1])

    # Main loop to control UAVs
    for t in range(600):
        angle = (
            2 * np.pi * t / 600
        )  # Angle based on the current time step（we hope have a circle movement）
        group_center_radius = 20  # Set the radius of the circle for the group center

        # Calculate the swarm center point
        group_center = group_center_radius * np.array(
            [[np.cos(angle)], [np.sin(angle)]]
        )

        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(
                client, vehicle_name=name_i
            )  # store the position of the current UAV

            # Define the formation points for each UAV within the group
            formation_radius = 5  # Distance between the UAVs in the formation
            formation_angle_offset = (
                2 * np.pi / 9
            )  # Angular offset between UAVs in the formation
            formation_angle = (
                formation_angle_offset * i
            )  # calculates the specific angle for the current UAV based on its index
            formation_point = group_center + formation_radius * np.array(
                [[np.cos(formation_angle)], [np.sin(formation_angle)]]
            )

            # Calculate the desired velocity for each UAV to reach its formation point
            v_mig = k_mig * (formation_point - pos_i)

            # Collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        # Calculate a repulsion vector
                        repulsion_vector = pos_i - pos_j
                        # Normalize and scale by the repulsion coefficient
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        # If the distance is below the repulsion distance, scale the repulsion vector further
                        if distance < repulsion_distance:
                            v_rep *= 2

            # Obstacle avoidance
            v_obstacle = np.zeros([2, 1])
            for obstacle in obstacles:
                obstacle_pos = np.array(obstacle[0])
                obstacle_radius = obstacle[1]
                obstacle_vector = pos_i - obstacle_pos
                obstacle_distance = np.linalg.norm(obstacle_vector)
                if obstacle_distance < feel_distance:
                    obstacle_direction = obstacle_vector / obstacle_distance
                    # Add repulsion behavior if UAV is close to the obstacle
                    if obstacle_distance < repulsion_distance:
                        v_repulsion = (
                            k_rep
                            * (obstacle_direction / obstacle_distance)
                            * (repulsion_distance - obstacle_distance)
                        )
                        v_obstacle += v_repulsion
                    else:
                        v_obstacle += (
                            k_rep
                            * (obstacle_direction / obstacle_distance)
                            * (feel_distance - obstacle_distance)
                        )

            v_cmd[:, i : i + 1] = v_mig + v_rep + v_obstacle

        # Set the velocity for each UAV
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )

    for i in range(9):
        name_i = "UAV" + str(i + 1)
        client.moveByVelocityZAsync(0, 0, z_cmd, 0.1, vehicle_name=name_i)


def cover_block():
    k_mig = 0.5
    safe_distance = [18, 18, 18]
    feel_distance_obstacle = 10

    safe_distance_uav = 2.5  # Safe distance between UAVs
    obstacles = [
        (60, 5),  # Obstacle 1 center point
        (20, 60),  # Obstacle 2 center point
        (-25, 30),
    ]  # Obstacle 3 center point

    repulsion_distance = 15  # Distance at which UAVs repel the center point
    repulsion_distance_uav = 2.5  # Distance at which UAVs repel each other
    k_rep = 10  # Repulsion coefficient

    # Main loop to control UAVs
    for t in range(600):
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = np.append(
                get_UAV_pos(client, vehicle_name=name_i)[:2], -40
            )  # Set x, y, and z-coordinate to -40

            # Determine the obstacle index for the current UAV
            obstacle_index = i // 3

            # Get the obstacle center point and safe distance for the corresponding index
            obstacle_center = np.array(obstacles[obstacle_index])
            obstacle_safe_distance = safe_distance[obstacle_index]

            # Calculate the angle for the circular formation
            angle = 2 * np.pi * i / 3

            # Calculate the position for the UAV in the circular formation
            formation_pos = obstacle_center + obstacle_safe_distance * np.array(
                [np.cos(angle), np.sin(angle)]
            )

            # Calculate the desired velocity for each UAV to reach its formation point
            v_mig = k_mig * (formation_pos[:2] - pos_i[:2])

            # Collision avoidance with other UAVs
            v_rep_uav = np.zeros(2)
            for j in range(9):
                if j != i:
                    name_j = "UAV" + str(j + 1)
                    pos_j = np.append(
                        get_UAV_pos(client, vehicle_name=name_j)[:2], -40
                    )  # Set x, y, and z-coordinate to -40
                    distance_uav = np.linalg.norm(pos_j - pos_i)
                    if distance_uav < safe_distance_uav:
                        # Calculate the repulsion vector for UAVs
                        repulsion_vector_uav = pos_i[:2] - pos_j[:2]
                        # Scale by the repulsion coefficient and the distance to create repulsion effect
                        v_rep_uav += (
                            k_rep
                            * (safe_distance_uav - distance_uav)
                            * repulsion_vector_uav
                        )
                        # If the distance is below the repulsion distance, scale the repulsion vector further
                        if distance_uav < repulsion_distance_uav:
                            v_rep_uav *= 2

            # Collision avoidance with obstacles
            v_rep_obstacle = np.zeros(2)
            distance_to_center = np.linalg.norm(pos_i[:2] - obstacle_center)
            if distance_to_center < repulsion_distance:
                repulsion_vector_obstacle = pos_i[:2] - obstacle_center
                v_rep_obstacle = (
                    k_rep
                    * (repulsion_distance - distance_to_center)
                    * repulsion_vector_obstacle
                )

            # Obstacle avoidance
            v_obstacle = np.zeros(2)
            for obstacle in obstacles:
                obstacle_center = np.array(obstacle)
                obstacle_vector = pos_i[:2] - obstacle_center
                obstacle_distance = np.linalg.norm(obstacle_vector)
                if obstacle_distance < feel_distance_obstacle:
                    obstacle_direction = obstacle_center - pos_i[:2]
                    v_obstacle += (
                        k_rep
                        * (obstacle_direction / obstacle_distance)
                        * (feel_distance_obstacle - obstacle_distance)
                    )

            # Adjust the desired velocity based on obstacle avoidance
            v_mig += v_rep_uav + v_rep_obstacle + v_obstacle

            # Set the velocity for each UAV
            client.moveByVelocityZAsync(
                v_mig[0], v_mig[1], -40, 0.1, vehicle_name=name_i
            )


def spiral_motion():
    k_mig = 2  # Migration coefficient
    safe_distance = [18, 18, 18]
    feel_distance_obstacle = 10
    safe_distance_uav = 2.5  # Safe distance between UAVs
    obstacles = [(60, 5), (20, 60), (-25, 30)]  # Obstacle center points
    repulsion_distance = 15  # Distance at which UAVs repel the center point
    repulsion_distance_uav = 2.5  # Distance at which UAVs repel each other
    k_rep = 15  # Repulsion coefficient

    # Spiral formation parameters
    spiral_center_height = -40  # Initial height at the center of the spiral
    spiral_increment = 1  # Change in height after each complete spiral
    spiral_max_radius = 18  # Maximum radius of the spiral
    spiral_min_radius = 25  # Minimum radius of the spiral
    spiral_radius_increment = 1  # Change in radius after each complete spiral

    # Main loop to control UAVs
    for t in range(600):
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = np.append(
                get_UAV_pos(client, vehicle_name=name_i)[:2], -40
            )  # Set x, y, and z-coordinate to -40

            # Determine the obstacle index for the current UAV
            obstacle_index = i // 3

            # Get the obstacle center point and safe distance for the corresponding index
            obstacle_center = np.array(
                [obstacles[obstacle_index][0], obstacles[obstacle_index][1], 0]
            )

            obstacle_safe_distance = safe_distance[obstacle_index]

            # Calculate the current height and radius based on the current time step
            current_height = spiral_center_height - t * spiral_increment / 600
            current_radius = spiral_min_radius + (
                (spiral_max_radius - spiral_min_radius) * np.sin(np.pi * t / 600)
            )

            # Calculate the angle for the spiraling formation
            angle = 2 * np.pi * i / 3 + 2 * np.pi * t / 600

            # Calculate the position for the UAV in the spiraling formation
            formation_pos = obstacle_center + current_radius * np.array(
                [np.cos(angle), np.sin(angle), current_height]
            )

            # Calculate the desired velocity for each UAV to reach its formation point
            v_mig = k_mig * (formation_pos[:2] - pos_i[:2])

            # ... Rest of your original code for collision avoidance ...
            # Collision avoidance with obstacles
            # Collision avoidance with other UAVs
            v_rep_uav = np.zeros(2)
            for j in range(9):
                if j != i:
                    name_j = "UAV" + str(j + 1)
                    pos_j = np.append(
                        get_UAV_pos(client, vehicle_name=name_j)[:2], -40
                    )  # Set x, y, and z-coordinate to -40
                    distance_uav = np.linalg.norm(pos_j - pos_i)
                    if distance_uav < safe_distance_uav:
                        # Calculate the repulsion vector for UAVs
                        repulsion_vector_uav = pos_i[:2] - pos_j[:2]
                        # Scale by the repulsion coefficient and the distance to create repulsion effect
                        v_rep_uav += (
                            k_rep
                            * (safe_distance_uav - distance_uav)
                            * repulsion_vector_uav
                        )
                        # If the distance is below the repulsion distance, scale the repulsion vector further
                        if distance_uav < repulsion_distance_uav:
                            v_rep_uav *= 2

            v_rep_obstacle = np.zeros(2)
            # distance_to_center = np.linalg.norm(pos_i[:2] - obstacle_center)
            distance_to_center = np.linalg.norm(pos_i - obstacle_center)

            if distance_to_center < repulsion_distance:
                repulsion_vector_obstacle = pos_i[:2] - obstacle_center
                v_rep_obstacle = (
                    k_rep
                    * (repulsion_distance - distance_to_center)
                    * repulsion_vector_obstacle
                )

            # Obstacle avoidance
            v_obstacle = np.zeros(2)
            for obstacle in obstacles:
                obstacle_center = np.array(obstacle)
                obstacle_vector = pos_i[:2] - obstacle_center
                obstacle_distance = np.linalg.norm(obstacle_vector)
                if obstacle_distance < feel_distance_obstacle:
                    obstacle_direction = obstacle_center - pos_i[:2]
                    v_obstacle += (
                        k_rep
                        * (obstacle_direction / obstacle_distance)
                        * (feel_distance_obstacle - obstacle_distance)
                    )

            # Adjust the desired velocity based on obstacle avoidance
            v_mig += v_rep_uav + v_rep_obstacle + v_obstacle

            # Set the velocity and altitude for each UAV
            client.moveByVelocityZAsync(
                v_mig[0], v_mig[1], current_height, 0.1, vehicle_name=name_i
            )


def target():
    # Parameters
    radius = 50  # radius of circle
    height = -40  # altitude
    speed = 30  # speed of UAV
    name_i = "UAV1"  # the UAV we're controlling
    time_step = 0.1  # Time interval in seconds

    # Get current position
    current_pos = get_UAV_pos(client, vehicle_name=name_i)

    # Calculate the time it takes for one complete circle
    circle_time = 2 * math.pi * radius / speed

    # Calculate the angle increment for each time step
    angle_increment = 2 * math.pi / (circle_time / time_step)

    # Set initial angle
    angle = 0

    # Main loop to control UAV
    for t in range(600):
        # Calculate the UAV's position on the circle
        x = current_pos[0] + radius * np.cos(angle)
        y = current_pos[1] + radius * np.sin(angle)

        # Calculate the distance and direction to the target position
        dx = x - current_pos[0]
        dy = y - current_pos[1]
        distance = np.sqrt(dx**2 + dy**2)

        # Calculate the velocity components based on the distance and speed
        v_x = -speed * (dx / distance)
        v_y = -speed * (dy / distance)

        # Send the command to the UAV
        client.moveByVelocityZAsync(
            float(v_x), float(v_y), height, time_step, vehicle_name=name_i
        )

        # Wait for the next time step
        time.sleep(time_step)

        # Update the angle
        angle += angle_increment


def chasing():
    # Parameters
    radius = 60  # radius of circle
    height = -40  # altitude
    speed = 30  # speed of UAV
    time_step = 0.1  # Time interval in seconds

    # Assign UAVs to chase UAV 1
    chase_uavs = ["UAV2", "UAV3", "UAV4"]
    target_uav = "UAV1"

    # Main loop to control chasing UAVs
    for t in range(400):
        # Get current position of target UAV
        target_pos = get_UAV_pos(client, vehicle_name=target_uav)

        # Loop through the chasing UAVs
        for name_i in chase_uavs:
            # Get current position of chasing UAV
            current_pos = get_UAV_pos(client, vehicle_name="UAV1")

            # Calculate the direction vector towards the target UAV
            direction = target_pos - current_pos
            distance = np.linalg.norm(direction)

            # Normalize the direction vector
            if distance != 0:
                direction /= distance

            # Calculate the velocity vector
            velocity = speed * direction

            # Send the command to the chasing UAV
            client.moveByVelocityZAsync(
                float(velocity[0]),
                float(velocity[1]),
                height,
                time_step,
                vehicle_name=name_i,
            )

        # Wait for the next time step
        time.sleep(time_step)


def target_and_chasing():
    # Parameters
    radius = 30  # radius of circle
    height = -30  # altitude
    speed = 20  # speed of UAV
    time_step = 0.1  # Time interval in seconds
    speed_chasing = 25

    # Assign UAVs to chase UAV 1
    chase_uavs = ["UAV2", "UAV3", "UAV4"]
    target_uav = "UAV1"
    toat_uavs = ["UAV1", "UAV2", "UAV3", "UAV4"]

    # Get initial position of UAV1
    current_pos = get_UAV_pos(client, vehicle_name="UAV1")

    # Calculate the time it takes for one complete circle
    circle_time = 2 * math.pi * radius / speed

    # Calculate the angle increment for each time step
    angle_increment = 2 * math.pi / (circle_time / time_step)

    # Set initial angle
    angle = 0

    # Main loop to control UAVs
    for t in range(300):
        # Calculate the position of UAV1 on the circle
        x = current_pos[0] + radius * np.cos(angle)
        y = current_pos[1] + radius * np.sin(angle)

        # Calculate the distance and direction to the target position
        dx = x - current_pos[0]
        dy = y - current_pos[1]
        distance = np.sqrt(dx**2 + dy**2)

        # Calculate the velocity components based on the distance and speed
        v_x = -speed * (dx / distance)
        v_y = -speed * (dy / distance)

        # Send the command to UAV1
        client.moveByVelocityZAsync(
            float(v_x), float(v_y), height, time_step, vehicle_name=target_uav
        )

        # Get current position of UAV1
        time.sleep(time_step)
        current_pos = get_UAV_pos(client, vehicle_name=target_uav)
        # Get current position of target UAV (UAV1)
        # time.sleep(time_step)
        target_pos = current_pos

        # Loop through the chasing UAVs
        for name_i in chase_uavs:
            # Get current position of chasing UAV
            current_pos_chase = get_UAV_pos(client, vehicle_name=name_i)

            # Calculate the direction vector towards the target UAV (UAV1)
            direction = target_pos - current_pos_chase
            distance = np.linalg.norm(direction)  # distance formula

            # Normalize the direction vector
            if distance != 0:
                direction /= distance

            # Calculate the velocity vector
            repulsion_vectors = uav_collision()

            # Combine the chasing velocity and collision avoidance velocity

            velocity = (
                speed_chasing * direction + repulsion_vectors[toat_uavs.index(name_i)]
            )
            # velocity = speed_chasing * direction

            # Send the command to the chasing UAVs
            client.moveByVelocityZAsync(
                float(velocity[0]),
                float(velocity[1]),
                height,
                time_step,
                vehicle_name=name_i,
            )

        # Update the angle
        angle += angle_increment
    for i in toat_uavs:
        client.moveByVelocityZAsync(0, 0, height, time_step, vehicle_name=i)


def stop_all():
    time_step = 0.1
    total_uavs = [
        "UAV1",
        "UAV2",
        "UAV3",
        "UAV4",
        "UAV5",
        "UAV6",
        "UAV7",
        "UAV8",
        "UAV9",
    ]
    for i in total_uavs:
        height = get_UAV_pos(client, vehicle_name=i)[2]
        client.moveByVelocityZAsync(0, 0, height, time_step, vehicle_name=i)


def env_collision():
    # Parameters

    # Get the position of the center point
    obstacle_collision = False
    pass


def uav_collision():
    repulsion_distance = 10  # Distance at which UAVs repel the center point
    safe_distance_uav = 5  # Safe distance between UAVs
    K_rep = 6  # Repulsion coefficient
    repulsion_vectors = []  # List to store the repulsion vectors for each UAV
    for i in range(9):
        name_i = "UAV" + str(i + 1)
        pos_i = get_UAV_pos(
            client, vehicle_name=name_i
        )  # store the position of the current UAV

        # Collision avoidance
        v_rep = np.zeros([2, 1])
        for j in range(9):
            if j != i:
                name_j = "UAV" + str(j + 1)
                pos_j = get_UAV_pos(client, vehicle_name=name_j)
                distance = np.linalg.norm(pos_j - pos_i)
                if distance < safe_distance_uav:
                    # Calculate a repulsion vector
                    repulsion_vector = pos_i - pos_j
                    # Normalize and scale by the repulsion coefficient
                    v_rep += (
                        K_rep
                        * (repulsion_vector / np.linalg.norm(repulsion_vector))
                        * (safe_distance_uav - distance)
                    )
                    # If the distance is below the repulsion distance, scale the repulsion vector further
                    if distance < repulsion_distance:
                        v_rep *= 2
        repulsion_vectors.append(v_rep)
    return repulsion_vectors


def form_circle():
    # Set the desired formation parameters
    k_mig = 1
    k_rep = 10
    formation_radius = 5
    safe_distance = 2.5
    repulsion_distance = 1.5
    formation_angle_offset = 2 * np.pi / 9

    # Define the group center and the repulsion vector
    group_center = np.array([[25], [0]])
    v_rep = np.zeros([2, 1])

    # Compute the average height of all UAVs
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    # Define the velocity command
    v_cmd = np.zeros([2, 9])

    # Main loop to control UAVs
    for t in range(500):  # assuming 200 time steps are enough to form the circle
        for i in range(9):
            # Define the name and position of the current UAV
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)

            # Calculate the formation point for the current UAV
            formation_angle = formation_angle_offset * i
            formation_point = group_center + formation_radius * np.array(
                [[np.cos(formation_angle)], [np.sin(formation_angle)]]
            )

            # Compute the desired velocity for the current UAV
            v_mig = k_mig * (formation_point - pos_i)

            # Perform collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    # Define the name and position of the other UAV
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)

                    # Compute the repulsion vector if the other UAV is too close
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        repulsion_vector = pos_i - pos_j
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        if distance < repulsion_distance:
                            v_rep *= 2

            # Store the computed velocity command for the current UAV
            v_cmd[:, i : i + 1] = v_mig + v_rep

        # Command each UAV to move according to the computed velocity command
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def circle_move_2():
    k_mig = 1
    k_rep = 10  # Repulsion coefficient
    v_cmd = np.zeros([2, 9])
    safe_distance = 2.5  # Safe distance between UAVs
    repulsion_distance = 1.5  # Distance at which UAVs start repelling each other
    group_center_radius = 20  # Set the radius of the circle for the group center
    z_cmd = [
        client.getMultirotorState(
            vehicle_name="UAV" + str(i + 1)
        ).kinematics_estimated.position.z_val
        for i in range(9)
    ]
    z_cmd = np.mean(z_cmd)

    # Main loop to control UAVs
    for t in range(600):
        angle = (
            2 * np.pi * t / 600
        )  # Angle based on the current time step（we hope have a circle movement）
        # Calculate the swarm center point
        group_center = group_center_radius * np.array(
            [[np.cos(angle)], [np.sin(angle)]]
        )
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)
            # Calculate the desired velocity for each UAV to reach its formation point
            v_mig = k_mig * (group_center - pos_i)

            # Collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_i)
                    distance = np.linalg.norm(pos_j - pos_i)

                    if distance < safe_distance:
                        # Calculate a repulsion vector
                        repulsion_vector = pos_i - pos_j
                        # Normalize and scale by the repulsion coefficient, but only if the norm of the repulsion vector is not zero
                        if np.linalg.norm(repulsion_vector) != 0:
                            v_rep += (
                                k_rep
                                * (repulsion_vector / np.linalg.norm(repulsion_vector))
                                * (safe_distance - distance)
                            )
                            # If the distance is below the repulsion distance, scale the repulsion vector further
                            if distance < repulsion_distance:
                                v_rep *= 2

            v_cmd[:, i : i + 1] = v_mig + v_rep

        # Set the velocity for each UAV
        for i in range(9):
            name_i = "UAV" + str(i + 1)
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def line():
    # Set the desired formation parameters
    k_mig = 0.5
    k_rep = 10
    grid_size = 30
    cell_distance = 5
    safe_distance = 3
    repulsion_distance = 3

    # Define the group center and the repulsion vector
    group_center = get_swarm_center()
    v_rep = np.zeros([2, 1])

    # Compute the average height of all UAVs
    z_cmd = np.mean(
        [
            client.getMultirotorState(
                vehicle_name="UAV" + str(i + 1)
            ).kinematics_estimated.position.z_val
            for i in range(9)
        ]
    )

    # Define the velocity command
    v_cmd = np.zeros([2, 9])

    # Main loop to control UAVs
    for t in range(500):  # assuming 500 time steps are enough to form the grid
        for i in range(9):
            # Define the name and position of the current UAV
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)

            # Calculate the formation point for the current UAV in the grid
            row = i // grid_size
            col = i % grid_size
            formation_point = (
                group_center
                + cell_distance * (col - grid_size // 2) * np.array([[1], [0]])
                + cell_distance * (row - grid_size // 2) * np.array([[0], [1]])
            )

            # Compute the desired velocity for the current UAV
            v_mig = k_mig * (formation_point - pos_i)

            # Perform collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    # Define the name and position of the other UAV
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)

                    # Compute the repulsion vector if the other UAV is too close
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        repulsion_vector = pos_i - pos_j
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        if distance < repulsion_distance:
                            v_rep *= 2

            # Store the computed velocity command for the current UAV
            v_cmd[:, i : i + 1] = v_mig + v_rep

            # Move the UAV using the computed velocity command
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )

    # Reverse the direction of the scan
    for t in range(500):  # assuming 500 time steps are enough to form the grid
        for i in range(9):
            # Define the name and position of the current UAV
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)

            # Calculate the formation point for the current UAV in the grid
            row = i // grid_size
            col = (grid_size - 1) - (i % grid_size)
            formation_point = (
                group_center
                + cell_distance * (col - grid_size // 2) * np.array([[1], [0]])
                + cell_distance * (row - grid_size // 2) * np.array([[0], [1]])
            )

            # Compute the desired velocity for the current UAV
            v_mig = k_mig * (formation_point - pos_i)

            # Perform collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    # Define the name and position of the other UAV
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)

                    # Compute the repulsion vector if the other UAV is too close
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        repulsion_vector = pos_i - pos_j
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        if distance < repulsion_distance:
                            v_rep *= 2

            # Store the computed velocity command for the current UAV
            v_cmd[:, i : i + 1] = v_mig + v_rep

            # Move the UAV using the computed velocity command
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


def form_grid_formation():
    # Set the desired formation parameters
    k_mig = 0.5
    k_rep = 10
    grid_rows = 30  # Number of rows in the grid
    grid_cols = 30  # Number of columns in the grid
    cell_distance = 5  # Distance between grid cells
    safe_distance = 2.5  # Safe distance between UAVs
    repulsion_distance = 1.5  # Distance at which UAVs start repelling each other

    # Compute the average height of all UAVs
    z_cmd = np.mean(
        [
            client.getMultirotorState(
                vehicle_name="UAV" + str(i + 1)
            ).kinematics_estimated.position.z_val
            for i in range(9)
        ]
    )

    # Define the velocity command
    v_cmd = np.zeros([2, 9])

    # Main loop to control UAVs
    for t in range(500):  # Assuming 500 time steps are enough to form the grid
        for i in range(9):
            # Calculate the row and column of the current UAV in the grid
            row = i // grid_rows
            col = i % grid_cols

            # Calculate the formation point for the current UAV
            formation_point = np.array([[col], [row]]) * cell_distance

            # Calculate the position of the current UAV
            pos_i = get_UAV_pos(client, vehicle_name="UAV" + str(i + 1))

            # Compute the desired velocity for the current UAV
            v_mig = k_mig * (formation_point - pos_i)

            # Perform collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    # Calculate the position of the other UAV
                    pos_j = get_UAV_pos(client, vehicle_name="UAV" + str(j + 1))

                    # Compute the repulsion vector if the other UAV is too close
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        repulsion_vector = pos_i - pos_j
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        if distance < repulsion_distance:
                            v_rep *= 2

            # Store the computed velocity command for the current UAV
            v_cmd[:, i : i + 1] = v_mig + v_rep

            # Move the UAV using the computed velocity command
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name="UAV" + str(i + 1)
            )


def form_slanted_line_formation():
    # Set the desired formation parameters
    k_mig = 1
    k_rep = 10
    line_length = 9  # Number of UAVs in the line formation
    line_spacing = 8  # Spacing between UAVs in the line formation
    safe_distance = 5  # Safe distance between UAVs
    repulsion_distance = 5  # Distance at which UAVs start repelling each other

    # Compute the average height of all UAVs
    z_cmd = np.mean(
        [
            client.getMultirotorState(
                vehicle_name="UAV" + str(i + 1)
            ).kinematics_estimated.position.z_val
            for i in range(9)
        ]
    )

    # Define the velocity command
    v_cmd = np.zeros([2, 9])

    # Main loop to control UAVs
    for t in range(500):  # Assuming 500 time steps are enough to form the line
        for i in range(9):
            # Calculate the position of the current UAV in the slanted line formation
            pos_i = np.array([[i * line_spacing], [i]])

            # Compute the desired velocity for the current UAV
            v_mig = k_mig * (
                pos_i - get_UAV_pos(client, vehicle_name="UAV" + str(i + 1))
            )

            # Perform collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    # Calculate the position of the other UAV
                    pos_j = get_UAV_pos(client, vehicle_name="UAV" + str(j + 1))

                    # Compute the repulsion vector if the other UAV is too close
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        repulsion_vector = pos_i - pos_j
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        if distance < repulsion_distance:
                            v_rep *= 2

            # Store the computed velocity command for the current UAV
            v_cmd[:, i : i + 1] = v_mig + v_rep

            # Move the UAV using the computed velocity command
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name="UAV" + str(i + 1)
            )


def left_to_right_scan():
    # Set the desired formation parameters
    k_mig = 1
    k_rep = 10
    scan_speed = 5  # Speed at which the scan will be performed
    scan_distance = 60  # Distance covered by the scan

    # Compute the average height of all UAVs
    z_cmd = np.mean(
        [
            client.getMultirotorState(
                vehicle_name="UAV" + str(i + 1)
            ).kinematics_estimated.position.z_val
            for i in range(9)
        ]
    )

    # Define the velocity command
    v_cmd = np.zeros([2, 9])

    # Main loop to control UAVs
    for t in range(500):  # Assuming 500 time steps are enough to complete the scan
        for i in range(9):
            # Calculate the position of the current UAV
            pos_i = get_UAV_pos(client, vehicle_name="UAV" + str(i + 1))

            # Calculate the desired position for the current UAV along the scan line
            desired_pos = np.array([[scan_distance * t / 500], [pos_i[1, 0]]])

            # Compute the desired velocity for the current UAV
            v_mig = k_mig * (desired_pos - pos_i)

            # Perform collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    # Calculate the position of the other UAV
                    pos_j = get_UAV_pos(client, vehicle_name="UAV" + str(j + 1))

                    # Compute the repulsion vector if the other UAV is too close
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < scan_distance:
                        repulsion_vector = pos_i - pos_j
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (scan_distance - distance)
                        )

            # Store the computed velocity command for the current UAV
            v_cmd[:, i : i + 1] = v_mig + v_rep

            # Move the UAV using the computed velocity command
            client.moveByVelocityAsync(
                v_cmd[0, i],
                v_cmd[1, i],
                z_cmd,
                scan_speed,
                vehicle_name="UAV" + str(i + 1),
            )


def line_scan():
    # Set the desired formation parameters
    k_mig = 0.5
    k_rep = 10
    grid_size = 30
    cell_distance = 30
    safe_distance = 3
    repulsion_distance = 3

    # Define the group center and the repulsion vector
    group_center = get_swarm_center()
    v_rep = np.zeros([2, 1])

    # Compute the average height of all UAVs
    z_cmd = np.mean(
        [
            client.getMultirotorState(
                vehicle_name="UAV" + str(i + 1)
            ).kinematics_estimated.position.z_val
            for i in range(9)
        ]
    )

    # Define the velocity command
    v_cmd = np.zeros([2, 9])

    # Main loop to control UAVs
    for t in range(500):  # assuming 500 time steps are enough to form the grid
        for i in range(9):
            # Define the name and position of the current UAV
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)

            # Calculate the formation point for the current UAV in the grid
            row = i // grid_size
            col = i % grid_size
            formation_point = (
                group_center
                + cell_distance * (col - grid_size // 2) * np.array([[1], [0]])
                + cell_distance * (row - grid_size // 2) * np.array([[0], [1]])
            )

            # Compute the desired velocity for the current UAV
            v_mig = k_mig * (formation_point - pos_i)

            # Perform collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    # Define the name and position of the other UAV
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)

                    # Compute the repulsion vector if the other UAV is too close
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        repulsion_vector = pos_i - pos_j
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        if distance < repulsion_distance:
                            v_rep *= 2

            # Store the computed velocity command for the current UAV
            v_cmd[:, i : i + 1] = v_mig + v_rep

            # Move the UAV using the computed velocity command
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )

    # Reverse the direction of the scan
    for t in range(500):  # assuming 500 time steps are enough to form the grid
        for i in range(9):
            # Define the name and position of the current UAV
            name_i = "UAV" + str(i + 1)
            pos_i = get_UAV_pos(client, vehicle_name=name_i)

            # Calculate the formation point for the current UAV in the grid
            row = i // grid_size
            col = (grid_size - 1) - (i % grid_size)
            formation_point = (
                group_center
                + cell_distance * (col - grid_size // 2) * np.array([[1], [0]])
                + cell_distance * (row - grid_size // 2) * np.array([[0], [1]])
            )

            # Compute the desired velocity for the current UAV
            v_mig = k_mig * (formation_point - pos_i)

            # Perform collision avoidance
            v_rep = np.zeros([2, 1])
            for j in range(9):
                if j != i:
                    # Define the name and position of the other UAV
                    name_j = "UAV" + str(j + 1)
                    pos_j = get_UAV_pos(client, vehicle_name=name_j)

                    # Compute the repulsion vector if the other UAV is too close
                    distance = np.linalg.norm(pos_j - pos_i)
                    if distance < safe_distance:
                        repulsion_vector = pos_i - pos_j
                        v_rep += (
                            k_rep
                            * (repulsion_vector / np.linalg.norm(repulsion_vector))
                            * (safe_distance - distance)
                        )
                        if distance < repulsion_distance:
                            v_rep *= 2
            v_cmd[:, i : i + 1] = v_mig + v_rep
            client.moveByVelocityZAsync(
                v_cmd[0, i], v_cmd[1, i], z_cmd, 0.1, vehicle_name=name_i
            )


if __name__ == "__main__":
    while True:
        command = input("Please input the command to control swarm: ")
        if command == "start":
            take_off()
        elif command == "merge":
            merge()
        elif command == "ls":
            line_scan()
        elif command == "spread":
            spread()
        elif command == "left":
            left()
        elif command == "right":
            right()
        elif command == "up":
            up()
        elif command == "down":
            down()
        elif command == "forward":
            forward()
        elif command == "backward":
            backward()
        elif command == "circle_rep":
            fly_circle()
        elif command == "t":
            test()
        elif command == "t2":
            test2()
        elif command == "circle_move":
            circle_move()
        elif command == "o":
            test_obs()
        elif command == "k":
            circle_move_with_obstacles()
        elif command == "c":
            cover_block()
        elif command == "s":
            spiral_motion()
        elif command == "target":
            target()
        elif command == "chase":
            chasing()
        elif command == "tc":
            target_and_chasing()
        elif command == "fcc":
            form_circle()
        elif command == "fcm":
            circle_move_2()
        elif command == "grid":
            form_grid_formation()
        elif command == "line":
            form_slanted_line_formation()
        elif command == "scan":
            left_to_right_scan()
        elif command == "exit":
            client.reset()

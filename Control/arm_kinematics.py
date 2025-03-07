#!/usr/bin/env python3
import numpy as np
import rospy
from centering_test import get_joint_angles
from arm_control import move_arm
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# This script also only moves joints robot_arm_shoulder_lift_joint, robot_arm_elbow_joint, robot_arm_wrist_1_joint.
# Those joints name correspond respectively to joint number 2, 3, 4. with index starting at 1, as on the file on teams.

# Arm segment lengths and camera offset
r1 = 0.4784
r2 = 0.360
r3 = 0.1  # Camera length placeholder


def direct_kin(theta1, theta2, wrist_angle=None):
    """
    Forward kinematics for the arm using a standard 2R formulation.
    theta1: angle of the first joint.
    theta2: relative angle of the second joint (added to theta1).

    The wrist (camera holder) joint is adjusted so that the camera stays
    parallel to the ground (i.e. level) at all times. If wrist_angle is not
    provided, it is computed automatically as - (theta1+theta2), so that:

         effective_angle = theta1 + theta2 + wrist_angle = 0.

    The camera is then placed at the end effector plus an offset along x.
    Returns the position in cartesian space (x,y,z) of the base, shoulder,
    elbow, end_effector and camera.
    """
    # Compute the positions of the elbow and the end effector (planar: x-z)
    elbow = np.array([r1 * np.cos(theta1),
                      0,
                      r1 * np.sin(theta1)])
    end_eff = np.array([r1 * np.cos(theta1) + r2 * np.cos(theta1 + theta2),
                        0,
                        r1 * np.sin(theta1) + r2 * np.sin(theta1 + theta2)])

    # Compute wrist angle if not provided (to keep the camera level)
    if wrist_angle is None:
        wrist_angle = -(theta1 + theta2)
    print("[DIRECT_KIN] Wrist angle: {}".format(wrist_angle))
    # With the wrist joint, the effective angle for the camera offset is:
    effective_angle = theta1 + theta2 + wrist_angle  # This should be 0.
    # The camera offset now is applied in a fixed horizontal direction.
    camera_offset = np.array([r3 * np.cos(effective_angle),
                              0,
                              r3 * np.sin(effective_angle)])
    # Given effective_angle is zero, camera_offset becomes [r3, 0, 0].
    camera_end = end_eff + camera_offset

    # Base and shoulder positions (as before)
    base = np.array([0, 0, -0.1])
    shoulder = np.array([0, 0, 0])

    points = np.vstack((base, shoulder, elbow, end_eff, camera_end))
    return points


def inverse_kin(target_z, current_angles):
    """
    Compute new joint angles (theta1, theta2, theta3) such that the end effector
    (from the 2R arm) has:
      - a z-coordinate equal to target_z, and
      - an x-coordinate as close as possible to the current x.

    current_angles: tuple (theta1_current, theta2_current) [theta2 and theta3 are relative].
    The camera wrist is assumed to adjust automatically to keep the camera level.
    """
    theta1_current, theta2_current = current_angles

    # Get current end-effector x coordinate from forward kinematics.
    pos_current = direct_kin(theta1_current, theta2_current)
    current_x = pos_current[3, 0]

    # For the desired (x, z) position, the distance from the shoulder is:
    # d = sqrt(x^2 + target_z^2)
    # For a 2R arm, d must lie in [|r1 - r2|, r1 + r2].
    d_min = abs(r1 - r2)
    d_max = r1 + r2

    # Try to use the current x; if not reachable, clamp it.
    d_current = np.sqrt(current_x ** 2 + target_z ** 2)
    if d_current < d_min:
        x_candidate = np.sqrt(max(d_min ** 2 - target_z ** 2, 0))
        x_new = np.copysign(x_candidate, current_x)
    elif d_current > d_max:
        x_candidate = np.sqrt(d_max ** 2 - target_z ** 2)
        x_new = np.copysign(x_candidate, current_x)
    else:
        x_new = current_x

    d_new = np.sqrt(x_new ** 2 + target_z ** 2)
    if d_new < d_min or d_new > d_max:
        print("[INVERSE_KIN] ERROR: Target is unreachable even after clamping x.")
        return None

    # --- Standard 2R Inverse Kinematics ---
    # Two candidate solutions for theta2:
    cos_theta2 = (d_new ** 2 - r1 ** 2 - r2 ** 2) / (2 * r1 * r2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2_sol1 = np.arccos(cos_theta2)
    theta2_sol2 = -theta2_sol1

    def compute_theta1(theta2_candidate):
        return np.arctan2(target_z, x_new) - np.arctan2(r2 * np.sin(theta2_candidate),
                                                        r1 + r2 * np.cos(theta2_candidate))

    sol1 = (compute_theta1(theta2_sol1), -theta2_sol1)
    sol2 = (compute_theta1(theta2_sol2), -theta2_sol2)

    # The shoulder joint should never be positive.
    # -pi < shoulder_angle  < 0
    if sol1[0] > 0 or sol1[0] < -np.pi:
        sol1 = None

    if sol2[0] > 0 or sol2[0] < -np.pi:
        sol2 = None

    # Choose the candidate that minimizes the total joint change.
    try:
        diff1 = abs(sol1[0] - theta1_current) + abs(sol1[1] - theta2_current)
    except:
        diff1 = 1e5
        print("[INVERSE_KIN] WARNING: Solution 1 is not safe.")

    try:
        diff2 = abs(sol2[0] - theta1_current) + abs(sol2[1] - theta2_current)
    except:
        diff2 = 1e5
        print("[INVERSE_KIN] WARNING: Solution 2 is not safe.")

    # Make the selection based on the difference in joints
    chosen_sol = sol1 if diff1 < diff2 else sol2

    if chosen_sol is not None:
        wrist_angle = -(chosen_sol[0] + chosen_sol[1]) - np.pi
        chosen_sol = chosen_sol + (wrist_angle,)  # Append wrist_angle as a new tuple
        print("[INVERSE_KIN] Desired x =", x_new, "for target z =", target_z)
        print("[INVERSE_KIN] Chosen sol: ", chosen_sol)
        return chosen_sol[0], chosen_sol[1], chosen_sol[2]
    else:
        print(
            "[INVERSE_KIN] ERROR: no solution could be found. Solution would result in positive shoulder joint angle.")
        return None


if __name__ == "__main__":
    rospy.init_node("image_trigger_script", anonymous=True)
    move_arm_pub = rospy.Publisher('/robot/arm/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)
    # y_distance = np.array([find_center(img)[1]])
    centering_y_accuracy = 50  # pixels

    # Calculate angles for first movement
    old_angles = get_joint_angles()
    print(old_angles)
    # old_angles[0] = old_angles[0]-np.pi/6
    # old_angles[3] = old_angles[3]+np.pi/6
    # old_angles[2] = np.pi
    # old_angles[4] = old_angles[4]+np.pi/2
    angle_tuple = (old_angles[1], old_angles[0])
    print(angle_tuple)
    delta_z = 0.05  # change this to first step size
    pos_current = direct_kin(old_angles[1], old_angles[0])
    # print(pos_current)
    current_end_z = pos_current[3, 2]
    z_target = current_end_z + delta_z
    new_angles = old_angles
    new_angles[1], new_angles[0], new_angles[3] = inverse_kin(z_target, angle_tuple)
    new_angles[0] = -new_angles[0]
    print(new_angles)
    move_arm(new_angles, move_arm_pub)
    # move_arm(old_angles, move_arm_pub)
    # # Loop to center image
    # while np.abs(y_distance[-1] > centering_y_accuracy):
    #     move_arm(new_angles, move_arm_pub)
    #     rospy.sleep(5)

    #     trigger_image_capture(image_flag)
    #     rospy.sleep(1)
    #     img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
    #     image_count += 1

    #     y_distance = np.append(y_distance, find_center(img)[1])
    #     delta_z = -np.abs((delta_z/(y_distance[-1]-y_distance[-2])* y_distance[-1])) # have to look at direction of this
    #     old_angles = get_joint_angles()
    #     angle_tuple = (old_angles[1], old_angles[0])
    #     pos_current = direct_kin(old_angles[1], old_angles[0])
    #     current_end_z = pos_current[3, 2]
    #     z_target = current_end_z + delta_z
    #     new_angles = old_angles
    #     new_angles[1], new_angles[0], new_angles[3] = inverse_kin(z_target, angle_tuple)

    # theta1_current = 1/3 * np.pi  
    # theta2_current = 1/6 * np.pi   # theta2 is the relative angle.
    # current_angles = (theta1_current, theta2_current)

    # # Compute the current end-effector position.
    # pos_current = direct_kin(theta1_current, theta2_current)
    # current_end_x = pos_current[3, 0]
    # current_end_z = pos_current[3, 2]
    # print("[ARM KINEMATICS] Current end effector position: x =", current_end_x, ", z =", current_end_z)

    # # Now suppose you want to raise the end effector by 5 cm in z.
    # target_z = current_end_z - 0.5

    # new_angles = inverse_kin(target_z, current_angles)
    # if new_angles is not None:
    #     new_theta1, new_theta2, wrist_angle = new_angles
    #     # The wrist joint is set automatically so that the camera remains level.
    #     pos_new = direct_kin(new_theta1, new_theta2, wrist_angle)
    #     new_end_x = pos_new[3, 0]
    #     new_end_z = pos_new[3, 2]
    #     print("[ARM KINEMATICS] New joint angles:")
    #     print("[ARM KINEMATICS]  theta1 =", new_theta1)
    #     print("[ARM KINEMATICS]  theta2 =", new_theta2)
    #     print("[ARM KINEMATICS] New end effector position: x =", new_end_x, ", z =", new_end_z)
    # else:
    #     print("[ARM KINEMATICS] Could not find a valid solution.")

    # ########################
    # ## PLOTTING
    # ########################
    # # Plot original configuration on the left and new configuration on the right.
    # plotting = True
    # if plotting == True:
    #     import matplotlib.pyplot as plt
    #     fig = plt.figure(figsize=(10, 6))

    #     # Original configuration.
    #     ax1 = fig.add_subplot(121, projection='3d')
    #     ax1.plot(pos_current[:, 0], pos_current[:, 1], pos_current[:, 2],
    #             marker='o', linestyle='-', color='b')
    #     ax1.scatter(*pos_current[1, :], color="k", s=100, label="Shoulder joint")
    #     ax1.scatter(*pos_current[2, :], color="g", s=100, label="Elbow joint")
    #     ax1.scatter(*pos_current[3, :], color="r", s=100, label="End Effector")
    #     ax1.set_xlim([-1, 1])
    #     ax1.set_ylim([-0.1, 0.1])
    #     ax1.set_zlim([-0.1, 1])
    #     ax1.set_xlabel("X [m]")
    #     ax1.set_ylabel("Y [m]")
    #     ax1.set_zlabel("Z [m]")
    #     ax1.set_title("Original Configuration")
    #     ax1.legend()
    #     ax1.set_aspect('equal', 'box')

    #     # New configuration.
    #     ax2 = fig.add_subplot(122, projection='3d')
    #     ax2.plot(pos_new[:, 0], pos_new[:, 1], pos_new[:, 2],
    #             marker='o', linestyle='-', color='b')
    #     ax2.scatter(*pos_new[1, :], color="k", s=100, label="Shoulder joint")
    #     ax2.scatter(*pos_new[2, :], color="g", s=100, label="Elbow joint")
    #     ax2.scatter(*pos_new[3, :], color="r", s=100, label="End Effector")
    #     ax2.set_xlim([-1, 1])
    #     ax2.set_ylim([-0.1, 0.1])
    #     ax2.set_zlim([-0.1, 1])
    #     ax2.set_xlabel("X [m]")
    #     ax2.set_ylabel("Y [m]")
    #     ax2.set_zlabel("Z [m]")
    #     ax2.set_title("New Configuration")
    #     ax2.legend()
    #     ax2.set_aspect('equal', 'box')

    #     plt.show()

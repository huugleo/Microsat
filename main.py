#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Bool
import cv2
from robotnik_navigation_msgs.msg import MoveActionGoal, MoveGoal
from sensor_msgs.msg import JointState

from robot_sweep_control import rotate_base, arrive_to_start, return_to_light
from find_center import find_center
from white_pixels import white_pixels
from approach import move_base_forward, arrival_test
from get_image import trigger_image_capture
from arm_control_test_2 import direct_kin, inverse_kin
from arm_control import move_arm

# We should better import the function from another file to make main look cleaner
def get_joint_angles():
    """
    Retrieves the current joint angles whenever this function is called
    """
    msg = rospy.wait_for_message("/joint_states", JointState)

    return msg.position[1], msg.position[2], msg.position[3]


if __name__ == "__main__":
    rospy.init_node("image_trigger_script", anonymous=True)  # Initialize node once
    image_flag = rospy.Publisher("/image_flag", Bool, queue_size=10)
    move_robot = rospy.Publisher('/robot/move/goal', MoveActionGoal, queue_size=10)
    rospy.sleep(1)  # Ensure publisher is registered
    image_count = 1

    sweep_range = 100 / 180 * np.pi
    incremental_angle = 15 / 180 * np.pi
    i = int(sweep_range / incremental_angle)
    arrive_to_start(sweep_range, move_robot)

    white_pixels_array = np.zeros(i)

    for j in range(i):  # Run until node is shutdown
        trigger_image_capture(image_flag)  # Call function to publish message
        rospy.sleep(1)  # Delay to avoid overwhelming the system
        img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
        white_pixels_array[j] = white_pixels(img)
        image_count = image_count + 1
        rotate_base(incremental_angle, move_robot)
        rospy.sleep(3)
        rospy.loginfo(f"Moving step number {j + 1} out of {i}")

    return_to_light(white_pixels_array, incremental_angle, move_robot)
    rospy.sleep(10)
    rospy.loginfo("Image trigger script shutting down.")

    # Center on target
    trigger_image_capture(image_flag)  # Call function to publish message
    rospy.sleep(1)  # Delay to avoid overwhelming the system
    img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
    image_count += 1
    x_distance = np.array([find_center(img)[0]])
    centering_x_accuracy = 50  # pixels
    fine_rotation = -np.sign(x_distance[-1]) * incremental_angle / 20

    while np.abs(x_distance[-1]) > centering_x_accuracy:
        print(f"fine_rotation={fine_rotation}")
        rotate_base(fine_rotation, move_robot)
        rospy.loginfo(f"Moved by fine rotation")
        rospy.sleep(1)
        trigger_image_capture(image_flag)  # Call function to publish message
        rospy.sleep(1)  # Delay to avoid overwhelming the system
        img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
        image_count += 1
        x_distance = np.append(x_distance, find_center(img)[0])
        fine_rotation = -np.abs((fine_rotation / (x_distance[-1] - x_distance[-2]))) * x_distance[-1]
    
    # Capture image
    trigger_image_capture(image_flag)
    rospy.sleep(1)
    img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
    image_count += 1
    
    # Find y_distance
    y_distance = np.array([find_center(img)[1]])
    centering_y_accuracy = 50  # pixels

    # Calculate angles for first movement
    old_angles = get_joint_angles()
    angle_tuple = (old_angles[1], old_angles[0])
    delta_z = 0.1 # change this to first step size
    pos_current = direct_kin(old_angles[1], old_angles[0])
    current_end_z = pos_current[3, 2]
    z_target = current_end_z + delta_z
    new_angles = old_angles
    new_angles[1], new_angles[0], new_angles[3] = inverse_kin(z_target, angle_tuple)

    # Loop to center image
    while np.abs(y_distance[-1] > centering_y_accuracy):
        move_arm(new_angles, move_arm_pub)
        rospy.sleep(5)
        
        trigger_image_capture(image_flag)
        rospy.sleep(1)
        img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
        image_count += 1

        y_distance = np.append(y_distance, find_center(img)[1])
        delta_z = -np.abs((delta_z/(y_distance[-1]-y_distance[-2])* y_distance[-1])) # have to look at direction of this
        old_angles = get_joint_angles()
        angle_tuple = (old_angles[1], old_angles[0])
        pos_current = direct_kin(old_angles[1], old_angles[0])
        current_end_z = pos_current[3, 2]
        z_target = current_end_z + delta_z
        new_angles = old_angles
        new_angles[1], new_angles[0], new_angles[3] = inverse_kin(z_target, angle_tuple)
        

    rospy.loginfo("Centered on target.")
    trigger_image_capture(image_flag)  # Call function to publish message
    rospy.sleep(1)  # Delay to avoid overwhelming the system
    while not arrival_test(cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg"), 0.5):
        rospy.sleep(1)
        trigger_image_capture(image_flag)
        rospy.sleep(1)
        image_count += 1
        move_base_forward(0.1, move_robot) # Change 0.1 with a variable step eventually

    rospy.loginfo("Arrived at target.")



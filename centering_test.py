#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import cv2
from white_pixels import white_pixels
from robotnik_navigation_msgs.msg import MoveActionGoal, MoveGoal
import numpy as np
from robot_sweep_control import step_pixels, arrive_to_start, return_to_light
from find_center import find_center
from sensor_msgs.msg import JointState
from arm_control import move_arm
from arm_kinematics import inverse_kin
from trajectory_msgs.msg import JointTrajectory
from get_arm_state import get_joint_angles
from approach import *

def trigger_image_capture(pub):
    """
    Publishes a Bool message with a value of True to /image_flag to signal that a new image should be captured or processed by another node.
    """

    rospy.loginfo("Triggering image capture...")
    pub.publish(True)
    rospy.loginfo("Image capture request sent.")


if __name__ == "__main__":
    rospy.init_node("image_trigger_script", anonymous=True)  # Initialize node once
    image_flag = rospy.Publisher("/image_flag", Bool, queue_size=10)
    move_robot =  rospy.Publisher('/robot/move/goal', MoveActionGoal, queue_size=10)
    rospy.sleep(1)  # Ensure publisher is registered
    image_count = 1

    sweep_range = 90/180 * np.pi
    incremental_angle = 15/180 * np.pi
    i = int(sweep_range/incremental_angle)
    arrive_to_start(sweep_range, move_robot)
    
    white_pixels_array = np.zeros(i)

    for j in range(i):  # Run until node is shutdown
        trigger_image_capture(image_flag)  # Call function to publish message
        rospy.sleep(1)  # Delay to avoid overwhelming the system
        img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
        white_pixels_array[j] =  white_pixels(img, image_count)
        image_count = image_count+1
        step_pixels(incremental_angle, move_robot)
        rospy.sleep(3)
        rospy.loginfo(f"Moving step number {j+1} out of {i}")
    return_to_light(white_pixels_array, incremental_angle, move_robot)
    rospy.sleep(10)
    trigger_image_capture(image_flag)
    rospy.sleep(2)
    img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
    print(image_count)
    # image_count +=1
    rospy.loginfo("Image trigger script shutting down.")
    while not arrival_test(img, 0.5, image_count):
        # Center on target
        trigger_image_capture(image_flag)   # Call function to publish message
        rospy.sleep(1)                      # Delay to avoid overwhelming the system
        img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
        image_count +=1
        x_distance = np.array([find_center(img, image_count)[0]])
        print(f"x distance = {x_distance[-1]}")
        centering_x_accuracy = 50           # pixels
        fine_rotation = -np.sign(x_distance[-1]) * incremental_angle/20

        while np.abs(x_distance[-1]) > centering_x_accuracy:
            print(f"fine_rotation={fine_rotation}")
            step_pixels(fine_rotation, move_robot)
            rospy.loginfo(f"Moved by fine rotation")
            rospy.sleep(1)
            trigger_image_capture(image_flag)   # Call function to publish message
            rospy.sleep(1)                      # Delay to avoid overwhelming the system
            img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
            image_count +=1
            x_distance = np.append(x_distance, find_center(img, image_count)[0])
            print(f"x distance = {x_distance[-1]}")
            fine_rotation = -np.abs((fine_rotation/(x_distance[-1] - x_distance[-2])))*x_distance[-1]
        
        user_input = input("Do you want to continue? (y/n): ").strip().lower()

        if user_input == 'y':
            print("Continuing with the code...")
            # Capture image
            trigger_image_capture(image_flag)
            rospy.sleep(1)
            img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
            image_count += 1
            
            move_arm_pub = rospy.Publisher('/robot/arm/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)
            rospy.sleep(2)

            # Find y_distance
            y_distance = np.array([find_center(img, image_count)[1]])
            centering_y_accuracy = 50  # pixels
            print(f"y distance = {y_distance[-1]}")
            # Calculate angles for first movement
            old_angles = get_joint_angles()
            angle_tuple = (old_angles[1], old_angles[0], old_angles[3])
            delta_z = np.sign(y_distance) * 0.05 # change this to first step size
            new_angles = old_angles
            new_angles[1], new_angles[0], new_angles[3] = inverse_kin(delta_z, angle_tuple)

            # Loop to center image
            while np.abs(y_distance[-1]) > centering_y_accuracy:
                move_arm(new_angles, move_arm_pub)
                trigger_image_capture(image_flag)
                rospy.sleep(1)
                img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
                image_count += 1

                y_distance = np.append(y_distance, find_center(img, image_count)[1])
                print(f"y distance = {y_distance[-1]}")
                if y_distance[-1] > 0:
                    delta_z = np.abs((delta_z/(y_distance[-1]-y_distance[-2])* y_distance[-1])) # have to look at direction of this
                else:
                    delta_z = -np.abs((delta_z/(y_distance[-1]-y_distance[-2])* y_distance[-1]))
                old_angles = get_joint_angles()
                angle_tuple = (old_angles[1], old_angles[0], old_angles[3])
                new_angles = old_angles
                new_angles[1], new_angles[0], new_angles[3] = inverse_kin(delta_z, angle_tuple)
        
        else:
            print("Exiting.")

        rospy.loginfo("Centered on target.")
        move_base_forward(-0.075, move_robot) # Change 0.1 with a variable step eventually
        trigger_image_capture(image_flag)
        rospy.sleep(2)
        img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
        print(image_count)
        
    rospy.loginfo("Arrived at target.")
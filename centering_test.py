#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import cv2
from white_pixels import white_pixels
from robotnik_navigation_msgs.msg import MoveActionGoal, MoveGoal
import numpy as np
from robot_sweep_control import rotate_base, arrive_to_start, return_to_light
from find_center import find_center
from sensor_msgs.msg import JointState

def trigger_image_capture(pub):
    """
    Publishes a Bool message with a value of True to /image_flag to signal that a new image should be captured or processed by another node.
    """

    rospy.loginfo("Triggering image capture...")
    pub.publish(True)
    rospy.loginfo("Image capture request sent.")

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

    
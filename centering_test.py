#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import cv2
from Image_processing.white_pixels import white_pixels
from robotnik_navigation_msgs.msg import MoveActionGoal, MoveGoal
import numpy as np
from robot_sweep_control import step_pixels, arrive_to_start, return_to_light

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

    sweep_range = np.pi
    incremental_angle = np.pi/10
    i = int(sweep_range/incremental_angle)
    arrive_to_start(sweep_range, move_robot)
    white_pixels_array = np.zeros(i)

    for j in range(i):  # Run until node is shutdown
        trigger_image_capture(image_flag)  # Call function to publish message
        rospy.sleep(1)  # Delay to avoid overwhelming the system
        img = cv2.imread(f"/home/thomasvkuik/catkin_ws/src/microsat_camera/src/images/target_{image_count}.jpg")
        white_pixels_array[j] =  white_pixels(img)
        image_count = image_count+1
        step_pixels(incremental_angle, move_robot)
        rospy.loginfo(f"Moving step number {j+1} out of {i}")

    return_to_light(white_pixels_array, incremental_angle, move_robot)

    rospy.loginfo("Image trigger script shutting down.")

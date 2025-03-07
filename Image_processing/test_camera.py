#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import cv2
from robotnik_navigation_msgs.msg import MoveActionGoal, MoveGoal

# Try "roslaunch galaxy_camera MER-139.launch"

def trigger_image_capture(pub):
    """Publishes a message to trigger image capture."""
    rospy.loginfo("Triggering image capture...")
    pub.publish(True)
    rospy.loginfo("Image capture request sent.")

if __name__ == "__main__":
    rospy.init_node("image_trigger_script", anonymous=True)  # Initialize node once
    image_flag = rospy.Publisher("/image_flag", Bool, queue_size=10)
    move_robot =  rospy.Publisher('/robot/move/goal', MoveActionGoal, queue_size=10)
    rospy.sleep(1)  # Ensure publisher is registered
    image_count = 10
    i=1
    while i > 0:  # Run until node is shutdown
        rospy.sleep(1)
        trigger_image_capture(image_flag)  # Call function to publish message
        rospy.sleep(1)  # Delay to avoid overwhelming the system
        img = cv2.imread(f"/home/robot/catkin_ws/src/microsat_group_2/src/images/target_{image_count}.jpg")
        image_count = image_count+1
        i= i-1

    rospy.loginfo("Image trigger script shutting down.")

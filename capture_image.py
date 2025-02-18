#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import cv2
from white_pixels import white_pixels

def trigger_image_capture(pub):
    """Publishes a message to trigger image capture."""
    rospy.loginfo("Triggering image capture...")
    pub.publish(True)
    rospy.loginfo("Image capture request sent.")

if __name__ == "__main__":
    rospy.init_node("image_trigger_script", anonymous=True)  # Initialize node once
    pub = rospy.Publisher("/image_flag", Bool, queue_size=10)
    i=5
    rospy.sleep(1)  # Ensure publisher is registered
    image_count = 1
    while i > 0:  # Run until node is shutdown
        trigger_image_capture(pub)  # Call function to publish message
        rospy.sleep(1)  # Delay to avoid overwhelming the system
        img = cv2.imread(f"/home/thomasvkuik/catkin_ws/src/microsat_camera/src/images/target_{image_count}.jpg")
        white_pixels(img)
        image_count = image_count+1
        i= i-1

    rospy.loginfo("Image trigger script shutting down.")

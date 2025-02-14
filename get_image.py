import rospy  # Import ROS Python library for handling nodes and topics
import cv2  # OpenCV for image processing
from cv_bridge import CvBridge, CvBridgeError  # Converts ROS images to OpenCV format
from sensor_msgs.msg import Image  # Message type for images in ROS
from std_msgs.msg import Bool  # Message type for Boolean values in ROS
import os  # Library for handling file paths and directories



# Initialize the ROS node named "camera"
rospy.init_node("camera", anonymous=True)

# Subscribe to the image topic to receive images from the camera
rospy.Subscriber("/galaxy_camera/galaxy_camera/image_raw", Image, image_callback)

# Subscribe to the flag topic to control image processing state
rospy.Subscriber("image_flag", Bool, flag_listener_callback)

rospy.loginfo("Image acquisition node started.")

# Keep the node running and continuously listen for messages
rospy.spin()
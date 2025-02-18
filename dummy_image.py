#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

def dummy_image_publisher():
    # Create the ROS node
    rospy.init_node('dummy_image_publisher', anonymous=True)
    
    # Create a publisher for the /galaxy_camera/galaxy_camera/image_raw topic
    pub = rospy.Publisher('/galaxy_camera/galaxy_camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    rospy.loginfo("Publishing dummy image...")

    # Publish the dummy image at a rate of 10 Hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dummy_image = np.zeros((480, 640), dtype=np.uint8)
        # Get image dimensions
        height, width= dummy_image.shape
        num_pixels = np.random.randint(100, 1000)
        # Generate random coordinates
        y_coords = np.random.randint(0, height, num_pixels)
        x_coords = np.random.randint(0, width, num_pixels)
        dummy_image[y_coords, x_coords] = [255]
        # Create an image message
        image_msg = bridge.cv2_to_imgmsg(dummy_image, encoding="mono8")

        # Add a header (optional but recommended)
        image_msg.header = Header()
        image_msg.header.stamp = rospy.Time.now()

        pub.publish(image_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        dummy_image_publisher()
    except rospy.ROSInterruptException:
        pass

from base_controller_example import publish_move_goal
from white_pixels import white_pixels
import rospy
import numpy as np

# One function for moving the base forward (x-direction)
def move_base_forward(movement_x, pub):
    # Move by incremental angle around its current position
    publish_move_goal({"goal": [movement_x, 0.0, 0.0],
                       "max_lin": [0.5, 0.5, 0.5],
                       "max_ang": [0.2, 0.2, 0.2]}, pub)

# One base for checking if the robot has arrived at the target
def arrival_test(img, threshold):
    # img is derived using cv2.imread from the image file
    white_pixels_count = white_pixels(img)
    total_pixels = img.shape[0] * img.shape[1]
    white_percentage = white_pixels_count / total_pixels
    if white_percentage > threshold:
        return True
    else:
        return False


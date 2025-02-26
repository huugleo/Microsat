from base_controller_example import publish_move_goal
import rospy
import numpy as np

"""
Functions call publish_move_goal with different parameters to rotate the robot base by a certain angle.
"""

def step_pixels(incremental_angle, pub):
    # Move by incremental angle around its current position
    publish_move_goal({"goal": [0.0, 0.0, incremental_angle],
                       "max_lin": [0.5, 0.5, 0.5],
                       "max_ang": [0.2, 0.2, 0.2]}, pub)


def arrive_to_start(sweep_range, pub):
    """
    Moves the robot to the starting position and waits until it has stopped before continuing.
    """
    rospy.loginfo("Moving to start position...")

    # Publish move goal
    publish_move_goal({"goal": [0.0, 0.0, -sweep_range/2],
                       "max_lin": [0.5, 0.5, 0.5],
                       "max_ang": [0.2, 0.2, 0.2]}, pub)
    rospy.sleep(10)
    rospy.loginfo("Arrived at start position.")

def return_to_light(white_pixels_count_array, incremental_angle, pub):
    # Find the index of the maximum white pixels
    max_white_pixels_index = np.argmax(white_pixels_count_array)
    # Calculate the angle to return to the maximum white pixels
    return_angle = -incremental_angle * (len(white_pixels_count_array) - max_white_pixels_index)
    # Return to the maximum white pixels
    publish_move_goal({"goal": [0.0, 0.0, return_angle],
                       "max_lin": [0.5, 0.5, 0.5],
                       "max_ang": [0.2, 0.2, 0.2]}, pub)

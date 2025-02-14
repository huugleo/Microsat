from base_controller_example import publish_move_goal
import numpy as np
import find_center

# Center the robot to the target

# Fetch distance, rotate, calculate ratio, determine rotation, rotate until convergence (accuracy input)
def centering(accuracy, rot_initial):
    # Fetch the image
    img = get_image() # Example function to fetch the image
    # Fetch the distance from the center
    x_distance, y_distance = find_center(img)
    # Rotate by the x_distance
    publish_move_goal({"goal": [0.0, 0.0, rot_initial],
                       "max_lin": [0.5, 0.5, 0.5],
                       "max_ang": [0.2, 0.2, 0.2]})
    # Fetch the image
    img = get_image() # Example function to fetch the image
    # Fetch the distance from the center
    x_distance, y_distance = find_center(img)
    # Rotate by the y_distance
    publish_move_goal({"goal": [0.0, 0.0, y_distance],
                       "max_lin": [0.5, 0.5, 0.5],
                       "max_ang": [0.2, 0.2, 0.2]})
    # Fetch the image
    img = get_image() # Example function to fetch the image
    # Fetch the distance from the center
    x_distance, y_distance = find_center(img)
    # Calculate the ratio
    ratio = x_distance/y_distance
    # Determine the rotation
    rotation = np.arctan(ratio)
    # Rotate by the rotation
    publish_move_goal({"goal": [0.0, 0.0, rotation],
                       "max_lin": [0.5, 0.5, 0.5],
                       "max_ang": [0.2, 0.2, 0.2]})
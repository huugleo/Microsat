from base_controller_example import publish_move_goal
import numpy as np
import white_pixels

# Loop around the sweep range
def sweep_initial(sweep_range, sweep_steps):
    # Calculate the incremental angle to sweep
    sweep_increment = sweep_range / sweep_steps
    # Calculate the starting angle
    starting_angle = - sweep_range / 2
    # Arrive at the starting angle
    publish_move_goal({"goal" : [0.0, 0.0, starting_angle],
				"max_lin" : [0.5, 0.5, 0.5],
				"max_ang" : [0.2, 0.2, 0.2]})
    # Create empty array to store the number of white pixels for the corresponding angle
    white_pixels_array_1 = np.arange(starting_angle, starting_angle + sweep_range + sweep_increment, sweep_increment)
    white_pixels_array_2 = np.zeros(sweep_steps)
    white_pixels_array = np.vstack((white_pixels_array_1, white_pixels_array_2))
    # Initial sweep
    i = 0
    while i != sweep_steps:
        # Fetch the image
        img = get_image() # Example function to fetch the image
        # Fetch the number of white pixels
        white_pixels_n = white_pixels(img);
        # Store the number of white pixels
        white_pixels_array[1][i] = white_pixels_n
        # Rotate by sweep increment
        publish_move_goal({"goal": [0.0, 0.0, sweep_increment],
                           "max_lin": [0.5, 0.5, 0.5],
                           "max_ang": [0.2, 0.2, 0.2]})
        # Increment the counter
        i += 1
    # Point to the maximum number of white pixels
    # Obtain maximum number of white pixels index
    max_white_pixels_index = np.argmax(white_pixels_array[1])
    # Fetch the corresponding angle
    max_white_pixels_angle = white_pixels_array[0][max_white_pixels_index]
    # Rotate to the corresponding angle
    publish_move_goal({"goal": [0.0, 0.0, -(white_pixels_array[0][-1]-max_white_pixels_angle)],
                       "max_lin": [0.5, 0.5, 0.5],
                       "max_ang": [0.2, 0.2, 0.2]})
# Center the robot to the target

# Fetch distance, rotate, calculate ratio, determine rotation, rotate until convergence (accuracy input)







import math

def distance_check(target_distance):
    # Check if the target is within a certain distance range.
    max_distance = 2 #m
    if target_distance < max_distance:
        return True
    else:
        return False

def calculate_movement(target_distance, target_center_x, target_center_y):
    # Compute the required orientation (angle_to_move) and distance (distance_to_move) for a robot (or robotic arm) to move from the image center to the target’s position.
    # Fixed values
    distance_scaler = 0.01  # Adjust as needed
    image_resolution = (1440, 1080)  # Adjust as needed

    # Calculate the center of the image in pixel coordinates
    center_x = image_resolution[0] / 2
    center_y = image_resolution[1] / 2

    # Calculate the offset from the image center to the target center
    offset_x = target_center_x - center_x
    offset_y = target_center_y - center_y

    # Calculate the angle and distance to move the robot arm
    angle_to_move = math.atan2(offset_y, offset_x) # in radians
    distance_to_move = math.sqrt(offset_x**2 + offset_y**2) * distance_scaler * target_distance # in pixels

    return angle_to_move, distance_to_move

def main():
    # Example inputs (replace with actual values)
    target_distance = 100  # Replace with the actual distance to the target
    pixel_circumference = [(x1, y1), (x2, y2), (x3, y3)]  # Replace with actual pixel list

    # Call the function to calculate the movement
    if distance_check(target_distance):
        angle, distance = calculate_movement(target_distance, pixel_circumference)

    # Print the results
    print(f"Angle to Move: {math.degrees(angle)} degrees")
    print(f"Distance to Move: {distance} units")

if __name__ == "__main__":
    # Call the main function
    main()
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rtde_control

# Connect to the UR16e
ROBOT_IP = "192.168.1.102"  # Replace with your robot's IP
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)

# UR16e arm segment lengths (meters)
L1 = 0.425  # Upper arm
L2 = 0.392  # Forearm

def compute_joint_positions(initial_position, delta_z):
    """
    Compute the 3D positions of the UR16e robotic arm joints 
    for a vertical movement while keeping the end-effector parallel to the floor.
    
    Parameters:
        initial_position (tuple): (x, y, z) initial position of the end effector.
        delta_z (float): Change in height (positive = up, negative = down).
    
    Returns:
        dict: Contains joint angles and 3D positions of the base, shoulder, elbow, and wrist.
    """
    x, y, initial_z = initial_position
    new_z = initial_z + delta_z  # Apply vertical movement

    # Solve for inverse kinematics
    shoulder_angle = np.arctan2(y, x)  # Base rotation (unchanged)
    r = np.sqrt(x**2 + y**2)  # Radius in XY plane

    # Use cosine rule to compute elbow angle
    d = np.sqrt(r**2 + new_z**2)  # Effective arm length
    elbow_angle = np.arccos((L1**2 + L2**2 - d**2) / (2 * L1 * L2))  # Law of Cosines

    # Compute shoulder pitch angle
    shoulder_pitch = np.arctan2(new_z, r) + np.arccos((L1**2 + d**2 - L2**2) / (2 * L1 * d))

    # Wrist 1 angle to maintain end-effector parallel to the floor
    wrist1_angle = -(shoulder_pitch + elbow_angle)

    # Convert to degrees
    shoulder_angle_deg = np.degrees(shoulder_pitch)
    elbow_angle_deg = np.degrees(elbow_angle)
    wrist1_angle_deg = np.degrees(wrist1_angle)

    # Compute forward kinematics (joint positions)
    base = np.array([0, 0, 0])
    shoulder = np.array([L1 * np.cos(shoulder_pitch), L1 * np.sin(shoulder_pitch), 0])
    elbow = shoulder + np.array([L2 * np.cos(shoulder_pitch + elbow_angle), 
                                 L2 * np.sin(shoulder_pitch + elbow_angle), 
                                 0])
    wrist = np.array([x, y, new_z])  # End effector final position

    return {
        "angles": (shoulder_angle_deg, elbow_angle_deg, wrist1_angle_deg),
        "positions": [base, shoulder, elbow, wrist]
    }

def move_robot(angles):
    """
    Moves the UR16e to the calculated joint angles.
    
    Parameters:
        angles (tuple): (shoulder, elbow, wrist1) angles in degrees.
    """
    shoulder, elbow, wrist1 = np.radians(angles)  # Convert to radians
    joint_positions = [0.0, shoulder, elbow, wrist1, 0.0, 0.0]  # Keeping other joints static

    print(f"Moving UR16e to Joint Positions: {joint_positions}")
    rtde_c.moveJ(joint_positions, 0.5, 0.5)  # Move with speed=0.5 rad/s, acceleration=0.5 rad/s²

def plot_robot_arm(positions):
    """Plots the robotic arm in 3D space."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract joint coordinates
    x_vals = [p[0] for p in positions]
    y_vals = [p[1] for p in positions]
    z_vals = [p[2] for p in positions]

    # Plot arm segments
    ax.plot(x_vals, y_vals, z_vals, marker='o', linestyle='-', color='b', markersize=8, label="UR16e Arm")

    # Labels and formatting
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.set_title("UR16e Arm Movement")
    ax.legend()
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([0, 1])

    plt.show()

# Example Usage
initial_position = (0.3, 0.0, 0.5)  # Example starting position
delta_z = 0.1  # Move up by 0.1 meters

result = compute_joint_positions(initial_position, delta_z)
shoulder, elbow, wrist1 = result["angles"]
positions = result["positions"]

print(f"Shoulder: {shoulder:.2f}°, Elbow: {elbow:.2f}°, Wrist1: {wrist1:.2f}°")

# Move the real UR16e
move_robot((shoulder, elbow, wrist1))

# Plot the arm
plot_robot_arm(positions)

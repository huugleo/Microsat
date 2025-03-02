import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# UR16e robot arm dimensions (in meters) from DH Parameters
L_B = 0.170   # Base height
a2 = 0.476    # Shoulder to elbow
a3 = 0.361    # Elbow to wrist 1
d4 = 0.194    # Wrist 1 to Wrist 2
d5 = 0.120    # Wrist 2 to Wrist 3
L_TP = 0.112  # Tool plate length

def dh_transformation(alpha, a, d, theta):
    """Compute the Denavit-Hartenberg transformation matrix."""
    theta = np.radians(theta)
    alpha = np.radians(alpha)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0, 0, 0, 1]
    ])

def inverse_kinematics(z):
    """Compute joint angles (theta2, theta3, theta4) given z position for UR16e, keeping the end effector parallel to the ground."""
    
    z_wrist = z - L_B
    D = (z_wrist**2 - a2**2 - a3**2) / (2 * a2 * a3)
    if np.abs(D) > 1:
        raise ValueError("Target position is out of reach.")
    
    theta3 = np.arctan2(-np.sqrt(1 - D**2), D)  
    theta2 = np.arctan2(z_wrist, a2 + a3 * np.cos(theta3))
    
    theta4 = -theta2 - theta3  # Ensure wrist keeps camera parallel to the floor
    theta1 = 0.0  # Fixed base
    theta5 = 0.0  # No rotation needed
    theta6 = 0.0  # No rotation needed
    
    return np.degrees([theta1, theta2, theta3, theta4, theta5, theta6])

def plot_robot(theta2, theta3, theta4):
    """Plot the robot arm in 3D space based on joint angles."""
    
    theta2, theta3, theta4 = np.radians([theta2, theta3, theta4])
    
    T1 = dh_transformation(0, 0, L_B, 0)
    T2 = dh_transformation(90, 0, 0, theta2 + 90)
    T3 = dh_transformation(0, a2, 0, theta3)
    T4 = dh_transformation(0, a3, d4, theta4 - 90)
    
    base = np.array([0, 0, 0, 1])
    joint2 = T1 @ T2 @ base
    joint3 = T1 @ T2 @ T3 @ base
    wrist1 = T1 @ T2 @ T3 @ T4 @ base
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    points = np.array([base[:3], joint2[:3], joint3[:3], wrist1[:3]])
    ax.plot(points[:, 0], points[:, 1], points[:, 2], marker='o', linestyle='-', color='b')
    ax.scatter(*wrist1[:3], color='r', s=100, label='End Effector')
    
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("UR16e Robot Arm Configuration")
    ax.legend()
    plt.show()

z_target = 0.5
joint_angles = inverse_kinematics(z_target)
print(f"UR16e Joint angles (theta1, theta2, theta3, theta4, theta5, theta6): {joint_angles}")
plot_robot(joint_angles[1], joint_angles[2], joint_angles[3])

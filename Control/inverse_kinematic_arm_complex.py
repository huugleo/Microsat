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

def inverse_kinematics(x, y, z):
    """Compute joint angles (theta1, theta2, theta3, theta4, theta5, theta6) given (x, y, z) position for UR16e."""
    
    theta1 = np.arctan2(y, x)
    x_wrist = np.sqrt(x**2 + y**2) - L_TP
    z_wrist = z - L_B
    
    D = (x_wrist**2 + z_wrist**2 - a2**2 - a3**2) / (2 * a2 * a3)
    if np.abs(D) > 1:
        raise ValueError("Target position is out of reach.")
    
    theta3 = np.arctan2(-np.sqrt(1 - D**2), D)  
    theta2 = np.arctan2(z_wrist, x_wrist) - np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))
    
    theta4 = -theta2 - theta3  # Ensure wrist keeps camera parallel to the floor
    theta5 = 0.0  
    theta6 = 0.0  
    
    return np.degrees([theta1, theta2, theta3, theta4, theta5, theta6])

def plot_robot(theta1, theta2, theta3, theta4, theta5, theta6):
    """Plot the robot arm in 3D space based on joint angles."""
    
    theta1, theta2, theta3, theta4, theta5, theta6 = np.radians([theta1, theta2, theta3, theta4, theta5, theta6])
    
    T1 = dh_transformation(0, 0, L_B, theta1)
    T2 = dh_transformation(90, 0, 0, theta2 + 90)
    T3 = dh_transformation(0, a2, 0, theta3)
    T4 = dh_transformation(0, a3, d4, theta4 - 90)
    T5 = dh_transformation(-90, 0, d5, theta5)
    T6 = dh_transformation(90, 0, 0, theta6)
    
    base = np.array([0, 0, 0, 1])
    joint1 = T1 @ base
    joint2 = T1 @ T2 @ base
    joint3 = T1 @ T2 @ T3 @ base
    wrist1 = T1 @ T2 @ T3 @ T4 @ base
    wrist2 = T1 @ T2 @ T3 @ T4 @ T5 @ base
    end_effector = T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ base
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    points = np.array([base[:3], joint1[:3], joint2[:3], joint3[:3], wrist1[:3], wrist2[:3], end_effector[:3]])
    ax.plot(points[:, 0], points[:, 1], points[:, 2], marker='o', linestyle='-', color='b')
    ax.scatter(*end_effector[:3], color='r', s=100, label='End Effector')
    
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("UR16e Robot Arm Configuration")
    ax.legend()
    plt.show()

x_target, y_target, z_target = 0.3, 0.2, 0.5
joint_angles = inverse_kinematics(x_target, y_target, z_target)
print(f"UR16e Joint angles (theta1, theta2, theta3, theta4, theta5, theta6): {joint_angles}")
plot_robot(*joint_angles)

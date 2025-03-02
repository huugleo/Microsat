import numpy as np
import matplotlib.pyplot as plt

r_1 = 0.4784
r_2 = 0.47985
r_3 = 0.1 # Camera length place holder

def direct_kin(theta_1, theta_2):
    # Gets the three angles for the joints of interest.
    theta_3 = (theta_1 + theta_2)
    # print(theta_3)
    base = [0, 0, -0.1]
    shoulder_joint = [0, 0, 0] # x, y, z
    elbow_joint = [r_1*np.cos(theta_1), 0, r_1*np.sin(theta_1)]
    end_joint = [r_2*np.cos(theta_2) + elbow_joint[0], 0 + elbow_joint[1], r_2*np.sin(theta_2)+ elbow_joint[2]]
    camera_end = [end_joint[0] + r_3*np.cos(theta_3), end_joint[1], end_joint[2] + r_3*np.sin(theta_3)]
    points = np.vstack((base, shoulder_joint, elbow_joint, end_joint, camera_end))
    return points

def inverse_kin(target_height):
    # Compute theta_2 using the cosine rule
    cos_theta_2 = (target_height**2 - (r_1**2 + r_2**2)) / (2 * r_1 * r_2)

    if abs(cos_theta_2) > 1:
        print("Target height is out of reach.")
        return None

    angle_2 = np.arccos(cos_theta_2)  # Two possible solutions: ±theta_2

    # Compute theta_1 using inverse sine
    angle_1 = np.arcsin((target_height - r_2 * np.sin(angle_2)) / r_1)

    return angle_1, angle_2

robot_arm_shoulder_lift_joint = 1/3*np.pi
robot_arm_elbow_joint = 1/6*np.pi

pos = direct_kin(robot_arm_shoulder_lift_joint, robot_arm_elbow_joint)
dir_height = pos[3,2] # Retrieves the height of the end effector from points, works
print(["TARGET HEIGHT", dir_height])
inv_theta_1, inv_theta_2 = inverse_kin(dir_height)



########################
## PLOTTING
########################
fig_3d, ax = plt.subplots(1,2, figsize=(15,10))
ax = fig_3d.add_subplot(121, projection='3d')

ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], marker='o', linestyle='-', color='b')

ax.scatter(*pos[1,:], color="k", s=100, label="Shoulder joint base")
ax.scatter(*pos[2,:], color="g", s=100, label="Elbow joint")
ax.scatter(*pos[3,:], color="r", s=100, label="End Effector")
ax.set_xlim([-1,1])

ax.set_ylim([-0.1,0.1])
ax.set_zlim([-0.1,1])
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title("UR16e Robot Arm Configuration")
ax.legend()
ax.set_aspect('equal', 'box')


### INVERSE
pos = direct_kin(inv_theta_1, inv_theta_2)
ax2 = fig_3d.add_subplot(122, projection='3d')

ax2.plot(pos[:, 0], pos[:, 1], pos[:, 2], marker='o', linestyle='-', color='b')

ax2.scatter(*pos[1,:], color="k", s=100, label="Shoulder joint base")
ax2.scatter(*pos[2,:], color="g", s=100, label="Elbow joint")
ax2.scatter(*pos[3,:], color="r", s=100, label="End Effector")
ax2.set_xlim([-1,1])

ax2.set_ylim([-0.1,0.1])
ax2.set_zlim([-0.1,1])
ax2.set_xlabel("X [m]")
ax2.set_ylabel("Y [m]")
ax2.set_zlabel("Z [m]")
ax2.set_title("UR16e Robot Arm Configuration")
ax2.legend()
ax2.set_aspect('equal', 'box')

plt.show()
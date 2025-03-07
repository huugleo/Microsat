#!/usr/bin/env python3
import numpy as np
import rospy
import math
from get_arm_state import get_joint_angles
from arm_control import move_arm
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# OPTIONAL: if you want to use scipy for a quick numeric solve:
try:
    import scipy.optimize
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

#######################
# ROBOT PARAMETERS
#######################
L1 = 0.4784  # link length 1 in meters
L2 = 0.360  # link length 2
L3 = 0.11985  # link length 3

JOINT_NAMES = ["robot_arm_shoulder_lift_joint", "robot_arm_elbow_joint", "robot_arm_wrist_1_joint"]  # adapt to your robot
INIT_JOINTS = [0.0, 0.0, 0.0]  # your starting guess

# Weighted factor for orientation constraint
LAMBDA = 10.0

#######################
# FORWARD KINEMATICS
#######################
def direct_kin(q):
    """ Return (x, y) for the end effector given angles q = [q1, q2, q3]. """
    q1, q2, q3 = q
    x = (L1*math.cos(q1)
         + L2*math.cos(q1 + q2)
         + L3*math.cos(q1 + q2 + q3))
    z = (L1*math.sin(q1)
         + L2*math.sin(q1 + q2)
         + L3*math.sin(q1 + q2 + q3))
    return x, z

#######################
# COST FUNCTION
#######################
def cost_function(q, x0, z0, delta_z):
    """
    Minimizes the difference in final y position AND enforces q1+q2+q3 = 0
    to keep the end-effector orientation horizontal.

    x0, y0 = current end effector position
    delta_y = desired small upward motion
    """
    # We only care about final Y and orientation sum
    x_calc, z_calc = direct_kin(q)

    # We want final y_calc to be (y0 + delta_y),
    # but we don't care if x drifts for this example, so let's let x float.

    # orientation constraint => sum of angles = 0
    orient_error = (q[0] + q[1] + q[2])**2

    # desired final y error
    z_error = (z_calc - (z0 + delta_z))**2

    return z_error + LAMBDA*orient_error

#######################
# MAIN DEMO
#######################

def inverse_kin(delta_z, current_positions):
    # rospy.init_node("planar_vertical_ik_demo_print_only")

    if not HAS_SCIPY:
        rospy.logerr("scipy is required for this example numeric solver.")
        return

    # 1) Start from some initial angles
    current_q = np.array(current_positions, dtype=float)

    # 2) Compute current end-effector position
    x0, z0 = direct_kin(current_q)

    # 3) Define the cost function for the solver
    def objective(q):
        return cost_function(q, x0, z0, delta_z)

    # 4) Solve
    res = scipy.optimize.minimize(objective, current_q, method='BFGS')
    if not res.success:
        rospy.logwarn("IK solver failed: %s", res.message)
        return

    new_q = res.x

    # 5) Joint-limit check: joint1 <= 0
    if new_q[0] > 0.0:
        rospy.logwarn("Joint 1 is out of limit (%.3f > 0). Discarding solution.", new_q[0])
        return

    # 6) Print results
    rospy.loginfo("Found a solution that moves up by %.3f m:", delta_z)
    rospy.loginfo("   %s = %.3f rad", JOINT_NAMES[0], new_q[0])
    rospy.loginfo("   %s = %.3f rad", JOINT_NAMES[1], new_q[1])
    rospy.loginfo("   %s = %.3f rad", JOINT_NAMES[2], new_q[2])

    # Also show final (x,y)
    xf, zf = direct_kin(new_q)
    rospy.loginfo("Final end-effector position: x=%.3f, y=%.3f", xf, zf)
    return new_q[0], new_q[1], new_q[2]


if __name__ == "__main__":
    rospy.init_node("image_trigger_script", anonymous=True)
    move_arm_pub = rospy.Publisher('/robot/arm/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)

    # Calculate angles for first movement
    old_angles = get_joint_angles()
    print("Old angles: ", old_angles)
    # old_angles[0] = old_angles[0]-np.pi/6
    # old_angles[3] = old_angles[3]+np.pi/6
    # old_angles[2] = np.pi
    # old_angles[4] = old_angles[4]+np.pi/2
    angle_tuple = (old_angles[1], old_angles[0], old_angles[3])
    print(angle_tuple)
    delta_z = 0.20  # change this to first step size
    pos_current = direct_kin(angle_tuple)
    # print(pos_current)
    current_end_z = pos_current[1]
    z_target = current_end_z + delta_z
    new_angles = old_angles
    new_angles[1], new_angles[0], new_angles[3] = inverse_kin(delta_z, angle_tuple)
    # new_angles[0] = -new_angles[0]
    print(new_angles)
    move_arm(new_angles, move_arm_pub)
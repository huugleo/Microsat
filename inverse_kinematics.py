#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

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
def fk(q):
    """ Return (x, y) for the end effector given angles q = [q1, q2, q3]. """
    q1, q2, q3 = q
    x = (L1*math.cos(q1)
         + L2*math.cos(q1 + q2)
         + L3*math.cos(q1 + q2 + q3))
    y = (L1*math.sin(q1)
         + L2*math.sin(q1 + q2)
         + L3*math.sin(q1 + q2 + q3))
    return x, y

#######################
# COST FUNCTION
#######################
def cost_function(q, x0, y0, delta_y):
    """
    Minimizes the difference in final y position AND enforces q1+q2+q3 = 0
    to keep the end-effector orientation horizontal.

    x0, y0 = current end effector position
    delta_y = desired small upward motion
    """
    # We only care about final Y and orientation sum
    x_calc, y_calc = fk(q)

    # We want final y_calc to be (y0 + delta_y),
    # but we don't care if x drifts for this example, so let's let x float.

    # orientation constraint => sum of angles = 0
    orient_error = (q[0] + q[1] + q[2])**2

    # desired final y error
    y_error = (y_calc - (y0 + delta_y))**2

    return y_error + LAMBDA*orient_error

#######################
# MAIN DEMO
#######################

def main(delta_y, current_positions):
    rospy.init_node("planar_vertical_ik_demo_print_only")

    if not HAS_SCIPY:
        rospy.logerr("scipy is required for this example numeric solver.")
        return

    # 1) Start from some initial angles
    current_q = np.array(current_positions, dtype=float)

    # 2) Compute current end-effector position
    x0, y0 = fk(current_q)

    # 3) Define the cost function for the solver
    def objective(q):
        return cost_function(q, x0, y0, delta_y)

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
    rospy.loginfo("Found a solution that moves up by %.3f m:", delta_y)
    rospy.loginfo("   %s = %.3f rad", JOINT_NAMES[0], new_q[0])
    rospy.loginfo("   %s = %.3f rad", JOINT_NAMES[1], new_q[1])
    rospy.loginfo("   %s = %.3f rad", JOINT_NAMES[2], new_q[2])

    # Also show final (x,y)
    xf, yf = fk(new_q)
    rospy.loginfo("Final end-effector position: x=%.3f, y=%.3f", xf, yf)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

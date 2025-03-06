import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

# Initialize the ROS node
rospy.init_node('control_ur_arm')

# Create a publisher to the eff_joint_traj_controller's command topic
pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

# Wait for the publisher to connect
rospy.sleep(1)

# Create a new JointTrajectory message
traj_msg = JointTrajectory()
traj_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Create a JointTrajectoryPoint to send the joint positions
point = JointTrajectoryPoint()
point.positions = [-np.pi/2, -np.pi/3, -np.pi/6, -np.pi*3/6, -np.pi/2, 0.0]  # Example joint positions in radians
# point.positions = [-np.pi/2, 0.17010624651202022, -1.9063374210922932, (-1.739513942425246), -np.pi/2, 0.0]  # Example joint positions in radians
point.time_from_start = rospy.Duration(3.0)  # Duration to reach these positions

# Add the point to the trajectory message
traj_msg.points.append(point)

# Publish the message
pub.publish(traj_msg)

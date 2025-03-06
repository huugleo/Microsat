import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_arm(input_positions, pub):
    print("Moving arm with joint positions:", input_positions)  # Debug print to check input positions
    # Create a new JointTrajectory message
    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']  # Match /joint_states order
    # Create a JointTrajectoryPoint to send the joint positions
    point = JointTrajectoryPoint()
    point.positions = input_positions  # Example joint positions in radians
    point.time_from_start = rospy.Duration(3.0)  # Duration to reach these positions

    # Add the point to the trajectory message
    traj_msg.points.append(point)

    # Publish the message
    pub.publish(traj_msg)
    rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("move_arm", anonymous=True)
    move_arm_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    positions = [-1.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    print("moving arm")
    move_arm(positions, move_arm_pub)

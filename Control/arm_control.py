import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

def move_arm(input_positions, pub):
    print("Moving arm with joint positions:", input_positions)  # Debug print to check input positions
    # Create a new JointTrajectory message
    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['robot_arm_elbow_joint', 'robot_arm_shoulder_lift_joint', 'robot_arm_shoulder_pan_joint', 'robot_arm_wrist_1_joint', 'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint']  # Match /joint_states order
    # Create a JointTrajectoryPoint to send the joint positions
    movement_time = 15.0
    point = JointTrajectoryPoint()
    point.positions = input_positions  # Example joint positions in radians
    point.time_from_start = rospy.Duration(movement_time)  # Duration to reach these positions

    # Add the point to the trajectory message
    traj_msg.points.append(point)
    rospy.sleep(1)
    # Publish the message
    pub.publish(traj_msg)
    rospy.sleep(movement_time)
    rospy.loginfo("Arm movement completed")

if __name__ == "__main__":
    rospy.init_node("move_arm", anonymous=True)
    move_arm_pub = rospy.Publisher('/robot/arm/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)
    # positions = [-1.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    positions = [0.31760865846742803, -1.1173250538161774, 3.292952897027135e-05, 0.8128904539295654, 1.63602876663208, -2.3561399618731897]
    print("moving arm")
    move_arm(positions, move_arm_pub)

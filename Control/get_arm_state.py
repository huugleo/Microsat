import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

def get_joint_angles():
    """
    Retrieves the current joint angles whenever this function is called
    """
    msg = rospy.wait_for_message("robot/arm/joint_states", JointState)

    return list(msg.position)

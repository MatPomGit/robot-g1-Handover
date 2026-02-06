from geometry_msgs.msg import PoseStamped

def compute_handover_pose(human_hand_pose):
    pose = PoseStamped()
    pose.header = human_hand_pose.header
    pose.pose = human_hand_pose.pose
    pose.pose.position.z -= 0.05   # ergonomiczny offset
    return pose

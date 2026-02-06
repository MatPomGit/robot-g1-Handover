from geometry_msgs.msg import PoseStamped

def compute_pregrasp(object_pose: PoseStamped) -> PoseStamped:
    pre = PoseStamped()
    pre.header = object_pose.header
    pre.pose = object_pose.pose
    pre.pose.position.z += 0.10
    return pre

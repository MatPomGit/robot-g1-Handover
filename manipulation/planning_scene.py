from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def add_table(scene):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = 0.6
    pose.pose.position.z = 0.35
    pose.pose.orientation.w = 1.0
    scene.add_box("table", pose, size=(1.0, 1.0, 0.7))

def add_human(scene):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = 0.8
    pose.pose.position.z = 0.9
    pose.pose.orientation.w = 1.0
    scene.add_cylinder("human", pose, height=1.7, radius=0.3)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

class MoveItInterface(Node):

    def __init__(self):
        super().__init__("moveit_interface")
        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("gripper")
        self.scene = PlanningSceneInterface()

    def move_to_pose(self, pose: PoseStamped):
        self.arm.set_pose_target(pose)
        plan = self.arm.plan()
        if plan[0]:
            self.arm.execute(plan[1], wait=True)

    def close_gripper(self):
        self.gripper.set_named_target("closed")
        self.gripper.go(wait=True)

    def open_gripper(self):
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)

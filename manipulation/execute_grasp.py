import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from manipulation.moveit_interface import MoveItInterface
from manipulation.grasp_planner import compute_pregrasp
from decision.wma_task_manager import WMATaskManager

class ExecuteGrasp(Node):

    def __init__(self):
        super().__init__("execute_grasp")
        self.moveit = MoveItInterface()
        self.wma = WMATaskManager()
        self.sub = self.create_subscription(
            PoseStamped, "/object_pose", self.cb, 10)

    def cb(self, pose):
        state = self.wma.update(pose)

        if state == "approach":
            self.moveit.move_to_pose(compute_pregrasp(pose))

        elif state == "grasp":
            self.moveit.move_to_pose(pose)
            self.moveit.close_gripper()

        elif state == "lift":
            lifted = PoseStamped()
            lifted.header = pose.header
            lifted.pose = pose.pose
            lifted.pose.position.z += 0.15
            self.moveit.move_to_pose(lifted)

def main():
    rclpy.init()
    rclpy.spin(ExecuteGrasp())
    rclpy.shutdown()

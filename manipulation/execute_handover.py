import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from manipulation.moveit_interface import MoveItInterface
from manipulation.handover_planner import compute_handover_pose
from decision.wma_handover_manager import WMAHandoverManager
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ExecuteHandoverWMA(Node):

    def __init__(self):
        super().__init__("execute_handover_wma")
        self.moveit = MoveItInterface()
        self.wma = WMAHandoverManager()

        self.human_pose = None
        self.human_reaching = False
        self.gripper_occupied = False
        self.camera_rgb = None
        self.bridge = CvBridge()

        self.create_subscription(PoseStamped, "/human_hand_pose", self.human_cb, 10)
        self.create_subscription(Bool, "/human_reaching", self.intent_cb, 10)
        self.create_subscription(Bool, "/gripper_state", self.gripper_cb, 10)
        self.create_subscription(Image, "/camera/color/image_raw", self.image_cb, 10)

    def human_cb(self, msg):
        self.human_pose = msg
        self.update()

    def intent_cb(self, msg):
        self.human_reaching = msg.data

    def gripper_cb(self, msg):
        self.gripper_occupied = msg.data

    def image_cb(self, msg):
        self.camera_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def update(self):
        if self.human_pose is None or self.camera_rgb is None:
            return

        # Tworzymy obserwację dla WMA
        observation = {
            "camera_rgb": self.camera_rgb,
            "gripper_state": self.gripper_occupied,
            "human_pose": self.human_pose,
            "human_reaching": self.human_reaching
        }

        action = self.wma.infer_action(observation)

        if action == "TAKE_FROM_HUMAN":
            pose = compute_handover_pose(self.human_pose)
            self.moveit.move_to_pose(pose)
            self.moveit.close_gripper()

        elif action == "GIVE_TO_HUMAN":
            pose = compute_handover_pose(self.human_pose)
            self.moveit.move_to_pose(pose)
            self.moveit.open_gripper()

        # IDLE = brak akcji

def main():
    rclpy.init()
    rclpy.spin(ExecuteHandoverWMA())
    rclpy.shutdown()

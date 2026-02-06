import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class HumanHandDetector(Node):

    def __init__(self):
        super().__init__("human_hand_detector")
        self.pub_pose = self.create_publisher(
            PoseStamped, "/human_hand_pose", 10)
        self.pub_intent = self.create_publisher(
            Bool, "/human_reaching", 10)

    def detect_hand(self, image):
        # Placeholder – MediaPipe / OpenPose
        # Zwraca (x,y,z) ręki
        return [0.6, 0.0, 1.0]

    def process(self):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.6
        pose.pose.position.y = 0.0
        pose.pose.position.z = 1.0
        pose.pose.orientation.w = 1.0

        self.pub_pose.publish(pose)
        self.pub_intent.publish(Bool(data=True))

def main():
    rclpy.init()
    node = HumanHandDetector()
    rclpy.spin(node)
    rclpy.shutdown()

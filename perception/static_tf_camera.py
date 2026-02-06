import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticCameraTF(Node):

    def __init__(self):
        super().__init__("static_camera_tf")
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        t = TransformStamped()

        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.25
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.90
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(StaticCameraTF())
    rclpy.shutdown()

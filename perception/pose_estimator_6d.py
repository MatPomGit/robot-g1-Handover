import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np

class PoseEstimator6D(Node):

    def __init__(self):
        super().__init__("pose_estimator_6d")

        self.bridge = CvBridge()
        self.depth = None
        self.fx = self.fy = self.cx = self.cy = None

        self.create_subscription(Image,
                                 "/camera/depth/image_raw",
                                 self.depth_cb, 10)
        self.create_subscription(CameraInfo,
                                 "/camera/color/camera_info",
                                 self.info_cb, 10)
        self.create_subscription(Detection2DArray,
                                 "/object_detections",
                                 self.det_cb, 10)

        self.pub = self.create_publisher(PoseStamped,
                                         "/object_pose", 10)

    def info_cb(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)

    def det_cb(self, msg):
        if self.depth is None or self.fx is None:
            return
        if not msg.detections:
            return

        det = msg.detections[0]
        u = int(det.bbox.center.x)
        v = int(det.bbox.center.y)
        z = float(self.depth[v, u]) / 1000.0
        if z <= 0.1:
            return

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        pose = PoseStamped()
        pose.header.frame_id = "camera_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        self.pub.publish(pose)

def main():
    rclpy.init()
    rclpy.spin(PoseEstimator6D())
    rclpy.shutdown()

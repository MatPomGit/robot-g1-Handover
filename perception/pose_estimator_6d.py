"""
Moduł estymatora pozy 6D obiektów (6D Pose Estimator)

Ten node ROS 2 oblicza pozycję 3D (x, y, z) obiektów wykrytych na obrazie 2D.
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from typing import Optional

# Configuration constants
MIN_DEPTH_M = 0.1  # Minimum valid depth in meters
MAX_DEPTH_M = 10.0  # Maximum valid depth in meters
DEPTH_SCALE = 1000.0  # Depth conversion: mm to m


class PoseEstimator6D(Node):
    """Node ROS 2 do estymacji pozy 6D obiektów"""

    def __init__(self):
        """Inicjalizacja node'a PoseEstimator6D"""
        super().__init__("pose_estimator_6d")

        self.bridge = CvBridge()
        
        self.depth: Optional[np.ndarray] = None
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self.depth_cb, 10)
        
        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self.info_cb, 10)
        
        self.create_subscription(Detection2DArray, "/object_detections",
                                 self.det_cb, 10)

        self.pub = self.create_publisher(PoseStamped, "/object_pose", 10)
        
        self.get_logger().info('Pose Estimator 6D initialized')

    def info_cb(self, msg: CameraInfo) -> None:
        """Callback odbierający parametry kalibracji kamery"""
        if len(msg.k) != 9:
            self.get_logger().error('Invalid camera calibration matrix')
            return
            
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        
        if self.fx <= 0 or self.fy <= 0:
            self.get_logger().error(f'Invalid focal lengths: fx={self.fx}, fy={self.fy}')
            self.fx = self.fy = None

    def depth_cb(self, msg: Image) -> None:
        """Callback odbierający mapę głębokości z kamery"""
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            self.depth = None

    def det_cb(self, msg: Detection2DArray) -> None:
        """Callback przetwarzający detekcje 2D i obliczający pozycje 3D"""
        if self.depth is None:
            self.get_logger().warn('No depth data available')
            return
            
        if self.fx is None or self.fy is None:
            self.get_logger().warn('Camera calibration not available')
            return
        
        if not msg.detections:
            return

        det = msg.detections[0]
        
        u = int(det.bbox.center.x)
        v = int(det.bbox.center.y)
        
        # Validate pixel coordinates
        if not (0 <= v < self.depth.shape[0] and 0 <= u < self.depth.shape[1]):
            self.get_logger().warn(f'Pixel coordinates out of bounds: ({u}, {v})')
            return
        
        z = float(self.depth[v, u]) / DEPTH_SCALE
        
        if not (MIN_DEPTH_M < z < MAX_DEPTH_M):
            self.get_logger().warn(f'Invalid depth value: {z:.3f}m')
            return

        # Pinhole camera model
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        pose = PoseStamped()
        pose.header.frame_id = "camera_link"
        pose.header.stamp = msg.header.stamp
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        self.pub.publish(pose)
        self.get_logger().debug(f'Published pose: ({x:.3f}, {y:.3f}, {z:.3f})')


def main(args=None):
    """Funkcja główna uruchamiająca node PoseEstimator6D"""
    rclpy.init(args=args)
    
    try:
        node = PoseEstimator6D()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in PoseEstimator6D: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

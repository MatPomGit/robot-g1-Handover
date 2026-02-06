"""
Moduł detekcji obiektów (Object Detector)

Ten node ROS 2 wykrywa obiekty na obrazie z kamery RGB używając sieci neuronowej YOLOv5.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from typing import Optional

# Configuration constants
DEFAULT_MODEL = "yolov5s"
DEFAULT_CONFIDENCE_THRESHOLD = 0.6
IMAGE_TOPIC = "/camera/color/image_raw"
DETECTION_TOPIC = "/object_detections"


class ObjectDetector(Node):
    """Node ROS 2 do detekcji obiektów za pomocą YOLOv5"""

    def __init__(self):
        """Inicjalizacja node'a ObjectDetector"""
        super().__init__("object_detector")
        
        # Declare parameters
        self.declare_parameter('confidence_threshold', DEFAULT_CONFIDENCE_THRESHOLD)
        self.declare_parameter('model_name', DEFAULT_MODEL)
        
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.model_name = self.get_parameter('model_name').value
        
        self.sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_cb, 10)
        
        self.pub = self.create_publisher(
            Detection2DArray, DETECTION_TOPIC, 10)

        self.bridge = CvBridge()
        
        # Load YOLOv5 model with error handling
        try:
            self.model = torch.hub.load(
                "ultralytics/yolov5", self.model_name, pretrained=True)
            self.get_logger().info(f'YOLOv5 model "{self.model_name}" loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv5 model: {e}')
            raise

    def image_cb(self, msg: Image) -> None:
        """Callback wywoływany przy każdym nowym obrazie z kamery"""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge conversion error: {e}')
            return
        
        if img is None or img.size == 0:
            self.get_logger().warn('Received empty image')
            return
        
        try:
            results = self.model(img)
        except Exception as e:
            self.get_logger().error(f'YOLOv5 inference error: {e}')
            return

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        for *xyxy, conf, cls in results.xyxy[0]:
            if conf < self.confidence_threshold:
                continue
            
            det = self._create_detection(xyxy, conf, int(cls))
            if det is not None:
                det_msg.detections.append(det)

        self.pub.publish(det_msg)
        self.get_logger().debug(f'Published {len(det_msg.detections)} detections')

    def _create_detection(self, bbox, conf: float, cls: int) -> Optional[Detection2D]:
        """Tworzy wiadomość Detection2D z pojedynczego wykrycia YOLOv5"""
        try:
            det = Detection2D()
            bb = BoundingBox2D()
            
            bb.center.x = float((bbox[0] + bbox[2]) / 2)
            bb.center.y = float((bbox[1] + bbox[3]) / 2)
            bb.size_x = float(bbox[2] - bbox[0])
            bb.size_y = float(bbox[3] - bbox[1])
            
            det.bbox = bb

            hyp = ObjectHypothesisWithPose()
            hyp.id = str(cls)
            hyp.score = float(conf)
            
            det.results.append(hyp)
            return det
        except Exception as e:
            self.get_logger().error(f'Error creating detection: {e}')
            return None


def main(args=None):
    """Funkcja główna uruchamiająca node ObjectDetector"""
    rclpy.init(args=args)
    
    try:
        node = ObjectDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in ObjectDetector: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

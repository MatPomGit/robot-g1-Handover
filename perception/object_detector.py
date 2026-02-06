import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import torch

class ObjectDetector(Node):

    def __init__(self):
        super().__init__("object_detector")
        self.sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_cb, 10)
        self.pub = self.create_publisher(
            Detection2DArray, "/object_detections", 10)

        self.bridge = CvBridge()
        self.model = torch.hub.load(
            "ultralytics/yolov5", "yolov5s", pretrained=True)

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img)

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        for *xyxy, conf, cls in results.xyxy[0]:
            if conf < 0.6:
                continue
            det = self._create_detection(xyxy, conf, int(cls))
            det_msg.detections.append(det)

        self.pub.publish(det_msg)

    def _create_detection(self, bbox, conf, cls):
        from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose
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

def main():
    rclpy.init()
    rclpy.spin(ObjectDetector())
    rclpy.shutdown()

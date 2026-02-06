"""
Moduł detekcji dłoni człowieka (Human Hand Detector)

UWAGA: Jest to wersja placeholder dla celów edukacyjnych.
Pełna implementacja wymaga integracji z MediaPipe lub OpenPose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from typing import Optional
import numpy as np

# Configuration constants
HAND_DETECTION_RATE_HZ = 30  # Detection rate
DEFAULT_HAND_POSITION = [0.6, 0.0, 1.0]  # Default hand position [x, y, z] in meters


class HumanHandDetector(Node):
    """
    Node ROS 2 do detekcji dłoni człowieka
    
    UWAGA: Obecna implementacja jest placeholderem.
    TODO: Zintegrować MediaPipe Hands lub OpenPose dla prawdziwej detekcji.
    """

    def __init__(self):
        """Inicjalizacja node'a HumanHandDetector"""
        super().__init__("human_hand_detector")
        
        self.pub_pose = self.create_publisher(
            PoseStamped, "/human_hand_pose", 10)
        
        self.pub_intent = self.create_publisher(
            Bool, "/human_reaching", 10)
        
        self.bridge = CvBridge()
        self.current_image: Optional[np.ndarray] = None
        
        # Subscribe to camera for future integration
        self.create_subscription(
            Image, "/camera/color/image_raw", self.image_cb, 10)
        
        # Timer for periodic updates
        self.timer = self.create_timer(
            1.0 / HAND_DETECTION_RATE_HZ, self.process)
        
        self.get_logger().warn(
            'HumanHandDetector initialized in PLACEHOLDER mode. '
            'Real hand detection not implemented yet.'
        )

    def image_cb(self, msg: Image) -> None:
        """Callback odbierający obraz z kamery"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def detect_hand(self, image: Optional[np.ndarray]) -> list:
        """
        Wykrywa pozycję dłoni na obrazie
        
        TODO: Zintegrować MediaPipe Hands:
        import mediapipe as mp
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands()
        results = hands.process(image)
        # Extract hand position from results.multi_hand_landmarks
        
        Args:
            image: Obraz z kamery RGB (numpy array, format BGR)
            
        Returns:
            list: Pozycja dłoni [x, y, z] w metrach
        """
        # PLACEHOLDER - zwraca stałą pozycję
        return DEFAULT_HAND_POSITION

    def process(self) -> None:
        """Przetwarza detekcję i publikuje wyniki"""
        hand_position = self.detect_hand(self.current_image)
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        
        pose.pose.position.x = hand_position[0]
        pose.pose.position.y = hand_position[1]
        pose.pose.position.z = hand_position[2]
        pose.pose.orientation.w = 1.0

        self.pub_pose.publish(pose)
        
        # Placeholder: zawsze True
        # TODO: Implement real reaching detection based on:
        # - Hand movement direction
        # - Hand velocity
        # - Distance from robot
        self.pub_intent.publish(Bool(data=True))


def main(args=None):
    """Funkcja główna uruchamiająca node HumanHandDetector"""
    rclpy.init(args=args)
    
    try:
        node = HumanHandDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in HumanHandDetector: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

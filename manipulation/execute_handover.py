"""
Moduł wykonywania przekazywania obiektów z WMA (Execute Handover with WMA)

Ten node ROS 2 jest głównym kontrolerem systemu przekazywania obiektów
między człowiekiem a robotem z użyciem World Model AI.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from typing import Optional
import numpy as np

# Import local modules
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from manipulation.moveit_interface import MoveItInterface
from manipulation.handover_planner import compute_handover_pose, validate_handover_safety
from config.constants import (
    HUMAN_HAND_POSE_TOPIC, HUMAN_REACHING_TOPIC,
    GRIPPER_STATE_TOPIC, CAMERA_RGB_TOPIC,
    ACTION_TAKE_FROM_HUMAN, ACTION_GIVE_TO_HUMAN, ACTION_IDLE
)

# Note: WMA import is optional - will work in mock mode if not available
try:
    from decision.wma_handover_manager import WMAHandoverManager
    WMA_AVAILABLE = True
except ImportError:
    WMA_AVAILABLE = False
    print("WARNING: WMA not available, using mock decision making")


class ExecuteHandoverWMA(Node):
    """Node ROS 2 wykonujący przekazywanie obiektów z użyciem WMA"""

    def __init__(self):
        """Inicjalizacja node'a ExecuteHandoverWMA"""
        super().__init__("execute_handover_wma")
        
        try:
            self.moveit = MoveItInterface()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MoveIt: {e}')
            raise
        
        # Initialize WMA if available
        if WMA_AVAILABLE:
            try:
                from decision.wma_handover_manager import WMAHandoverManager
                self.wma = WMAHandoverManager()
                self.get_logger().info('WMA initialized successfully')
            except Exception as e:
                self.get_logger().warn(f'WMA initialization failed: {e}. Using mock mode.')
                self.wma = None
        else:
            self.wma = None
            self.get_logger().warn('WMA not available - using mock decision making')

        # State variables
        self.human_pose: Optional[PoseStamped] = None
        self.human_reaching: bool = False
        self.gripper_occupied: bool = False
        self.camera_rgb: Optional[np.ndarray] = None
        
        self.bridge = CvBridge()
        
        # Subscribers
        self.create_subscription(
            PoseStamped, HUMAN_HAND_POSE_TOPIC, self.human_cb, 10)
        self.create_subscription(
            Bool, HUMAN_REACHING_TOPIC, self.intent_cb, 10)
        self.create_subscription(
            Bool, GRIPPER_STATE_TOPIC, self.gripper_cb, 10)
        self.create_subscription(
            Image, CAMERA_RGB_TOPIC, self.image_cb, 10)
        
        self.get_logger().info('ExecuteHandoverWMA initialized')

    def human_cb(self, msg: PoseStamped) -> None:
        """Callback odbierający pozycję dłoni człowieka"""
        self.human_pose = msg
        self.update()

    def intent_cb(self, msg: Bool) -> None:
        """Callback odbierający informację o intencji człowieka"""
        self.human_reaching = msg.data

    def gripper_cb(self, msg: Bool) -> None:
        """Callback odbierający stan chwytaka"""
        self.gripper_occupied = msg.data

    def image_cb(self, msg: Image) -> None:
        """Callback odbierający obraz RGB z kamery"""
        try:
            self.camera_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def mock_decision(self) -> str:
        """
        Mock decision making when WMA is not available
        
        Returns:
            str - akcja do wykonania
        """
        # Simple rule-based logic
        if self.human_reaching:
            if not self.gripper_occupied:
                return ACTION_TAKE_FROM_HUMAN
            else:
                return ACTION_GIVE_TO_HUMAN
        return ACTION_IDLE

    def update(self) -> None:
        """Główna funkcja przetwarzania - podejmuje i wykonuje decyzje"""
        if self.human_pose is None:
            self.get_logger().debug('Waiting for human pose data')
            return

        # Prepare observation
        observation = {
            "camera_rgb": self.camera_rgb,
            "gripper_state": self.gripper_occupied,
            "human_pose": self.human_pose,
            "human_reaching": self.human_reaching
        }

        # Get action from WMA or mock
        try:
            if self.wma is not None and self.camera_rgb is not None:
                action = self.wma.infer_action(observation)
            else:
                action = self.mock_decision()
        except Exception as e:
            self.get_logger().error(f'Error in decision making: {e}')
            action = ACTION_IDLE

        # Execute action
        self.execute_action(action)

    def execute_action(self, action: str) -> None:
        """
        Wykonuje wybraną akcję
        
        Args:
            action: str - akcja do wykonania (TAKE/GIVE/IDLE)
        """
        if action == ACTION_TAKE_FROM_HUMAN:
            self.get_logger().info('Executing: TAKE_FROM_HUMAN')
            self.execute_take()
            
        elif action == ACTION_GIVE_TO_HUMAN:
            self.get_logger().info('Executing: GIVE_TO_HUMAN')
            self.execute_give()
            
        else:  # ACTION_IDLE
            self.get_logger().debug('Action: IDLE (waiting)')

    def execute_take(self) -> None:
        """Wykonuje akcję TAKE_FROM_HUMAN"""
        try:
            pose = compute_handover_pose(self.human_pose)
            if pose is None:
                self.get_logger().error('Failed to compute handover pose')
                return
            
            # Safety check
            current_pose = self.moveit.get_current_pose()
            if current_pose and not validate_handover_safety(self.human_pose, current_pose):
                self.get_logger().error('Handover position unsafe - aborting')
                return
            
            # Execute movement
            success = self.moveit.move_to_pose(pose)
            if success:
                self.moveit.close_gripper()
                self.get_logger().info('Successfully took object from human')
            else:
                self.get_logger().error('Failed to move to handover position')
                
        except Exception as e:
            self.get_logger().error(f'Error in execute_take: {e}')

    def execute_give(self) -> None:
        """Wykonuje akcję GIVE_TO_HUMAN"""
        try:
            pose = compute_handover_pose(self.human_pose)
            if pose is None:
                self.get_logger().error('Failed to compute handover pose')
                return
            
            # Safety check
            current_pose = self.moveit.get_current_pose()
            if current_pose and not validate_handover_safety(self.human_pose, current_pose):
                self.get_logger().error('Handover position unsafe - aborting')
                return
            
            # Execute movement
            success = self.moveit.move_to_pose(pose)
            if success:
                self.moveit.open_gripper()
                self.get_logger().info('Successfully gave object to human')
            else:
                self.get_logger().error('Failed to move to handover position')
                
        except Exception as e:
            self.get_logger().error(f'Error in execute_give: {e}')


def main(args=None):
    """Funkcja główna uruchamiająca node ExecuteHandoverWMA"""
    rclpy.init(args=args)
    
    try:
        node = ExecuteHandoverWMA()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in ExecuteHandoverWMA: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Interfejs MoveIt 2 (MoveIt Interface)

Ten moduł zapewnia uproszczony interfejs do planowania i wykonywania ruchów
ramienia robota oraz sterowania chwytakiem za pomocą MoveIt 2.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from typing import Optional, Tuple

# Import configuration constants
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.constants import (
    ARM_MOVE_GROUP, GRIPPER_MOVE_GROUP,
    MAX_VELOCITY_SCALE, MAX_ACCELERATION_SCALE,
    MOVEMENT_TIMEOUT_SEC
)


class MoveItInterface(Node):
    """Node ROS 2 zapewniający interfejs do MoveIt 2"""

    def __init__(self):
        """Inicjalizacja interfejsu MoveIt 2"""
        super().__init__("moveit_interface")
        
        try:
            self.robot = RobotCommander()
            self.arm = MoveGroupCommander(ARM_MOVE_GROUP)
            self.gripper = MoveGroupCommander(GRIPPER_MOVE_GROUP)
            self.scene = PlanningSceneInterface()
            
            # Set velocity and acceleration scaling
            self.arm.set_max_velocity_scaling_factor(MAX_VELOCITY_SCALE)
            self.arm.set_max_acceleration_scaling_factor(MAX_ACCELERATION_SCALE)
            
            # Set planning timeout
            self.arm.set_planning_time(MOVEMENT_TIMEOUT_SEC)
            
            self.get_logger().info('MoveIt Interface initialized successfully')
            self.get_logger().info(f'Robot groups: {self.robot.get_group_names()}')
            self.get_logger().info(f'End effector: {self.arm.get_end_effector_link()}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MoveIt Interface: {e}')
            raise

    def move_to_pose(self, pose: PoseStamped, wait: bool = True) -> bool:
        """
        Przesuwa ramię robota do zadanej pozycji w przestrzeni
        
        Args:
            pose: PoseStamped - docelowa pozycja i orientacja
            wait: bool - czekać na zakończenie ruchu
            
        Returns:
            bool - True jeśli ruch się powiódł, False w przeciwnym razie
        """
        try:
            self.arm.set_pose_target(pose)
            
            success, plan, planning_time, error_code = self.arm.plan()
            
            if not success:
                self.get_logger().error(
                    f'Planning failed with error code: {error_code}'
                )
                return False
            
            self.get_logger().info(
                f'Planning succeeded in {planning_time:.2f}s, executing...'
            )
            
            execute_success = self.arm.execute(plan, wait=wait)
            
            if execute_success:
                self.get_logger().info('Motion executed successfully')
            else:
                self.get_logger().error('Motion execution failed')
                
            return execute_success
            
        except Exception as e:
            self.get_logger().error(f'Error in move_to_pose: {e}')
            return False

    def move_to_joint_values(self, joint_values: list, wait: bool = True) -> bool:
        """
        Przesuwa ramię do zadanych wartości kątów stawów
        
        Args:
            joint_values: list - lista wartości kątów dla każdego stawu
            wait: bool - czekać na zakończenie ruchu
            
        Returns:
            bool - True jeśli ruch się powiódł
        """
        try:
            self.arm.set_joint_value_target(joint_values)
            return self.arm.go(wait=wait)
        except Exception as e:
            self.get_logger().error(f'Error in move_to_joint_values: {e}')
            return False

    def close_gripper(self, wait: bool = True) -> bool:
        """
        Zamyka chwytak robota
        
        Args:
            wait: bool - czekać na zakończenie
            
        Returns:
            bool - True jeśli operacja się powiodła
        """
        try:
            self.gripper.set_named_target("closed")
            success = self.gripper.go(wait=wait)
            if success:
                self.get_logger().info('Gripper closed successfully')
            else:
                self.get_logger().error('Failed to close gripper')
            return success
        except Exception as e:
            self.get_logger().error(f'Error in close_gripper: {e}')
            return False

    def open_gripper(self, wait: bool = True) -> bool:
        """
        Otwiera chwytak robota
        
        Args:
            wait: bool - czekać na zakończenie
            
        Returns:
            bool - True jeśli operacja się powiodła
        """
        try:
            self.gripper.set_named_target("open")
            success = self.gripper.go(wait=wait)
            if success:
                self.get_logger().info('Gripper opened successfully')
            else:
                self.get_logger().error('Failed to open gripper')
            return success
        except Exception as e:
            self.get_logger().error(f'Error in open_gripper: {e}')
            return False

    def stop(self) -> None:
        """Zatrzymuje wszystkie ruchy robota (emergency stop)"""
        try:
            self.arm.stop()
            self.gripper.stop()
            self.get_logger().warn('Emergency stop executed')
        except Exception as e:
            self.get_logger().error(f'Error in stop: {e}')

    def get_current_pose(self) -> Optional[PoseStamped]:
        """
        Pobiera aktualną pozycję end-effectora
        
        Returns:
            PoseStamped - aktualna pozycja lub None w przypadku błędu
        """
        try:
            return self.arm.get_current_pose()
        except Exception as e:
            self.get_logger().error(f'Error getting current pose: {e}')
            return None

    def get_current_joint_values(self) -> Optional[list]:
        """
        Pobiera aktualne wartości kątów stawów
        
        Returns:
            list - lista kątów lub None w przypadku błędu
        """
        try:
            return self.arm.get_current_joint_values()
        except Exception as e:
            self.get_logger().error(f'Error getting joint values: {e}')
            return None


def main(args=None):
    """Funkcja główna - test interfejsu MoveIt"""
    rclpy.init(args=args)
    
    try:
        node = MoveItInterface()
        
        # Test: wyświetl aktualną pozycję
        current_pose = node.get_current_pose()
        if current_pose:
            node.get_logger().info(f'Current pose: {current_pose}')
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in MoveItInterface: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

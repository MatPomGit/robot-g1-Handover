"""
Planer chwytania obiektów (Grasp Planner)

Ten moduł oblicza pozycję pre-grasp (przed-chwytową), która jest używana
podczas podchodzenia do obiektu przed jego chwyceniem.
"""

from geometry_msgs.msg import PoseStamped
from typing import Optional
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.constants import PREGRASP_OFFSET_Z, LIFT_HEIGHT, FRAME_BASE_LINK


def compute_pregrasp(object_pose: PoseStamped, offset_z: float = PREGRASP_OFFSET_Z) -> Optional[PoseStamped]:
    """
    Oblicza pozycję pre-grasp na podstawie pozycji obiektu
    
    Args:
        object_pose: PoseStamped - pozycja obiektu do chwycenia
        offset_z: float - offset w osi Z (domyślnie z constants)
    
    Returns:
        PoseStamped - pozycja pre-grasp lub None w przypadku błędu
    """
    if object_pose is None:
        return None
    
    try:
        pre = PoseStamped()
        pre.header = object_pose.header
        pre.pose = object_pose.pose
        
        pre.pose.position.z += offset_z
        
        return pre
    except Exception as e:
        print(f'Error computing pregrasp: {e}')
        return None


def compute_lift_pose(grasp_pose: PoseStamped, lift_height: float = LIFT_HEIGHT) -> Optional[PoseStamped]:
    """
    Oblicza pozycję podniesienia po chwyceniu obiektu
    
    Args:
        grasp_pose: PoseStamped - pozycja chwytania
        lift_height: float - wysokość podniesienia
    
    Returns:
        PoseStamped - pozycja podniesiona lub None w przypadku błędu
    """
    if grasp_pose is None:
        return None
    
    try:
        lift = PoseStamped()
        lift.header = grasp_pose.header
        lift.pose = grasp_pose.pose
        
        lift.pose.position.z += lift_height
        
        return lift
    except Exception as e:
        print(f'Error computing lift pose: {e}')
        return None


def validate_grasp_pose(pose: PoseStamped) -> bool:
    """
    Waliduje czy pozycja chwytania jest osiągalna i bezpieczna
    
    Args:
        pose: PoseStamped - pozycja do walidacji
    
    Returns:
        bool - True jeśli pozycja jest poprawna
    """
    if pose is None:
        return False
    
    # Check if position is within reasonable bounds
    pos = pose.pose.position
    
    # Example workspace limits (adjust for your robot)
    if not (0.2 <= pos.x <= 1.0):  # 20cm to 1m in front
        return False
    if not (-0.5 <= pos.y <= 0.5):  # +/- 50cm left/right
        return False
    if not (0.0 <= pos.z <= 1.5):  # 0 to 1.5m height
        return False
    
    return True

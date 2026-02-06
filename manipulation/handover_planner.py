"""
Planer przekazywania obiektów (Handover Planner)

Ten moduł oblicza optymalną pozycję, w której robot powinien przekazać
lub odebrać obiekt od człowieka.
"""

from geometry_msgs.msg import PoseStamped
from typing import Optional
import sys
import os
import copy
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.constants import HANDOVER_OFFSET_Z, EMERGENCY_STOP_DISTANCE
import math


def compute_handover_pose(human_hand_pose: PoseStamped, 
                          offset_z: float = HANDOVER_OFFSET_Z) -> Optional[PoseStamped]:
    """
    Oblicza pozycję przekazania obiektu na podstawie pozycji dłoni człowieka
    
    Args:
        human_hand_pose: PoseStamped - pozycja dłoni człowieka w 3D
        offset_z: float - ergonomiczny offset w osi Z
    
    Returns:
        PoseStamped - pozycja dla robota lub None w przypadku błędu
    """
    if human_hand_pose is None:
        return None
    
    try:
        pose = copy.deepcopy(human_hand_pose)
        pose.pose.position.z += offset_z
        
        return pose
    except Exception as e:
        print(f'Error computing handover pose: {e}')
        return None


def validate_handover_safety(human_hand_pose: PoseStamped,
                             robot_pose: PoseStamped) -> bool:
    """
    Waliduje bezpieczeństwo przekazania - sprawdza odległość między robotem a człowiekiem
    
    Args:
        human_hand_pose: PoseStamped - pozycja dłoni człowieka
        robot_pose: PoseStamped - pozycja robota
    
    Returns:
        bool - True jeśli przekazanie jest bezpieczne
    """
    if human_hand_pose is None or robot_pose is None:
        return False
    
    try:
        # Calculate Euclidean distance
        dx = robot_pose.pose.position.x - human_hand_pose.pose.position.x
        dy = robot_pose.pose.position.y - human_hand_pose.pose.position.y
        dz = robot_pose.pose.position.z - human_hand_pose.pose.position.z
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Check if distance is safe (not too close)
        if distance < EMERGENCY_STOP_DISTANCE:
            print(f'SAFETY WARNING: Distance {distance:.3f}m is too close!')
            return False
        
        # Check if distance is reachable (not too far)
        if distance > 1.0:  # 1 meter max reach
            print(f'WARNING: Distance {distance:.3f}m may be unreachable')
            return False
        
        return True
    except Exception as e:
        print(f'Error validating handover safety: {e}')
        return False


def compute_approach_trajectory(current_pose: PoseStamped,
                                target_pose: PoseStamped,
                                num_waypoints: int = 5) -> list:
    """
    Oblicza trajektorię podejścia do pozycji przekazania
    
    Args:
        current_pose: PoseStamped - aktualna pozycja robota
        target_pose: PoseStamped - docelowa pozycja przekazania
        num_waypoints: int - liczba punktów pośrednich
    
    Returns:
        list - lista PoseStamped reprezentujących trajektorię
    """
    if current_pose is None or target_pose is None:
        return []
    
    trajectory = []
    
    try:
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            
            waypoint = PoseStamped()
            waypoint.header = target_pose.header
            
            # Linear interpolation
            waypoint.pose.position.x = (
                current_pose.pose.position.x * (1 - t) + 
                target_pose.pose.position.x * t
            )
            waypoint.pose.position.y = (
                current_pose.pose.position.y * (1 - t) + 
                target_pose.pose.position.y * t
            )
            waypoint.pose.position.z = (
                current_pose.pose.position.z * (1 - t) + 
                target_pose.pose.position.z * t
            )
            
            # Keep orientation same as target
            waypoint.pose.orientation = target_pose.pose.orientation
            
            trajectory.append(waypoint)
    except Exception as e:
        print(f'Error computing approach trajectory: {e}')
        return []
    
    return trajectory

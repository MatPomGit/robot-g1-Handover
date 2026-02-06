"""
Plik uruchomieniowy podstawowego pipeline (Full Pipeline Launch)

Ten launch file uruchamia podstawowy pipeline wykrywania dłoni człowieka
i przekazywania obiektów.

KROK PO KROKU:
1. Uruchamia node wykrywający dłoń człowieka (human_hand_detector)
2. Uruchamia node wykonujący przekazywanie obiektów (execute_handover)

UŻYCIE:
    ros2 launch g1_pick_and_handover full_pipeline.launch.py

CO ROBI TEN LAUNCH FILE:
- Uruchamia minimalistyczny system do testowania handover
- Nie uruchamia percepcji obiektów (tylko detekcja dłoni)
- Idealny do prototypowania i debugowania

WYMAGANIA:
- human_hand_detector musi być zbudowany i zainstalowany
- execute_handover musi być zbudowany i zainstalowany
- MoveIt 2 musi być uruchomiony osobno

TOPIKI UTWORZONE:
- /human_hand_pose (geometry_msgs/PoseStamped) - pozycja dłoni
- /human_reaching (std_msgs/Bool) - intencja człowieka
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generuje opis uruchomienia (launch description)
    
    Returns:
        LaunchDescription: Lista node'ów do uruchomienia
    """
    return LaunchDescription([
        # Node wykrywający dłoń człowieka
        Node(package="g1_pick_and_handover",
               executable="human_hand_detector"),
        # Node wykonujący przekazywanie obiektów
        Node(package="g1_pick_and_handover",
               executable="execute_handover"),
    ])

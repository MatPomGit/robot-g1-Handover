"""
Plik uruchomieniowy modułu percepcji (Launch Perception)

Ten launch file uruchamia wszystkie komponenty percepcji jednocześnie:
- static_tf_camera (transformacja kamery)
- object_detector (detekcja obiektów YOLOv5)
- pose_estimator_6d (estymacja pozy 3D)
- human_hand_detector (detekcja dłoni)

UŻYCIE:
    ros2 launch g1_pick_and_handover launch_perception.launch.py

WYMAGANIA:
- Kamera RGB-D (np. Intel RealSense D435) uruchomiona osobno
- Zainstalowany PyTorch i YOLOv5
- Zainstalowany MediaPipe
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generuje opis uruchomienia modułu percepcji

    Returns:
        LaunchDescription: Lista node'ów percepcji do uruchomienia
    """
    return LaunchDescription([
        Node(
            package='g1_pick_and_handover',
            executable='static_tf_camera',
            name='camera_tf',
            output='screen'
        ),
        Node(
            package='g1_pick_and_handover',
            executable='object_detector',
            name='obj_detector',
            output='screen'
        ),
        Node(
            package='g1_pick_and_handover',
            executable='pose_estimator_6d',
            name='pose_est',
            output='screen'
        ),
        Node(
            package='g1_pick_and_handover',
            executable='human_hand_detector',
            name='hand_detector',
            output='screen'
        ),
    ])

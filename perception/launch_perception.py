"""
Plik uruchomieniowy percepcji (Launch Perception)

Ten plik to placeholder dla przyszłego launch file'a, który uruchomi
wszystkie komponenty percepcji jednocześnie.

W ROS 2, launch file'y pozwalają:
- Uruchamiać wiele node'ów jednocześnie
- Przekazywać parametry konfiguracyjne
- Zarządzać zależnościami między node'ami
- Automatyzować setup środowiska

PRZYKŁADOWE UŻYCIE (po implementacji):
    ros2 launch g1_pick_and_handover launch_perception.py

TO DO: Zaimplementować launch file uruchamiający:
- static_tf_camera (transformacja kamery)
- object_detector (detekcja obiektów)
- pose_estimator_6d (estymacja pozy 3D)
- human_hand_detector (detekcja dłoni)

PRZYKŁADOWA STRUKTURA:
    from launch import LaunchDescription
    from launch_ros.actions import Node
    
    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='g1_pick_and_handover',
                executable='static_tf_camera',
                name='camera_tf'
            ),
            Node(
                package='g1_pick_and_handover',
                executable='object_detector',
                name='obj_detector'
            ),
            Node(
                package='g1_pick_and_handover',
                executable='pose_estimator_6d',
                name='pose_est'
            ),
            Node(
                package='g1_pick_and_handover',
                executable='human_hand_detector',
                name='hand_detector'
            )
        ])
"""

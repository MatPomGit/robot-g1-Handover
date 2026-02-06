from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TF kamery względem bazy
        Node(package="g1_pick_and_handover",
             executable="static_tf_camera"),
        # Percepcja obiektów
        Node(package="g1_pick_and_handover",
             executable="object_detector"),
        Node(package="g1_pick_and_handover",
             executable="pose_estimator_6d"),
        # Detekcja ręki człowieka
        Node(package="g1_pick_and_handover",
             executable="human_hand_detector"),
        # Handover z WMA
        Node(package="g1_pick_and_handover",
             executable="execute_handover_wma")
    ])

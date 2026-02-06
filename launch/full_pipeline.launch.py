from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="g1_pick_and_handover",
               executable="human_hand_detector"),
        Node(package="g1_pick_and_handover",
               executable="execute_handover"),
             
    ])

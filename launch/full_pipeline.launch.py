from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="g1_pick_and_handover",
             executable="static_tf_camera"),
        Node(package="g1_pick_and_handover",
             executable="object_detector"),
        Node(package="g1_pick_and_handover",
             executable="pose_estimator_6d"),
        Node(package="g1_pick_and_handover",
             executable="execute_grasp")
    ])

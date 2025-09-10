# file: carla_environment/launch/data_collection.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='carla_environment', executable='camera_yolo_node', output='screen'),
        Node(package='carla_environment', executable='lidar_cluster_node', output='screen'),
        Node(package='carla_environment', executable='ekf_lead', output='screen'),
    ])


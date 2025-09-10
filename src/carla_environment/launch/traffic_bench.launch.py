from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Spawn background traffic
        Node(package='carla_environment', executable='spawn_traffic_node', output='screen',
             parameters=[{'n_cars': 60, 'seed': 7}]),

        # Perception
        Node(package='carla_environment', executable='camera_yolo_node', output='screen'),
        Node(package='carla_environment', executable='lidar_cluster_node', output='screen'),

        # Ground truth publisher
        Node(package='carla_environment', executable='carla_gt_publisher', output='screen'),

        # Tracker (EKF)
        Node(package='carla_environment', executable='ekf_tracker', output='screen',
             parameters=[{'algo_name': 'ekf', 'fixed_frame': 'base_link'}]),

        # Evaluator (pass one or more algos, e.g., "ekf,ukf,pf")
        Node(package='carla_environment', executable='online_evaluator', output='screen',
             parameters=[{'algos': 'ekf'}]),
    ])


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('carla_localization_benchmark')
    params = os.path.join(pkg, 'config', 'params.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'loc_benchmark.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='carla_localization_benchmark',
            executable='noise_sensors',
            name='noise_sensors',
            parameters=[params],
            output='screen'),
        Node(
            package='carla_localization_benchmark',
            executable='ekf_localizer',
            name='ekf_localizer',
            parameters=[params],
            output='screen'),
        Node(
            package='carla_localization_benchmark',
            executable='metrics_node',
            name='metrics_node',
            parameters=[{'gt_odom_topic': '/carla/ego/ground_truth/odom',
                         'fused_odom_topic': '/fusion/odom',
                         'window_n': 300,
                         'csv_log_path': ''}],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=LaunchConfiguration('use_rviz'))
    ])


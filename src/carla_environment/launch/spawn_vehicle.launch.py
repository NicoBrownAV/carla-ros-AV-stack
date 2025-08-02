from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carla_environment',
            executable='spawn_vehicle_w_sensors',
            name='vehicle_node',
            output='screen'
        )
    ])


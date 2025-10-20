from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'carla_localization_benchmark'  # or your package name
    share = get_package_share_directory(pkg)
    config_dir = share + '/config'

    # navsat transform
    navsat_tf = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config_dir + '/navsat.yaml'],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/gps/fix', '/gps/fix'),
            # navsat needs yaw source:
            ('/odometry/filtered', '/odometry/filtered/ekf'),
            ('/gps/odom', '/gps/odom')
        ]
    )

    # EKF filter
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        output='screen',
        parameters=[config_dir + '/ekf.yaml'],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered/ekf'),
            ('/set_pose', '/set_pose')
        ]
    )

    # # Optional UKF  
    # ukf = Node(
    #     package='robot_localization',
    #     executable='ukf_node',
    #     name='ukf_filter',
    #     output='screen',
    #     parameters=[config_dir + '/ukf.yaml'],
    #     remappings=[
    #         ('/odometry/filtered', '/odometry/filtered/ukf'),
    #         ('/set_pose', '/set_pose')
    #     ]
    # )

    # Your CSV logger / metrics nodes
    csv_logger = Node(
        package=pkg,
        executable='csv_logger.py',
        name='csv_logger',
        output='screen',
        parameters=[{'filter_topic': '/odometry/filtered/ekf'}]
    )

    metrics = Node(
        package=pkg,
        executable='metrics_node.py',
        name='metrics_node',
        output='screen',
        parameters=[{'filter_topic': '/odometry/filtered/ekf'}]
    )

    # Sensor / vehicle nodes (from carla_environment)
    from launch_ros.actions import Node as ROSNode
    sensor_nodes = [
        ROSNode(
            package='carla_environment',
            executable='carla_gt_publisher.py',
            name='carla_gt_publisher',
            output='screen'
        ),
        # add other nodes: imu adapter, gps adapter, etc
    ]

    return LaunchDescription([navsat_tf, ekf, csv_logger, metrics] + sensor_nodes)
    # return LaunchDescription([navsat_tf, ukf, csv_logger, metrics] + sensor_nodes)


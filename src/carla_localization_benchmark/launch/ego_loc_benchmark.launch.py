#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_manual = LaunchConfiguration('use_manual')  # true/false

    pkg   = 'carla_localization_benchmark'
    share = get_package_share_directory(pkg)
    params = os.path.join(share, 'config', 'params.yaml')

    # 1) Spawner first
    spawner = Node(
        package=pkg,
        executable='spawn_vehicle_w_sensors',
        name='spawn_vehicle_w_sensors',
        output='screen',
    )

    # 2) Ego GT Odom (delay so ego exists)
    ego_gt = Node(
        package=pkg,
        executable='ego_gt_odom',
        name='ego_gt_odom',
        output='screen',
        parameters=[{'ego_role_name': 'hero', 'rate_hz': 50.0}],
    )
    ego_gt_delayed = TimerAction(period=1.5, actions=[ego_gt])

    # 3) Noisy sensors (after GT is likely up)
    noise = Node(
        package=pkg,
        executable='noise_sensors',
        name='noise_sensors',
        output='screen',
        parameters=[params],
    )
    noise_delayed = TimerAction(period=2.0, actions=[noise])

    # 3.5) Measurement Mux (latches speed + imu -> /ekf/inputs)
    mux = Node(
        package=pkg,
        executable='measurement_mux',
        name='measurement_mux',
        output='screen',
        parameters=[{'rate_hz': 50.0}],  # publish ekf inputs at 50 Hz
    )
    mux_delayed = TimerAction(period=1.8, actions=[mux])

    # # 4) EKF (modular node using ekf_core)
    # ekf = Node(
    #     package=pkg,
    #     executable='ekf_node',
    #     name='ekf_localizer',
    #     output='screen',
    #     parameters=[{
    #         # you can also read these from params.yaml if you prefer
    #         'ekf_rate_hz': 50.0,
    #         'init_source': 'gps',           # or 'gt'
    #         'init_with_velocity': True,
    #         'q_x': 0.05, 'q_y': 0.05, 'q_yaw': 0.02, 'q_v': 0.20,
    #         'gps_r_x_std_m': 0.8, 'gps_r_y_std_m': 0.8, 'gps_gate_chi2': 7.0,
        # }],
    # )

    # ekf_delayed = TimerAction(period=2.0, actions=[ekf])

    ekf = Node(
        package=pkg,
        executable='ekf_node_sync',
        name='ekf_localizer',
        output='screen',
        parameters=[{
            'ekf_rate_hz': 50.0,          # heartbeat/republish
            'init_with_velocity': True,
            'gps_r_x_std_m': 1.2,
            'gps_r_y_std_m': 1.2,
            'q_x': 0.05, 'q_y': 0.05, 'q_yaw': 0.03, 'q_v': 0.25,
        }],
    )
    ekf_delayed = TimerAction(period=2.0, actions=[ekf])

    # 5) CSV Logger (EKF, GPS, GT, DIAG -> CSV)
    csv = Node(
        package=pkg,
        executable='csv_logger',
        name='csv_logger',
        output='screen',
        parameters=[{
            'csv_path': os.path.expanduser('~/Documents/1.Localization_data/ekf_run_latest.csv')
        }],
    )
    csv_delayed = TimerAction(period=2.2, actions=[csv])

    plotter = Node(
        package='carla_localization_benchmark',
        executable='live_plotter',
        name='live_plotter',
        output='screen',
        parameters=[{'max_points': 8000, 'update_hz': 10.0}],
    )
    plotter_delayed = TimerAction(period=2.4, actions=[plotter])
    # add plotter_delayed to your LaunchDescription list

    spawner = Node(
    package=pkg,
    executable='spawn_vehicle_w_sensors',
    name='spawn_vehicle_w_sensors',
    output='screen',
    parameters=[{
        'use_autopilot': True,
        'tm_global_speed_reduction': 10.0,
        'tm_vehicle_speed_reduction': 0.0,
        'tm_min_dist': 2.5,
        'tm_auto_lane_change': True,
        'tm_ignore_traffic_lights': False,
        'tm_ignore_signs': False,
        'tm_hybrid_physics': True,
        'tm_port': 0,
    }],
)
    


    return LaunchDescription([
        DeclareLaunchArgument('use_manual', default_value='false'),
        spawner,
        ego_gt_delayed,
        noise_delayed,
        mux_delayed,
        ekf_delayed,
        csv_delayed,
    ])

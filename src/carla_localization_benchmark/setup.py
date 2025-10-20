from setuptools import setup, find_packages

package_name = 'carla_localization_benchmark'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/params.yaml',
            'config/sensors.yaml'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/ego_loc_benchmark.launch.py'
        ]),
        # uncomment if you plan to include an RViz file
        # ('share/' + package_name + '/rviz', [
        #     'carla_localization_benchmark/rviz/loc_benchmark.rviz'
        # ]),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='CARLA localization benchmark (spawner, GT, noisy sensors, EKF, metrics, controller)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # core localization nodes
            'noise_sensors = carla_localization_benchmark.noise_sensors:main',  
            'metrics_node = carla_localization_benchmark.metrics_node:main',

            # environment / vehicle nodes
            'spawn_vehicle_w_sensors = carla_localization_benchmark.spawn_vehicle_w_sensors:main',
            'ego_gt_odom = carla_localization_benchmark.ego_gt_odom:main',

            # optional utilities
            'carla_gt_publisher = carla_localization_benchmark.carla_gt_publisher:main',

            # tracker
            'ekf_node = carla_localization_benchmark.ekf_node:main',
            'ekf_node_sync = carla_localization_benchmark.ekf_node_sync:main',
            'measurement_mux = carla_localization_benchmark.measurement_mux:main',
            'csv_logger = carla_localization_benchmark.csv_logger:main',

            # live plot
            'live_plotter = carla_localization_benchmark.live_plotter:main',

              
        ],
    },
)

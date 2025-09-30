from setuptools import find_packages, setup

package_name = 'carla_localization_benchmark'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/benchmark.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),       
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nic',
    maintainer_email='nico.brown@gmail.com',
    description='Real-time GPS+IMU localization benchmark with EKF vs CARLA GT',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'noise_sensors = carla_localization_benchmark.noise_sensors:main',
            'ekf_localizer = carla_localization_benchmark.ekf_localizer:main',
            'metrics_node = carla_localization_benchmark.metrics_node:main',
        ],
    },
)

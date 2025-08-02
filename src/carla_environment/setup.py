from setuptools import setup
from setuptools import find_packages

package_name = 'carla_environment'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sensors.yaml']),
        ('share/' + package_name + '/launch', ['launch/spawn_vehicle.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicholas Brown',
    maintainer_email='nico.brown.av@gmail.com',
    description='Spawns ego vehicle and sensors in CARLA with ROS 2 integration',
    license='MIT',
    entry_points={
        'console_scripts': [
            'spawn_vehicle_w_sensors = carla_environment.spawn_vehicle_w_sensors:main',
        ],
    },
)


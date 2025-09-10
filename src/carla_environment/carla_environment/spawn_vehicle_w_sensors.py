#!/usr/bin/env python3

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import rclpy
import carla
import numpy as np
import os
import yaml
import struct
import signal
import sys
import std_msgs.msg
import math

class CarlaSensorPublisher(Node):
    def __init__(self):
        super().__init__('carla_sensor_publisher')

        # === CARLA Connection ===
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        # === ROS Setup ===
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.actors = []

        # === ROS Publishers ===
        self.lidar_pub = self.create_publisher(PointCloud2, '/vehicle_lidar/points', 10)
        self.radar_pub = self.create_publisher(PointCloud2, '/vehicle_radar/detections', 10)
        
        self.camera_pub = self.create_publisher(Image, '/vehicle_camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/vehicle_camera/camera_info', 10)

        self.gnss_pub = self.create_publisher(NavSatFix, '/vehicle_gnss/fix', 10)

        # === Load sensor config ===
        self.sensor_cfg = self.load_sensor_config()

        # === Spawn Vehicle and Sensors ===
        self.spawn_vehicle_and_sensors()

        # === Graceful shutdown on Ctrl+C ===
        signal.signal(signal.SIGINT, self.shutdown)

        self.get_logger().info("Vehicle and sensors running. Ctrl+C to exit.")
    
    def load_sensor_config(self):
        cfg_path = os.path.join(
            get_package_share_directory('carla_environment'),
            'config',
            'sensors.yaml'
        )
        with open(cfg_path, 'r') as f:
            return yaml.safe_load(f)

    def apply_attributes(self, bp, params):
        for key, value in params.items():
            if key != 'blueprint':
                try:
                    bp.set_attribute(key, str(value))
                except RuntimeError:
                    self.get_logger().warn(f"Attribute '{key}' not supported by {bp.id}")

    def spawn_vehicle_and_sensors(self):
        # === Ego Vehicle ===
        vehicle_bp = self.bp_lib.filter("vehicle.tesla.model3")[0]
        spawn_point = self.world.get_map().get_spawn_points()[0]
        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        self.actors.append(self.vehicle)
        self.get_logger().info("Spawned ego vehicle.")

        # === Sensors ===
        for sensor_name, params in self.sensor_cfg.items():
            bp = self.bp_lib.find(params['blueprint'])
            self.apply_attributes(bp, params)

            if sensor_name == 'camera':
                transform = carla.Transform(carla.Location(x=1.0, z=1.6))
                sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
                sensor.listen(lambda data: self.camera_callback(data, sensor))
                self.camera = sensor

            elif sensor_name == 'lidar':
                transform = carla.Transform(carla.Location(x=0.0, z=2.0))
                sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
                sensor.listen(lambda data: self.lidar_callback(data, sensor))
                self.lidar = sensor

            elif sensor_name == 'gnss':
                transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.9))
                sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
                sensor.listen(lambda data: self.gnss_callback(data, sensor))
                self.gnss = sensor

            elif sensor_name == 'radar':
                transform = carla.Transform(carla.Location(x=2.25, z=0.75))
                sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
                sensor.listen(lambda data: self.radar_callback(data, sensor))
                self.radar = sensor

            self.actors.append(sensor)
            self.get_logger().info(f"Attached {sensor_name} sensor.")

    def broadcast_transform(self, sensor_actor):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'base_link'
        tf_msg.child_frame_id = sensor_actor.attributes.get('role_name', sensor_actor.type_id)

        tf = sensor_actor.get_transform()
        tf_msg.transform.translation.x = tf.location.x
        tf_msg.transform.translation.y = tf.location.y
        tf_msg.transform.translation.z = tf.location.z

        # Convert rotation (Euler) to quaternion if needed
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_msg)

    def radar_callback(self, radar_data, actor):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = actor.attributes.get('role_name', 'radar_frame')

        points = []
        for detection in radar_data:
            # Compute position in sensor frame
            azimuth = detection.azimuth  # radians
            altitude = detection.altitude  # radians
            r = detection.depth # meters

            # Spherical to Catesian coordinates
            x = r * np.cos(altitude) * np.cos(azimuth)
            y = -1 * ( r * np.cos(altitude) * np.sin(azimuth) )
            z = r * np.sin(altitude)
            velocity = detection.velocity

            points.append([x, y, z, velocity])

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1), 
        ]

        pointcloud = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=16 * len(points),
            is_dense=True,
            data=np.array(points, dtype=np.float32).tobytes()
        )

        self.radar_pub.publish(pointcloud)
        self.broadcast_transform(actor)

    def lidar_callback(self, data, actor):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = actor.attributes.get('role_name', 'lidar_frame')

        points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)

        # Convert to ROS convention: x forward, y LEFT, z up
        points[:, 1] = -points[:, 1]

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        cloud = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=16 * len(points),
            is_dense=True,
            data=points.tobytes()
        )

        self.lidar_pub.publish(cloud)
        self.broadcast_transform(actor)

    def publish_camera_info(self, frame_id: str, width: int, height: int, fov_deg: float):
        # Normalize types early
        width  = int(width)
        height = int(height)
        fov_deg = float(fov_deg)

        cam_info = CameraInfo()
        cam_info.header.stamp = self.get_clock().now().to_msg()
        cam_info.header.frame_id = str(frame_id)
        cam_info.width = width
        cam_info.height = height
        cam_info.distortion_model = 'plumb_bob'
        cam_info.d = []  # or [0.0]*5 if you prefer explicit zeros

        # Focal length from horizontal FOV (square pixels assumption)
        fov_rad = math.radians(fov_deg)
        fx = (width  / 2.0) / math.tan(fov_rad / 2.0)
        fy = fx  # square pixels

        # Principal point at the image center (pixel center convention)
        cx = (width  - 1) / 2.0
        cy = (height - 1) / 2.0

        # Intrinsic matrix K (row-major)
        cam_info.k = [
            float(fx), 0.0,       float(cx),
            0.0,       float(fy), float(cy),
            0.0,       0.0,       1.0,
        ]

        # Projection matrix P for monocular (no Tx)
        cam_info.p = [
            float(fx), 0.0,       float(cx), 0.0,
            0.0,       float(fy), float(cy), 0.0,
            0.0,       0.0,       1.0,       0.0,
        ]

        # (Optional) Identity rectification matrix R if you want to fill it:
        # cam_info.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]

        self.camera_info_pub.publish(cam_info)

    def camera_callback(self, data, actor):
        # CARLA gives BGRA; you’re slicing :3 (BGR). Either publish as 'bgr8' or reorder to RGB.
        image = np.frombuffer(data.raw_data, dtype=np.uint8).reshape((data.height, data.width, 4))
        msg = self.bridge.cv2_to_imgmsg(image[:, :, :3], encoding="bgr8")  # <- was "rgb8"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = actor.attributes.get('role_name', 'camera_frame')

        self.camera_pub.publish(msg)
        self.broadcast_transform(actor)

        # Publish CameraInfo with explicit casts
        self.publish_camera_info(
            msg.header.frame_id,
            int(data.width),
            int(data.height),
            float(actor.attributes.get('fov', 90.0)))
        # Publish CameraInfo
        self.publish_camera_info(msg.header.frame_id, data.width, data.height, float(actor.attributes.get('fov', 90)))

    def gnss_callback(self, data, actor):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = actor.attributes.get('role_name', 'gnss_frame')
        fix.latitude = data.latitude
        fix.longitude = data.longitude
        fix.altitude = data.altitude

        self.gnss_pub.publish(fix)
        self.broadcast_transform(actor)

    def shutdown(self, sig, frame):
        self.get_logger().info("Shutting down. Cleaning up actors...")
        for actor in self.actors:
            try:
                actor.destroy()
            except Exception as e:
                self.get_logger().warn(f"Failed to destroy actor: {e}")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaSensorPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

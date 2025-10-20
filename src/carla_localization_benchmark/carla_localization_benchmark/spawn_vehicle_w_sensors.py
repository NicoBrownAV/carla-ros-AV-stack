#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Imu, Image, PointCloud2, PointField, NavSatFix, CameraInfo
from geometry_msgs.msg import TransformStamped, TwistStamped
from visualization_msgs.msg import Marker
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
        # Remember local (vehicle-relative) transforms for TF broadcasting
        self.sensor_rel = {}

        # === ROS Publishers ===
        self.lidar_pub = self.create_publisher(PointCloud2, '/vehicle_lidar/points', 10)
        self.radar_pub = self.create_publisher(PointCloud2, '/vehicle_radar/detections', 10)
        self.camera_pub = self.create_publisher(Image, '/vehicle_camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/vehicle_camera/camera_info', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, '/vehicle_gnss/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/vehicle_imu/data', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/vehicle/speed', 10)
        self.speed_marker_pub = self.create_publisher(Marker, '/viz/speed_text', 10)

        # === Auto Pilot Parameters ===
        self.declare_parameter('use_autopilot', True)
        self.declare_parameter('tm_global_speed_reduction', 10.0)   # percent slower than limit (0 = speed limit)
        self.declare_parameter('tm_vehicle_speed_reduction', 0.0)   # percent slower for this vehicle only
        self.declare_parameter('tm_min_dist', 2.5)                  # meters to leading vehicle
        self.declare_parameter('tm_auto_lane_change', True)
        self.declare_parameter('tm_ignore_traffic_lights', False)
        self.declare_parameter('tm_ignore_signs', False)
        self.declare_parameter('tm_hybrid_physics', False)           # safer for big maps
        self.declare_parameter('tm_port', 8000)                     # default TM port

        # === Load sensor config ===
        self.sensor_cfg = self.load_sensor_config()

        # === Spawn Vehicle and Sensors ===
        self.spawn_vehicle_and_sensors()

        # === Graceful shutdown on Ctrl+C ===
        signal.signal(signal.SIGINT, self.shutdown)

        self.get_logger().info("Vehicle and sensors running. Ctrl+C to exit.")
    
    def load_sensor_config(self):
        cfg_path = os.path.join(
            get_package_share_directory('carla_localization_benchmark'),
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

    def _find_existing_hero(self):
        for v in self.world.get_actors().filter('vehicle.*'):
            if v.attributes.get('role_name', '') == 'hero':
                return v
        return None

    def _try_spawn_ego(self, vehicle_bp):
        """Try to spawn ego by walking through spawn points (collision-safe)."""
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found in current map.")
        self.get_logger().info(f"Found {len(spawn_points)} spawn points.")
        for sp in spawn_points:
            actor = self.world.try_spawn_actor(vehicle_bp, sp)
            if actor is not None:
                self.get_logger().info(f"Spawned ego at {sp.location} with role_name=hero.")
                return actor
        return None
    
    def _follow_with_spectator(self):
        try:
            spectator = self.world.get_spectator()

            def tick():
                tf = self.vehicle.get_transform()

                # Desired spectator position (behind and above the car)
                desired = tf.transform(carla.Location(x=-15.0, z=4.5))

                # Current spectator location
                current = spectator.get_transform().location

                # Smooth interpolation factor (0–1, smaller = smoother/slower)
                alpha = 0.15

                smoothed = carla.Location(
                    x=current.x + alpha * (desired.x - current.x),
                    y=current.y + alpha * (desired.y - current.y),
                    z=current.z + alpha * (desired.z - current.z)
                )

                spectator.set_transform(carla.Transform(
                    smoothed,
                    carla.Rotation(
                        pitch=tf.rotation.pitch - 10.0,
                        yaw=tf.rotation.yaw,
                        roll=tf.rotation.roll
                    )
                ))

            # call once immediately, then keep following at 25 Hz
            tick()
            self._spec_timer = self.create_timer(0.04, tick)
            self.get_logger().info("Spectator following hero (smoothed).")

        except Exception as e:
            self.get_logger().warn(f"Could not move spectator: {e}")

    def spawn_vehicle_and_sensors(self):
        # === Ego Vehicle ===
        # Reuse existing hero if present
        hero = self._find_existing_hero()
        if hero is not None:
            self.vehicle = hero
            self.get_logger().info("Reusing existing ego vehicle with role_name=hero.")
        else:
            vehicle_bp = self.bp_lib.filter("vehicle.tesla.model3")[0]
            vehicle_bp.set_attribute('role_name', 'hero')

            # Try collision-safe spawning first
            self.vehicle = self._try_spawn_ego(vehicle_bp)
            if self.vehicle is None:
                # Fallback (shouldn’t normally be needed): force spawn at index 0 (may collide)
                spawn_point = self.world.get_map().get_spawn_points()[0]
                self.get_logger().warn("All spawn points busy; attempting direct spawn at index 0 (may collide).")
                self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)

            if self.vehicle is None:
                raise RuntimeError("Failed to spawn ego vehicle: all spawn attempts failed.")
            self.get_logger().info("Spawned ego vehicle.")

        self.actors.append(self.vehicle)

        # --- AUTOPILOT SETUP ---
        if bool(self.get_parameter('use_autopilot').value):
            wanted_port = int(self.get_parameter('tm_port').value)
            try:
                tm = self.client.get_trafficmanager(wanted_port)
            except RuntimeError as e:
                self.get_logger().warn(
                    f"Traffic Manager bind failed on port {wanted_port}: {e}. "
                    f"Retrying with auto-selected port 0."
                )
                tm = self.client.get_trafficmanager(0)  # let CARLA pick a free port

            # Read TM params
            tm_global   = float(self.get_parameter('tm_global_speed_reduction').value)
            tm_vehicle  = float(self.get_parameter('tm_vehicle_speed_reduction').value)
            tm_min_dist = float(self.get_parameter('tm_min_dist').value)
            tm_auto_ln  = bool(self.get_parameter('tm_auto_lane_change').value)
            tm_ign_l    = bool(self.get_parameter('tm_ignore_traffic_lights').value)
            tm_ign_s    = bool(self.get_parameter('tm_ignore_signs').value)
            tm_hybrid   = bool(self.get_parameter('tm_hybrid_physics').value)

            # Configure TM
            tm.set_hybrid_physics_mode(tm_hybrid)
            tm.global_percentage_speed_difference(tm_global)
            tm.vehicle_percentage_speed_difference(self.vehicle, tm_vehicle)
            tm.distance_to_leading_vehicle(self.vehicle, tm_min_dist)
            tm.auto_lane_change(self.vehicle, tm_auto_ln)
            tm.ignore_lights_percentage(self.vehicle, 100 if tm_ign_l else 0)
            tm.ignore_signs_percentage(self.vehicle, 100 if tm_ign_s else 0)

            # Attach autopilot using the actual port the TM ended up using
            assigned_port = tm.get_port()
            self.vehicle.set_autopilot(True, assigned_port)
            self.get_logger().info(f"[TM] autopilot requested ON (TM port {assigned_port})")

        else:
            self.vehicle.set_autopilot(False)
            self.get_logger().info("Autopilot disabled (use_autopilot:=false)")

        # === Sensors ===
        for sensor_name, params in self.sensor_cfg.items():
            # Skip non-dicts or entries missing a blueprint field
            bp = self.bp_lib.find(params['blueprint'])
            if sensor_name == 'camera':
                transform = carla.Transform(carla.Location(x=1.0, z=1.6))
                if bp.has_attribute('role_name'):
                    bp.set_attribute('role_name', 'camera')
                sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
                self.sensor_rel[sensor.id] = transform
                sensor.listen(lambda data, s=sensor: self.camera_callback(data, s))
                self.camera = sensor

            elif sensor_name == 'lidar':
                transform = carla.Transform(carla.Location(x=0.0, z=2.0))
                if bp.has_attribute('role_name'):
                    bp.set_attribute('role_name', 'lidar')
                sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
                self.sensor_rel[sensor.id] = transform
                sensor.listen(lambda data, s=sensor: self.lidar_callback(data, s))
                self.lidar = sensor

            elif sensor_name == 'gnss':
                transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.9))
                if bp.has_attribute('role_name'):
                    bp.set_attribute('role_name', 'gnss')
                sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
                self.sensor_rel[sensor.id] = transform
                sensor.listen(lambda data, s=sensor: self.gnss_callback(data, s))
                self.gnss = sensor

            elif sensor_name == 'imu':
                transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.0))
                if bp.has_attribute('role_name'):
                    bp.set_attribute('role_name', 'imu')
                sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
                self.sensor_rel[sensor.id] = transform
                sensor.listen(lambda data, s=sensor: self.imu_callback(data, s))
                self.imu = sensor

            elif sensor_name == 'radar':
                transform = carla.Transform(carla.Location(x=2.25, z=0.75))
                if bp.has_attribute('role_name'):
                    bp.set_attribute('role_name', 'radar')
                sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
                self.sensor_rel[sensor.id] = transform
                sensor.listen(lambda data, s=sensor: self.radar_callback(data, s))
                self.radar = sensor

            else:
                continue

            self.actors.append(sensor)
            self.get_logger().info(f"Attached {sensor_name} sensor.")

        # start periodic speed publisher (20 Hz)
        self._speed_timer = self.create_timer(0.05, self._publish_speed)

        self._follow_with_spectator()

    def broadcast_transform(self, sensor_actor):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'base_link'  # parent: vehicle body frame

        child = sensor_actor.attributes.get('role_name', sensor_actor.type_id)
        tf_msg.child_frame_id = child

        # Use the relative transform we used when attaching the sensor to the vehicle
        rel = self.sensor_rel.get(sensor_actor.id, carla.Transform())

        # Translation (meters) relative to base_link
        tf_msg.transform.translation.x = float(rel.location.x)
        tf_msg.transform.translation.y = float(rel.location.y)
        tf_msg.transform.translation.z = float(rel.location.z)

        # Rotation: CARLA uses degrees; convert to quaternion
        r = rel.rotation
        # Yaw-Pitch-Roll (Z-Y-X) intrinsic → quaternion
        cy = math.cos(math.radians(r.yaw)   * 0.5)
        sy = math.sin(math.radians(r.yaw)   * 0.5)
        cp = math.cos(math.radians(r.pitch) * 0.5)
        sp = math.sin(math.radians(r.pitch) * 0.5)
        cr = math.cos(math.radians(r.roll)  * 0.5)
        sr = math.sin(math.radians(r.roll)  * 0.5)

        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        qw = cr*cp*cy + sr*sp*sy

        tf_msg.transform.rotation.x = float(qx)
        tf_msg.transform.rotation.y = float(qy)
        tf_msg.transform.rotation.z = float(qz)
        tf_msg.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(tf_msg)


    def radar_callback(self, radar_data, actor):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = actor.attributes.get('role_name', 'radar_frame')

        points = []
        for detection in radar_data:
            # Spherical to Cartesian
            azimuth = detection.azimuth
            altitude = detection.altitude
            r = detection.depth
            x = r * np.cos(altitude) * np.cos(azimuth)
            y = -1 * (r * np.cos(altitude) * np.sin(azimuth))
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

        raw = np.frombuffer(data.raw_data, dtype=np.float32)
        n = raw.size
        if n == 0:
            return
        if n % 4 != 0:
            self.get_logger().warn(f"LiDAR packet size {n} not divisible by 4; skipping frame")
            return

        # [x, y, z, intensity] in CARLA sensor frame.
        points = raw.reshape(-1, 4).copy()  # copy → writable
        # CARLA: y right -> ROS: y left
        points[:, 1] *= -1.0

        # Ensure proper dtype/contiguity for tobytes()
        payload = np.asarray(points, dtype=np.float32, order='C')

        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16
        width = payload.shape[0]

        cloud = PointCloud2(
            header=header,
            height=1,
            width=width,
            fields=fields,
            is_bigendian=False,
            point_step=point_step,
            row_step=point_step * width,
            is_dense=True,                     # set False if you ever include NaNs
            data=payload.tobytes(),
        )

        self.lidar_pub.publish(cloud)
        self.broadcast_transform(actor)

    def publish_camera_info(self, frame_id: str, width: int, height: int, fov_deg: float):
        width = int(width); height = int(height); fov_deg = float(fov_deg)

        cam_info = CameraInfo()
        cam_info.header.stamp = self.get_clock().now().to_msg()
        cam_info.header.frame_id = str(frame_id)
        cam_info.width = width
        cam_info.height = height
        cam_info.distortion_model = 'plumb_bob'
        cam_info.d = []  # or [0.0]*5

        fov_rad = math.radians(fov_deg)
        fx = (width / 2.0) / math.tan(fov_rad / 2.0)
        fy = fx
        cx = (width  - 1) / 2.0
        cy = (height - 1) / 2.0

        cam_info.k = [
            float(fx), 0.0,       float(cx),
            0.0,       float(fy), float(cy),
            0.0,       0.0,       1.0,
        ]
        cam_info.p = [
            float(fx), 0.0,       float(cx), 0.0,
            0.0,       float(fy), float(cy), 0.0,
            0.0,       0.0,       1.0,       0.0,
        ]
        cam_info.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]

        self.camera_info_pub.publish(cam_info)

    def camera_callback(self, data, actor):
        # CARLA gives BGRA; publish as bgr8 (or reorder to rgb8 if you prefer)
        image = np.frombuffer(data.raw_data, dtype=np.uint8).reshape((data.height, data.width, 4))
        msg = self.bridge.cv2_to_imgmsg(image[:, :, :3], encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = actor.attributes.get('role_name', 'camera_frame')

        self.camera_pub.publish(msg)
        self.broadcast_transform(actor)

        self.publish_camera_info(
            msg.header.frame_id,
            int(data.width),
            int(data.height),
            float(actor.attributes.get('fov', 90.0))
        )

    def gnss_callback(self, data, actor):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = actor.attributes.get('role_name', 'gnss_frame')
        fix.latitude = data.latitude
        fix.longitude = data.longitude
        fix.altitude = data.altitude

        self.gnss_pub.publish(fix)
        self.broadcast_transform(actor)

    def imu_callback(self, data, actor):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = actor.attributes.get('role_name', 'imu_frame')

        # Orientation: unknown (we don't have a quaternion here)
        imu_msg.orientation_covariance[0] = -1.0

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = data.gyroscope.x
        imu_msg.angular_velocity.y = data.gyroscope.y
        imu_msg.angular_velocity.z = data.gyroscope.z
        imu_msg.angular_velocity_covariance = [0.0]*9

        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = data.accelerometer.x
        imu_msg.linear_acceleration.y = data.accelerometer.y
        imu_msg.linear_acceleration.z = data.accelerometer.z
        imu_msg.linear_acceleration_covariance = [0.0]*9

        self.imu_pub.publish(imu_msg)
        self.broadcast_transform(actor)

    def _publish_speed(self):
        if not hasattr(self, 'vehicle') or self.vehicle is None:
            return

        # Current transform & world-frame velocities (CARLA: x fwd, y right, z up)
        tf = self.vehicle.get_transform()
        v_w = self.vehicle.get_velocity()
        ang_w = self.vehicle.get_angular_velocity()

        # Yaw (degrees -> radians)
        yaw_rad = math.radians(tf.rotation.yaw)

        # Rotate world linear velocity into VEHICLE BODY frame:
        # v_b = R^T * v_w for yaw-only rotation
        # [vx_b]   [ cosψ  sinψ]*[vx_w]
        # [vy_b] = [-sinψ  cosψ]*[vy_w]
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        vx_b =  c * v_w.x + s * v_w.y
        vy_b = -s * v_w.x + c * v_w.y
        vz_b = v_w.z  # near zero in most driving; keep as-is

        speed_mps = math.sqrt(vx_b*vx_b + vy_b*vy_b + vz_b*vz_b)

        # --- Publish TwistStamped (body-frame velocity) ---
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = float(vx_b)  # forward
        twist.twist.linear.y = float(vy_b)  # left (+)
        twist.twist.linear.z = float(vz_b)  # up
        # If you want yaw-rate too (about body Z), ang_w.z ~ yaw rate (rad/s)
        twist.twist.angular.z = float(ang_w.z)
        self.vel_pub.publish(twist)

        # --- Publish RViz speed text marker anchored to the vehicle frame ---
        # Pose is RELATIVE to base_frame; keep text above the roof.
        m = Marker()
        m.header.stamp = twist.header.stamp
        m.header.frame_id = 'base_link'
        m.ns = 'speed_text'
        m.id = 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 2.5  # above vehicle
        m.pose.orientation.w = 1.0

        mph = speed_mps * 2.23693629
        m.text = f"{speed_mps:4.1f} m/s  ({mph:4.1f} mph)"
        m.scale.z = 1.0  # font height in meters
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.95
        m.lifetime = Duration(seconds=0.2).to_msg()

        self.speed_marker_pub.publish(m)

    def shutdown(self, *args):
        # Graceful cleanup called by SIGINT
        self.get_logger().info("Shutting down: cancel timers and destroy actors...")
        try:
            if hasattr(self, '_spec_timer') and self._spec_timer:
                self._spec_timer.cancel()
        except Exception:
            pass
        try:
            if hasattr(self, '_speed_timer') and self._speed_timer:
                self._speed_timer.cancel()
        except Exception:
            pass
        # Destroy sensors/vehicle (reverse order just in case)
        try:
            for a in getattr(self, 'actors', [])[::-1]:
                try:
                    a.destroy()
                except Exception:
                    pass
            self.get_logger().info("All CARLA actors destroyed.")
        except Exception as e:
            self.get_logger().warn(f"Error destroying actors: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CarlaSensorPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

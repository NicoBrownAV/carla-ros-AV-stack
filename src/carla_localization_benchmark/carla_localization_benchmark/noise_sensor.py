import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from .utils import BiasRandomWalk, append_path, wrap_angle

def yaw_from_quat(q: Quaternion):
    # simplest 2D yaw
    z, w = q.z, q.w
    return math.atan2(2.0*z*w, 1.0 - 2.0*z*z)

class NoiseSensors(Node):
    def __init__(self):
        super().__init__('noise_sensors')
        p = self.get_parameter
        self.declare_parameters('', [
            ('gt_odom_topic', '/carla/ego/ground_truth/odom'),
            ('noisy_odom_topic', '/sim/gps/odom'),
            ('noisy_imu_topic', '/sim/imu'),
            ('gps_path_topic', '/viz/gps_path'),
            ('gps_xy_std_m', 0.8),
            ('gps_bias_std_m', 1.5),
            ('imu_gyro_std', 0.01),
            ('imu_gyro_bias_std', 0.005),
            ('imu_ax_std', 0.3),
            ('imu_ax_bias_std', 0.1),
            ('gps_rate_hz', 10.0),
            ('imu_rate_hz', 100.0),
            ('publish_paths', True),
        ])
        self.gt_topic = p('gt_odom_topic').value
        self.noisy_odom_topic = p('noisy_odom_topic').value
        self.noisy_imu_topic = p('noisy_imu_topic').value
        self.gps_path_topic = p('gps_path_topic').value
        self.gps_xy_std = float(p('gps_xy_std_m').value)
        self.gps_bias_std = float(p('gps_bias_std_m').value)
        self.imu_gyro_std = float(p('imu_gyro_std').value)
        self.imu_gyro_bias_std = float(p('imu_gyro_bias_std').value)
        self.imu_ax_std = float(p('imu_ax_std').value)
        self.imu_ax_bias_std = float(p('imu_ax_bias_std').value)
        self.gps_dt = 1.0/float(p('gps_rate_hz').value)
        self.imu_dt = 1.0/float(p('imu_rate_hz').value)
        self.publish_paths = bool(p('publish_paths').value)

        self.sub_gt = self.create_subscription(Odometry, self.gt_topic, self.on_gt, 20)
        self.pub_gps = self.create_publisher(Odometry, self.noisy_odom_topic, 10)
        self.pub_imu = self.create_publisher(Imu, self.noisy_imu_topic, 50)
        self.pub_gps_path = self.create_publisher(Path, self.gps_path_topic, 1) if self.publish_paths else None

        # Bias random walks
        self.x_bias_rw = BiasRandomWalk(self.gps_bias_std, self.gps_dt)
        self.y_bias_rw = BiasRandomWalk(self.gps_bias_std, self.gps_dt)
        self.gyro_bias_rw = BiasRandomWalk(self.imu_gyro_bias_std, self.imu_dt)
        self.ax_bias_rw = BiasRandomWalk(self.imu_ax_bias_std, self.imu_dt)

        self.last_gt = None
        self.last_gps_t = 0.0
        self.last_imu_t = 0.0

        self.gps_path = Path()

        # Timers to emit IMU/GPS even if GT is slow
        self.create_timer(self.imu_dt, self.tick_imu)
        self.create_timer(self.gps_dt, self.tick_gps)

    def on_gt(self, msg: Odometry):
        self.last_gt = msg

    def tick_gps(self):
        if self.last_gt is None: return
        gt = self.last_gt
        hdr = gt.header
        x = gt.pose.pose.position.x
        y = gt.pose.pose.position.y

        # Gaussian noise + slow bias drift
        nx = np.random.normal(0.0, self.gps_xy_std) + self.x_bias_rw.step()
        ny = np.random.normal(0.0, self.gps_xy_std) + self.y_bias_rw.step()

        gps = Odometry()
        gps.header = hdr
        gps.child_frame_id = 'base_link'
        gps.pose.pose.position.x = x + nx
        gps.pose.pose.position.y = y + ny
        gps.pose.pose.position.z = gt.pose.pose.position.z
        gps.pose.pose.orientation = gt.pose.pose.orientation  # GPS does NOT give yaw, but keep orientation to visualize
        # Very loose covariance (position only)
        cov = [0.0]*36
        cov[0] = cov[7] = self.gps_xy_std**2 + self.gps_bias_rw_var()
        cov[14] = 3.0**2
        gps.pose.covariance = cov
        self.pub_gps.publish(gps)

        if self.pub_gps_path:
            self.gps_path = append_path(self.gps_path, hdr.stamp, hdr.frame_id,
                                        gps.pose.pose.position.x, gps.pose.pose.position.y,
                                        yaw_from_quat(gps.pose.pose.orientation))
            self.pub_gps_path.publish(self.gps_path)

    def gps_bias_rw_var(self):
        # crude visualization variance; not used by filter (filter uses params)
        return (self.gps_bias_std**2)

    def tick_imu(self):
        if self.last_gt is None: return
        gt = self.last_gt
        hdr = gt.header
        yaw = yaw_from_quat(gt.pose.pose.orientation)
        vx = gt.twist.twist.linear.x
        vy = gt.twist.twist.linear.y
        v = math.hypot(vx, vy)
        yaw_rate_true = gt.twist.twist.angular.z
        # simple longitudinal accel estimate (derivative in GT can be noisy; use twist linear accel if present)
        ax_true = gt.twist.twist.linear.x  # CARLA Odom sometimes stores body-frame accel here; OK for demo

        gyro_z = yaw_rate_true + np.random.normal(0.0, self.imu_gyro_std) + self.gyro_bias_rw.step()
        ax = ax_true + np.random.normal(0.0, self.imu_ax_std) + self.ax_bias_rw.step()

        imu = Imu()
        imu.header = hdr
        imu.orientation = gt.pose.pose.orientation  # we won’t use absolute orientation in EKF; use gyro for yaw
        imu.angular_velocity.z = gyro_z
        imu.linear_acceleration.x = ax
        # covariances (diagonal)
        imu.angular_velocity_covariance = [0.0]*9
        imu.angular_velocity_covariance[8] = self.imu_gyro_std**2
        imu.linear_acceleration_covariance = [0.0]*9
        imu.linear_acceleration_covariance[0] = self.imu_ax_std**2
        self.pub_imu.publish(imu)

def main():
    rclpy.init()
    rclpy.spin(NoiseSensors())
    rclpy.shutdown()


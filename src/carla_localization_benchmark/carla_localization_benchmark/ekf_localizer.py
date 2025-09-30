import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion
from .utils import append_path, wrap_angle

def yaw_from_quat(q: Quaternion):
    z, w = q.z, q.w
    return math.atan2(2.0*z*w, 1.0 - 2.0*z*z)

class EKFLocalizer(Node):
    """
    State: [x, y, yaw, v]
    Inputs:
      - IMU: yaw_rate (r), ax (longitudinal accel in body)
      - GPS: position (x,y)
    Discrete bicycle-lite kinematics (flat, small dt).
    """
    def __init__(self):
        super().__init__('ekf_localizer')
        self.declare_parameters('', [
            ('gt_odom_topic', '/carla/ego/ground_truth/odom'),
            ('noisy_odom_topic', '/sim/gps/odom'),
            ('noisy_imu_topic', '/sim/imu'),
            ('fused_odom_topic', '/fusion/odom'),
            ('gt_path_topic', '/viz/gt_path'),
            ('ekf_path_topic', '/viz/ekf_path'),
            ('ekf_rate_hz', 50.0),
            ('publish_paths', True),
            ('live_plot', False),
            ('csv_log_path', ''),
        ])
        p = self.get_parameter
        self.gt_topic = p('gt_odom_topic').value
        self.gps_topic = p('noisy_odom_topic').value
        self.imu_topic = p('noisy_imu_topic').value
        self.out_topic = p('fused_odom_topic').value
        self.gt_path_topic = p('gt_path_topic').value
        self.ekf_path_topic = p('ekf_path_topic').value
        self.dt = 1.0/float(p('ekf_rate_hz').value)
        self.publish_paths = bool(p('publish_paths').value)
        self.live_plot = bool(p('live_plot').value)
        self.csv_path = p('csv_log_path').value

        self.sub_gps = self.create_subscription(Odometry, self.gps_topic, self.on_gps, 10)
        self.sub_imu = self.create_subscription(Odometry, self.gt_topic, self.on_gt, 10)  # only for GT path & timing ref
        self.sub_imu_raw = self.create_subscription_type_erased(self.imu_topic)  # subscribe lazy in callback

        self.pub_odom = self.create_publisher(Odometry, self.out_topic, 10)
        self.pub_gt_path = self.create_publisher(Path, self.gt_path_topic, 1) if self.publish_paths else None
        self.pub_ekf_path = self.create_publisher(Path, self.ekf_path_topic, 1) if self.publish_paths else None

        # EKF state
        self.x = np.zeros((4,1))          # [x,y,yaw,v]
        self.P = np.diag([1.0,1.0,(10*math.pi/180)**2, 1.0])
        self.Q = np.diag([0.05, 0.05, (0.5*math.pi/180)**2, 0.5])  # process noise
        self.R_gps = np.diag([1.5**2, 1.5**2])  # measurement noise

        self.last_time = None
        self.last_imu = {'r':0.0, 'ax':0.0}
        self.have_gps = False

        self.gt_path = Path()
        self.ekf_path = Path()

        # live plotting (optional)
        self.plotter = None
        if self.live_plot:
            try:
                import matplotlib.pyplot as plt
                self.plt = plt
                self.fig, self.ax = plt.subplots()
                self.ax.set_aspect('equal')
                self.ax.set_title('GT (black) vs EKF (blue) vs GPS (gray)')
                self.gt_xy, = self.ax.plot([], [], '.', alpha=0.8)
                self.ekf_xy, = self.ax.plot([], [], '.', alpha=0.8)
                self.gps_xy, = self.ax.plot([], [], '.', alpha=0.4)
                self.gt_hist = []
                self.ekf_hist = []
                self.gps_hist = []
                plt.ion(); plt.show()
            except Exception as e:
                self.get_logger().warn(f"Matplotlib disabled: {e}")

        # CSV logging
        self.csv = None
        if self.csv_path:
            self.csv = open(self.csv_path, 'w')
            self.csv.write("stamp_ns,x_gt,y_gt,x_gps,y_gps,x_ekf,y_ekf,yaw_ekf,v_ekf,dt_ms,latency_ms\n")

        # EKF predict timer (runs even if GPS intermittent)
        self.create_timer(self.dt, self.predict_step)

    # Type-erased sub: we need sensor_msgs/Imu but don’t want extra import here
    def create_subscription_type_erased(self, topic):
        from rclpy.qos import QoSProfile
        return self.create_subscription(
            msg_type=object, topic=topic, callback=self.on_imu, qos_profile=QoSProfile(depth=50)
        )

    def on_gt(self, msg: Odometry):
        if self.pub_gt_path:
            yaw = yaw_from_quat(msg.pose.pose.orientation)
            self.gt_path = append_path(self.gt_path, msg.header.stamp, msg.header.frame_id,
                                       msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            self.pub_gt_path.publish(self.gt_path)

    def on_gps(self, msg: Odometry):
        self.gps_msg = msg
        self.have_gps = True
        if self.live_plot and hasattr(self, 'gps_hist'):
            self.gps_hist.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def on_imu(self, msg):
        # Extract only needed fields from Imu
        try:
            imu = msg  # sensor_msgs/Imu
            self.last_imu['r'] = imu.angular_velocity.z
            self.last_imu['ax'] = imu.linear_acceleration.x
        except Exception:
            pass

    def predict_step(self):
        t_start = time.perf_counter_ns()

        r = self.last_imu['r']
        ax = self.last_imu['ax']

        x, y, yaw, v = self.x.flatten()
        dt = self.dt

        # Simple kinematics
        yaw_new = wrap_angle(yaw + r*dt)
        v_new   = v + ax*dt
        x_new   = x + v_new * math.cos(yaw_new) * dt
        y_new   = y + v_new * math.sin(yaw_new) * dt

        F = np.eye(4)
        F[0,2] = -v_new * math.sin(yaw_new) * dt
        F[0,3] = math.cos(yaw_new) * dt
        F[1,2] =  v_new * math.cos(yaw_new) * dt
        F[1,3] = math.sin(yaw_new) * dt
        F[2,2] = 1.0
        F[3,3] = 1.0

        self.x = np.array([[x_new], [y_new], [yaw_new], [v_new]])
        self.P = F @ self.P @ F.T + self.Q

        # GPS update if available
        if self.have_gps:
            z = np.array([[self.gps_msg.pose.pose.position.x],
                          [self.gps_msg.pose.pose.position.y]])
            H = np.array([[1,0,0,0],
                          [0,1,0,0]])
            yk = z - H @ self.x
            S = H @ self.P @ H.T + self.R_gps
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x = self.x + K @ yk
            self.P = (np.eye(4) - K @ H) @ self.P

        # Publish fused odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        yaw = float(self.x[2])
        odom.pose.pose.orientation.z = math.sin(yaw/2.0)
        odom.pose.pose.orientation.w = math.cos(yaw/2.0)
        odom.twist.twist.linear.x = float(self.x[3])
        self.pub_odom.publish(odom)

        if self.publish_paths and self.pub_ekf_path:
            self.ekf_path = append_path(self.ekf_path, odom.header.stamp, odom.header.frame_id,
                                        float(self.x[0]), float(self.x[1]), yaw)
            self.pub_ekf_path.publish(self.ekf_path)

        # Live plot
        if self.live_plot and hasattr(self, 'gt_hist') and hasattr(self, 'gps_hist'):
            self.ekf_hist.append((float(self.x[0]), float(self.x[1])))
            if len(self.gt_path.poses) > 0:
                self.gt_hist.append((self.gt_path.poses[-1].pose.position.x,
                                     self.gt_path.poses[-1].pose.position.y))
            self._refresh_plot()

        dt_ms = (time.perf_counter_ns() - t_start)/1e6
        # Latency proxy: dt_ms includes predict+update; you can refine if needed
        if self.csv:
            x_gt = self.gt_path.poses[-1].pose.position.x if len(self.gt_path.poses) else float('nan')
            y_gt = self.gt_path.poses[-1].pose.position.y if len(self.gt_path.poses) else float('nan')
            x_gps = self.gps_msg.pose.pose.position.x if self.have_gps else float('nan')
            y_gps = self.gps_msg.pose.pose.position.y if self.have_gps else float('nan')
            self.csv.write(f"{self.get_clock().now().nanoseconds},{x_gt},{y_gt},{x_gps},{y_gps},{float(self.x[0])},{float(self.x[1])},{yaw},{float(self.x[3])},{self.dt*1000.0:.3f},{dt_ms:.3f}\n")
            self.csv.flush()

    def _refresh_plot(self):
        try:
            gx, gy = zip(*self.gt_hist) if self.gt_hist else ([],[])
            ex, ey = zip(*self.ekf_hist) if self.ekf_hist else ([],[])
            px, py = zip(*self.gps_hist) if self.gps_hist else ([],[])
            self.gt_xy.set_data(gx, gy)
            self.ekf_xy.set_data(ex, ey)
            self.gps_xy.set_data(px, py)
            self.ax.relim(); self.ax.autoscale_view()
            self.plt.pause(0.001)
        except Exception:
            pass

def main():
    rclpy.init()
    rclpy.spin(EKFLocalizer())
    rclpy.shutdown()


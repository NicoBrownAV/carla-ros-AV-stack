# measurement_mux.py  (drop-in replacement of the class)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu

class MeasurementMux(Node):
    """
    Latch /vehicle/speed (TwistStamped, base_link) and /imu/data (Imu),
    publish /ekf/inputs (TwistStamped) with:
      header.stamp = max(stamp(speed), stamp(imu))
      linear.x = v (m/s)   [0 if stale > thresh]
      angular.z = r (rad/s)[0 if stale > thresh]
    """
    def __init__(self):
        super().__init__('measurement_mux')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('stale_thresh_s', 0.25)
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.stale_thresh = float(self.get_parameter('stale_thresh_s').value)

        self.v = 0.0
        self.r = 0.0
        self.speed_stamp = None
        self.imu_stamp = None

        self.sub_speed = self.create_subscription(TwistStamped, '/vehicle/speed', self.on_speed, 50)
        self.sub_imu   = self.create_subscription(Imu, '/imu/data', self.on_imu, 100)

        self.pub_inputs = self.create_publisher(TwistStamped, '/ekf/inputs', 20)
        self.timer = self.create_timer(self.dt, self.tick)

    def on_speed(self, msg: TwistStamped):
        self.v = float(msg.twist.linear.x)
        self.speed_stamp = msg.header.stamp  # builtin_interfaces/Time

    def on_imu(self, msg: Imu):
        self.r = float(msg.angular_velocity.z)
        self.imu_stamp = msg.header.stamp

    def _to_ns(self, t):
        return t.sec * 1_000_000_000 + t.nanosec

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        v_use = self.v
        r_use = self.r

        # choose newest source time as header.stamp
        stamp = self.speed_stamp or self.imu_stamp
        if self.speed_stamp and self.imu_stamp:
            stamp = self.speed_stamp if self._to_ns(self.speed_stamp) >= self._to_ns(self.imu_stamp) else self.imu_stamp

        # stale checks
        if self.speed_stamp is None or (now_ns - self._to_ns(self.speed_stamp)) * 1e-9 > self.stale_thresh:
            v_use = 0.0
        if self.imu_stamp is None or (now_ns - self._to_ns(self.imu_stamp)) * 1e-9 > self.stale_thresh:
            r_use = 0.0

        # publish
        out = TwistStamped()
        out.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = v_use
        out.twist.angular.z = r_use
        self.pub_inputs.publish(out)

def main():
    rclpy.init()
    rclpy.spin(MeasurementMux())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

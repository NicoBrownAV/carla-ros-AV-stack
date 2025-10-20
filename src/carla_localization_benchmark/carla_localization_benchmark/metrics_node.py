#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped

class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_node')
        self.last_imu = None
        self.last_gps = None
        self.last_wheel = None
        self.gt_msg = None

        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.create_subscription(NavSatFix, '/gps/fix', self.on_gps, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.on_wheel, 20)
        self.create_subscription(Odometry, '/fusion/odom', self.on_fused, 20)
        self.create_subscription(Odometry, '/carla/ego/ground_truth/odom', self.on_gt, 20)
        self.create_subscription(TwistStamped, '/filter/compute_time', self.on_compute_time, 10)

    def to_ns(self, stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def on_imu(self, msg: Imu):
        self.last_imu = self.to_ns(msg.header.stamp)

    def on_gps(self, msg: NavSatFix):
        self.last_gps = self.to_ns(msg.header.stamp)

    def on_wheel(self, msg: Odometry):
        self.last_wheel = self.to_ns(msg.header.stamp)

    def on_gt(self, msg: Odometry):
        self.gt_msg = msg

    def on_compute_time(self, msg: TwistStamped):
        # msg.twist.linear.x holds compute duration in microseconds
        compute_us = msg.twist.linear.x
        self.get_logger().info(f'Filter compute time (us): {compute_us}')

    def on_fused(self, msg: Odometry):
        out_ns = self.to_ns(msg.header.stamp)
        inputs = [t for t in [self.last_imu, self.last_gps, self.last_wheel] if t is not None]
        if inputs:
            lat = out_ns - max(inputs)
            self.get_logger().info(f'Latency (ns): {lat}')

        if self.gt_msg:
            dx = msg.pose.pose.position.x - self.gt_msg.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.gt_msg.pose.pose.position.y
            dz = msg.pose.pose.position.z - self.gt_msg.pose.pose.position.z
            err = (dx*dx + dy*dy + dz*dz)**0.5
            self.get_logger().info(f'Pos error: {err}')

def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import csv
import os

class CsvLogger(Node):
    def __init__(self):
        super().__init__('csv_logger')
        self.create_subscription(Odometry, '/carla/ego/ground_truth/odom', self.on_gt, 10)
        self.create_subscription(Odometry, '/sim/gps/odom', self.on_gps, 10)
        self.create_subscription(Odometry, '/fusion/odom', self.on_ekf, 10)
        self.create_subscription(TwistStamped, '/ekf/diag', self.on_diag, 20)
        # Also subscribe to compute-time diagnostics
        self.create_subscription(TwistStamped, '/filter/compute_time', self.on_compute_time, 10)

        self.csv_path = os.path.expanduser('~/localization_log.csv')
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['topic', 'sec', 'nanosec', 'data'])

    def on_gt(self, msg: Odometry):
        h = msg.header.stamp
        p = msg.pose.pose.position
        self.writer.writerow(['gt', h.sec, h.nanosec, f'{p.x},{p.y},{p.z}'])
        self.csv_file.flush()

    def on_gps(self, msg: Odometry):
        h = msg.header.stamp
        p = msg.pose.pose.position
        self.writer.writerow(['gps', h.sec, h.nanosec, f'{p.x},{p.y},{p.z}'])
        self.csv_file.flush()

    def on_ekf(self, msg: Odometry):
        h = msg.header.stamp
        p = msg.pose.pose.position
        self.writer.writerow(['fusion', h.sec, h.nanosec, f'{p.x},{p.y},{p.z}'])
        self.csv_file.flush()

    def on_diag(self, msg: TwistStamped):
        h = msg.header.stamp
        tv = msg.twist
        self.writer.writerow(['diag', h.sec, h.nanosec, f'{tv.linear.x},{tv.linear.y},{tv.linear.z}'])
        self.csv_file.flush()

    def on_compute_time(self, msg: TwistStamped):
        h = msg.header.stamp
        tv = msg.twist
        self.writer.writerow(['compute_time_us', h.sec, h.nanosec, f'{tv.linear.x}'])
        self.csv_file.flush()

    def destroy_node(self):
        super().destroy_node()
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = CsvLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

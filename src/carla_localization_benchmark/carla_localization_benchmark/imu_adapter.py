#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuAdapter(Node):
    def __init__(self):
        super().__init__('imu_adapter')
        self.create_subscription(Imu, '/vehicle_imu/data', self.on_vehicle_imu, 10)
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

    def on_vehicle_imu(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ImuAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

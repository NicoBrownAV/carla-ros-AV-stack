#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GnssAdapter(Node):
    def __init__(self):
        super().__init__('gnss_adapter')
        self.create_subscription(NavSatFix, '/vehicle_gnss/fix', self.on_fix, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

    def on_fix(self, msg: NavSatFix):
        out = NavSatFix()
        out.header = msg.header
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        out.position_covariance = msg.position_covariance
        out.position_covariance_type = msg.position_covariance_type
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = GnssAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

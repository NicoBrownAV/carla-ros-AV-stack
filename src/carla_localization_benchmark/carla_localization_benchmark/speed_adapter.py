#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class SpeedAdapter(Node):
    def __init__(self):
        super().__init__('speed_adapter')
        self.create_subscription(TwistStamped, '/vehicle/speed', self.on_speed, 50)
        self.pub = self.create_publisher(Odometry, '/wheel/odom', 10)

    def on_speed(self, msg: TwistStamped):
        od = Odometry()
        od.header = msg.header
        od.child_frame_id = 'base_link'
        # Set pose to zero / identity
        od.pose.pose.position.x = 0.0
        od.pose.pose.position.y = 0.0
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation.w = 1.0
        od.twist.twist.linear = msg.twist.linear
        od.twist.twist.angular = msg.twist.angular
        self.pub.publish(od)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

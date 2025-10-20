#!/usr/bin/env python3

import rclpy, math, carla
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw/2.0)
    q.w = math.cos(yaw/2.0)
    return q

class EgoGTPub(Node):
    def __init__(self):
        super().__init__('ego_gt_odom')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('ego_role_name', 'hero')
        self.declare_parameter('rate_hz', 50.0)

        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.dt = 1.0/float(self.get_parameter('rate_hz').value)
        self.role = self.get_parameter('ego_role_name').value

        self.client = carla.Client(host, port); self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.ego = None

        # try to bind now; if not found, poll until it exists
        self._bind_ego()
        self.create_timer(0.5, self._bind_ego)  # keeps trying until found

        self.pub = self.create_publisher(Odometry, '/carla/ego/ground_truth/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(self.dt, self.tick)

    def _bind_ego(self):
        if self.ego is not None:
            return
        vehicles = self.world.get_actors().filter('vehicle.*')
        for v in vehicles:
            if v.attributes.get('role_name', '') == self.role:
                self.ego = v
                self.get_logger().info(f"ego_gt_odom bound to role_name={self.role}")
                return
        # No fallback to [0]; just keep waiting
        self.get_logger().warn("Waiting for ego with role_name=hero...")

    def tick(self):
        tr = self.ego.get_transform()
        vel = self.ego.get_velocity()
        ang = self.ego.get_angular_velocity()
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'map'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = tr.location.x
        od.pose.pose.position.y = tr.location.y
        od.pose.pose.position.z = tr.location.z
        yaw = math.radians(tr.rotation.yaw)
        od.pose.pose.orientation = yaw_to_quat(yaw)
        od.twist.twist.linear.x = vel.x
        od.twist.twist.linear.y = vel.y
        od.twist.twist.linear.z = vel.z
        od.twist.twist.angular.z = math.radians(ang.z)
        self.pub.publish(od)

                # --- Broadcast map -> base_link transform ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'base_link'

        # Position
        tf_msg.transform.translation.x = od.pose.pose.position.x
        tf_msg.transform.translation.y = od.pose.pose.position.y
        tf_msg.transform.translation.z = od.pose.pose.position.z

        # Orientation (already a quaternion in odom.pose.pose.orientation)
        q = od.pose.pose.orientation
        tf_msg.transform.rotation.x = q.x
        tf_msg.transform.rotation.y = q.y
        tf_msg.transform.rotation.z = q.z
        tf_msg.transform.rotation.w = q.w

        # Send the transform
        self.tf_broadcaster.sendTransform(tf_msg)

def main():
    rclpy.init(); rclpy.spin(EgoGTPub()); rclpy.shutdown()
if __name__ == '__main__': main()


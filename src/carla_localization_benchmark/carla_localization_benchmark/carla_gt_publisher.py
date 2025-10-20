#!/usr/bin/env python3

import rclpy, numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray, Header
from tf2_ros import Buffer, TransformListener
import carla

class CarlaGTPublisher(Node):
    def __init__(self):
        super().__init__('carla_gt_publisher')
        self.pub_poses = self.create_publisher(PoseArray, '/gt/poses', 5)
        self.pub_ids   = self.create_publisher(Int32MultiArray, '/gt/ids', 5)

        self.fixed_frame = 'base_link'
        self.world = carla.Client('localhost', 2000).get_world()
        self.tf_buf = Buffer(); self.tf = TransformListener(self.tf_buf, self)

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def tick(self):
        # TF: world → ego base_link
        try:
            tf = self.tf_buf.lookup_transform(self.fixed_frame, 'map', rclpy.time.Time())
        except Exception:
            # If you don’t use 'map', change to your world frame name in your TF tree.
            return

        T = self.tf_to_mat(tf.transform)

        vehicles = self.world.get_actors().filter('vehicle.*')
        poses = PoseArray(); poses.header = Header()
        poses.header.stamp = self.get_clock().now().to_msg()
        poses.header.frame_id = self.fixed_frame

        ids = []
        for v in vehicles:
            loc = v.get_location()
            pt = np.array([loc.x, loc.y, loc.z, 1.0], dtype=float)
            ego = T @ pt
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(ego[0]), float(ego[1]), float(ego[2])
            p.orientation.w = 1.0
            poses.poses.append(p)
            ids.append(int(v.id))

        self.pub_poses.publish(poses)
        out_ids = Int32MultiArray(); out_ids.data = ids
        self.pub_ids.publish(out_ids)

    @staticmethod
    def tf_to_mat(t):
        import tf_transformations as tft
        R = tft.quaternion_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        M = np.eye(4); M[:3,:3] = R[:3,:3]
        M[0,3] = t.translation.x; M[1,3] = t.translation.y; M[2,3] = t.translation.z
        return M

def main():
    rclpy.init(); rclpy.spin(CarlaGTPublisher()); rclpy.shutdown()
if __name__ == '__main__': main()


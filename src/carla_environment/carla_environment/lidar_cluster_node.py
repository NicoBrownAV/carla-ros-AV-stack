#!/usr/bin/env python3

import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, PoseArray
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft
from sklearn.cluster import DBSCAN

class LidarClusterNode(Node):
    def __init__(self):
        super().__init__('lidar_cluster_node')
        self.sub_pc = self.create_subscription(PointCloud2, '/vehicle_lidar/points', self.pc_cb, 5)
        self.sub_det = self.create_subscription(Detection2DArray, '/detections_2d', self.det_cb, 5)
        self.pub = self.create_publisher(PoseArray, '/detections_3d', 5)

        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)

        self.latest_det = None
        self.eps = 0.6  # meters
        self.min_samples = 8

        # camera intrinsics (from your CameraInfo)
        self.fx = None; self.fy = None; self.cx = None; self.cy = None
        self.camera_frame = 'camera_frame'
        self.fixed_frame  = 'base_link'

        # Optionally subscribe to CameraInfo to fill fx,fy,cx,cy dynamically
        from sensor_msgs.msg import CameraInfo
        self.create_subscription(CameraInfo, '/vehicle_camera/camera_info', self.caminfo_cb, 1)

    def caminfo_cb(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.camera_frame = msg.header.frame_id

    def det_cb(self, msg: Detection2DArray):
        self.latest_det = msg  # keep most recent

    def pc_cb(self, msg: PointCloud2):
        # Convert PC2 to Nx3
        pts = np.array(list(point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True)))
        if pts.size == 0:
            return

        # Cluster in LiDAR frame first
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(pts[:, :2])  # 2D XY clustering
        labels = clustering.labels_
        uniq = [l for l in set(labels) if l != -1]
        if not uniq:
            return

        # Transform cluster centroids to base_link
        try:
            tf = self.tf_buf.lookup_transform(self.fixed_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        T = self.tf_to_mat(tf.transform)

        cluster_centroids = []
        for lab in uniq:
            mask = (labels == lab)
            cluster = pts[mask]
            centroid = cluster.mean(axis=0)
            centroid_h = np.array([centroid[0], centroid[1], centroid[2], 1.0])
            ego_h = T @ centroid_h
            cluster_centroids.append(ego_h[:3])

        # If we have a camera det and intrinsics, pick the cluster whose projection best overlaps the bbox center
        chosen = None
        if self.latest_det and self.fx is not None:
            # Get bbox center of the most confident vehicle det
            dets = self.latest_det.detections
            if dets:
                det = max(dets, key=lambda d: (d.results[0].hypothesis.score if d.results else 0.0))
                u_center = det.bbox.center.x; v_center = det.bbox.center.y

                # Transform centroids to camera frame and project to pixels
                try:
                    tf_cam = self.tf_buf.lookup_transform(self.camera_frame, self.fixed_frame, rclpy.time.Time())
                    Tc = self.tf_to_mat(tf_cam.transform)
                except Exception as e:
                    self.get_logger().warn(f"TF to camera failed: {e}")
                    Tc = None

                if Tc is not None:
                    best_idx = None; best_err = 1e9
                    for i, c in enumerate(cluster_centroids):
                        c_h = np.array([c[0], c[1], c[2], 1.0])
                        cam = Tc @ c_h
                        X, Y, Z = cam[0], cam[1], cam[2]
                        if Z <= 0:  # behind camera
                            continue
                        u = self.fx * (X/Z) + self.cx
                        v = self.fy * (Y/Z) + self.cy
                        err = (u - u_center)**2 + (v - v_center)**2
                        if err < best_err:
                            best_err, best_idx = err, i
                    if best_idx is not None:
                        chosen = [cluster_centroids[best_idx]]

        # Publish all centroids (or only the chosen one if present)
        poses = PoseArray()
        poses.header.frame_id = self.fixed_frame
        poses.header.stamp = self.get_clock().now().to_msg()
        for c in (chosen if chosen else cluster_centroids):
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(c[0]), float(c[1]), float(c[2])
            poses.poses.append(p)
        self.pub.publish(poses)

    @staticmethod
    def tf_to_mat(t):
        # Geometry msg Transform -> 4x4 matrix
        import numpy as np
        q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        R = tft.quaternion_matrix(q)
        T = np.eye(4)
        T[:3,:3] = R[:3,:3]
        T[0,3] = t.translation.x
        T[1,3] = t.translation.y
        T[2,3] = t.translation.z
        return T

def main():
    rclpy.init()
    rclpy.spin(LidarClusterNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()


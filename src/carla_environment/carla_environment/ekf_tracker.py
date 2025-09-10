#!/usr/bin/env python3

import time, math
import numpy as np
import rclpy

from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Int32MultiArray, Float32, Header

class Track:
    def __init__(self, x0, P0, tid):
        self.x = x0          # [x,y,vx,vy]^T
        self.P = P0
        self.id = tid
        self.hits = 1
        self.misses = 0
        self.state = 'INIT'  # -> 'CONFIRMED' after confirm_hits
        

class EKFTracker(Node):
    def __init__(self):
        super().__init__('ekf_tracker')

        # Parameters
        self.algo = self.declare_parameter('algo_name', 'ekf').get_parameter_value().string_value
        self.fixed_frame = self.declare_parameter('fixed_frame', 'base_link').get_parameter_value().string_value
        self.confirm_hits = int(self.declare_parameter('confirm_hits', 3).get_parameter_value().integer_value or 3)
        self.max_misses = int(self.declare_parameter('max_misses', 8).get_parameter_value().integer_value or 8)

        # Noise
        self.Q = np.diag([0.5, 0.5, 1.0, 1.0]).astype(float)      # process
        self.R = np.diag([0.4, 0.4]).astype(float)                # lidar pos meas

        # Sub/Pub
        self.sub = self.create_subscription(PoseArray, '/detections_3d', self.cb_meas, 10)
        self.pub_tracks = self.create_publisher(PoseArray, f'/tracks/{self.algo}/poses', 5)
        self.pub_ids = self.create_publisher(Int32MultiArray, f'/tracks/{self.algo}/ids', 5)
        self.pub_ms = self.create_publisher(Float32, f'/metrics/{self.algo}/proc_ms', 5)
        self.pub_lead = self.create_publisher(PoseStamped, f'/tracks/{self.algo}/lead', 5)
        self.pub_markers = self.create_publisher(MarkerArray, f'/tracks/{self.algo}/markers', 5)


        # State
        self.tracks = []
        self.next_id = 1
        self.last_t = None

    # ---------- EKF primitives ----------
    @staticmethod
    def F(dt):
        return np.array([[1,0,dt,0],
                         [0,1,0,dt],
                         [0,0,1, 0],
                         [0,0,0, 1]], dtype=float)
    def predict_all(self, dt):
        F = self.F(dt)
        for tr in self.tracks:
            tr.x = F @ tr.x
            tr.P = F @ tr.P @ F.T + self.Q

    def update(self, tr: Track, z):
        H = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
        y = z - (H @ tr.x)
        S = H @ tr.P @ H.T + self.R
        K = tr.P @ H.T @ np.linalg.inv(S)
        tr.x = tr.x + K @ y
        tr.P = (np.eye(4) - K @ H) @ tr.P

    # ---------- Association (greedy NN with Mahalanobis gating) ----------
    @staticmethod
    def maha(z, x, P, R):
        H = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
        y = z - (H @ x)
        S = H @ P @ H.T + R
        return float(y.T @ np.linalg.inv(S) @ y)

    def associate(self, Z):
        pairs = []
        unmatched_i = set(range(len(Z)))
        unmatched_j = set(range(len(self.tracks)))
        costs = []
        for j,tr in enumerate(self.tracks):
            for i,z in enumerate(Z):
                d2 = self.maha(z, tr.x, tr.P, self.R)
                costs.append((d2,i,j))
        for d2,i,j in sorted(costs):
            if d2 > 9.21:  # ~95% gate for 2D
                continue
            if i in unmatched_i and j in unmatched_j:
                pairs.append((i,j))
                unmatched_i.remove(i)
                unmatched_j.remove(j)
        return pairs, list(unmatched_i), list(unmatched_j)

    # ---------- Main callback ----------
    def cb_meas(self, msg: PoseArray):
        t0 = time.perf_counter()

        # dt
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_t is None:
            self.last_t = now
        dt = max(1e-3, now - self.last_t)
        self.last_t = now

        # Predict
        self.predict_all(dt)

        # Build measurement list
        Z = [np.array([[p.position.x],[p.position.y]], dtype=float) for p in msg.poses]

        # Associate
        pairs, unmatched_i, unmatched_j = self.associate(Z)

        # Update matched
        for i,j in pairs:
            tr = self.tracks[j]
            self.update(tr, Z[i])
            tr.hits += 1
            tr.misses = 0
            if tr.state == 'INIT' and tr.hits >= self.confirm_hits:
                tr.state = 'CONFIRMED'

        # Missed tracks
        for j in unmatched_j:
            tr = self.tracks[j]
            tr.misses += 1

        # New tracks from unmatched measurements
        for i in unmatched_i:
            x0 = np.array([[float(Z[i][0])],[float(Z[i][1])],[0.0],[0.0]], dtype=float)
            P0 = np.diag([10,10,10,10]).astype(float)
            self.tracks.append(Track(x0, P0, self.next_id))
            self.next_id += 1

        # Prune
        kept = []
        for tr in self.tracks:
            if tr.state == 'INIT' and tr.misses > 2:
                continue
            if tr.misses > self.max_misses:
                continue
            kept.append(tr)
        self.tracks = kept

        # Publish tracks (CONFIRMED only)
        out = PoseArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.fixed_frame
        ids = []
        for tr in self.tracks:
            if tr.state != 'CONFIRMED':
                continue
            p = Pose()
            p.position.x = float(tr.x[0]); p.position.y = float(tr.x[1]); p.position.z = 0.0
            p.orientation.w = 1.0
            out.poses.append(p)
            ids.append(tr.id)

        self.pub_tracks.publish(out)
        self.pub_ids.publish(Int32MultiArray(data=ids))

        # Lead (front-most confirmed)
        if out.poses:
            lead_idx = int(np.argmax([p.position.x for p in out.poses]))
            lead = PoseStamped()
            lead.header = out.header
            lead.pose = out.poses[lead_idx]
            self.pub_lead.publish(lead)

        # Runtime metric
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.pub_ms.publish(Float32(data=float(dt_ms)))
        
        
        markers = MarkerArray()
        t = self.get_clock().now().to_msg()
        ns = f'{self.algo}_track'
        # text labels
        for tr, pose, track_id in zip(self.tracks, out.poses, ids):
            if tr.state != 'CONFIRMED':
                continue
            m = Marker()
            m.header.stamp = t
            m.header.frame_id = self.fixed_frame
            m.ns = ns
            m.id = int(track_id)             # stable per track
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = pose.position.x
            m.pose.position.y = pose.position.y
            m.pose.position.z = 1.2          # a little above ground
            m.scale.z = 0.6                  # text height (meters)
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
            m.text = f'ID {track_id}'
            m.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()  # refresh at ~5 Hz
            markers.markers.append(m)

        # optional: small spheres at track positions
        for tr, pose, track_id in zip(self.tracks, out.poses, ids):
            if tr.state != 'CONFIRMED':
                continue
            s = Marker()
            s.header.stamp = t
            s.header.frame_id = self.fixed_frame
            s.ns = ns + '_pt'
            s.id = 100000 + int(track_id)
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.pose = pose
            s.scale.x = s.scale.y = s.scale.z = 0.4
            s.color.r = 0.1; s.color.g = 0.9; s.color.b = 0.1; s.color.a = 0.9
            s.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
            markers.markers.append(s)

        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    rclpy.spin(EKFTracker())
    rclpy.shutdown()

if __name__ == '__main__':
    main()


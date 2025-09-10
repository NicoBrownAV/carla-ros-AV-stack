#!/usr/bin/env python3

import os, csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32MultiArray, Float32

from scipy.optimize import linear_sum_assignment

class OnlineEvaluator(Node):
    def __init__(self):
        super().__init__('online_evaluator')

        # Comma-separated list param, default: "ekf"
        algos_param = self.declare_parameter('algos', 'ekf').get_parameter_value().string_value
        self.algos = [s.strip() for s in algos_param.split(',') if s.strip()]
        if not self.algos:
            self.algos = ['ekf']

        # GT
        self.gt_pts = None
        self.create_subscription(PoseArray, '/gt/poses', self.cb_gt, 10)

        # Algo buffers
        self.track_pts = {a: None for a in self.algos}
        self.proc_ms   = {a: 0.0   for a in self.algos}

        for a in self.algos:
            self.create_subscription(PoseArray, f'/tracks/{a}/poses', lambda m, aa=a: self.cb_tracks(m, aa), 10)
            self.create_subscription(Float32, f'/metrics/{a}/proc_ms', lambda m, aa=a: self.cb_ms(m, aa), 10)

        # CSV
        log_dir = os.path.expanduser('~/carla_bench_logs'); os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, 'eval_log.csv')
        self.csv = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.csv)
        self.w.writerow(['stamp','algo','matches','ade','rmse','proc_ms'])
        self.get_logger().info(f"Logging to {self.csv_path}")

        self.timer = self.create_timer(0.1, self.tick)

    def cb_gt(self, msg: PoseArray):
        self.gt_pts = np.array([[p.position.x, p.position.y] for p in msg.poses], dtype=float)

    def cb_tracks(self, msg: PoseArray, algo: str):
        self.track_pts[algo] = np.array([[p.position.x, p.position.y] for p in msg.poses], dtype=float)

    def cb_ms(self, m: Float32, algo: str):
        self.proc_ms[algo] = float(m.data)

    def tick(self):
        if self.gt_pts is None:
            return
        stamp = float(self.get_clock().now().nanoseconds) * 1e-9
        for a in self.algos:
            tr = self.track_pts.get(a)
            if tr is None:
                continue
            C = self.cost(self.gt_pts, tr)
            if C.size == 0:
                self.w.writerow([stamp, a, 0, '', '', self.proc_ms[a]]); self.csv.flush(); continue
            rows, cols = linear_sum_assignment(C)
            errs = np.linalg.norm(self.gt_pts[rows] - tr[cols], axis=1)
            ade  = float(np.mean(errs)) if errs.size else 0.0
            rmse = float(np.sqrt(np.mean(errs**2))) if errs.size else 0.0
            self.w.writerow([stamp, a, int(errs.size), ade, rmse, self.proc_ms[a]])
            self.csv.flush()

    @staticmethod
    def cost(A,B):
        if A.size == 0 or B.size == 0:
            return np.zeros((0,0), dtype=float)
        AA = np.sum(A*A, axis=1, keepdims=True)
        BB = np.sum(B*B, axis=1, keepdims=True).T
        return AA + BB - 2.0 * (A @ B.T)

def main():
    rclpy.init(); rclpy.spin(OnlineEvaluator()); rclpy.shutdown()
if __name__ == '__main__': main()


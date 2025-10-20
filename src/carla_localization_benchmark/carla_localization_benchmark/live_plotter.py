#!/usr/bin/env python3
import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use("TkAgg")  # good default for interactive; change if needed
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class LivePlotter(Node):
    """
    Subscribes to:
      /carla/ego/ground_truth/odom  (GT, map->base_link)
      /sim/gps/odom                 (GPS, map->base_link)
      /fusion/odom                  (EKF, map->base_link)

    Shows two live plots:
      1) Trajectories in XY (equal aspect)
      2) Error vs time: ||GPS-GT|| and ||EKF-GT||
    """

    def __init__(self):
        super().__init__("live_plotter")

        # ---- parameters ----
        self.declare_parameter("max_points", 5000)          # history cap
        self.declare_parameter("update_hz", 10.0)           # plot refresh rate
        self.max_points = int(self.get_parameter("max_points").value)
        self.update_dt = 1.0 / float(self.get_parameter("update_hz").value)

        # ---- data buffers ----
        self.gt_xy = deque(maxlen=self.max_points)       # (x,y)
        self.gps_xy = deque(maxlen=self.max_points)
        self.ekf_xy = deque(maxlen=self.max_points)

        self.t_s   = deque(maxlen=self.max_points)       # seconds since first GT
        self.err_gps = deque(maxlen=self.max_points)     # |GPS-GT|
        self.err_ekf = deque(maxlen=self.max_points)     # |EKF-GT|

        self._t0_ns = None  # first GT stamp (for time axis)
        self._last_gt = None
        self._last_gps = None
        self._last_ekf = None

        # ---- subs ----
        self.create_subscription(Odometry, "/carla/ego/ground_truth/odom", self.on_gt, 20)
        self.create_subscription(Odometry, "/sim/gps/odom", self.on_gps, 20)
        self.create_subscription(Odometry, "/fusion/odom", self.on_ekf, 30)

        # ---- plotting setup (matplotlib objects) ----
        self.fig, (self.ax_xy, self.ax_err) = plt.subplots(1, 2, figsize=(12, 6))
        self.fig.canvas.manager.set_window_title("Live Localization Plots")

        # XY plot
        (self.l_gt,)  = self.ax_xy.plot([], [], linewidth=2, label="GT")
        (self.l_gps,) = self.ax_xy.plot([], [], linestyle="--", label="GPS")
        (self.l_ekf,) = self.ax_xy.plot([], [], linestyle="-",  label="EKF")
        self.ax_xy.set_xlabel("x [m]")
        self.ax_xy.set_ylabel("y [m]")
        self.ax_xy.set_title("Trajectories (map frame)")
        self.ax_xy.grid(True)
        self.ax_xy.legend(loc="best")
        self.ax_xy.set_aspect("equal", adjustable="datalim")

        # Error plot
        (self.l_egps,) = self.ax_err.plot([], [], label="‖GPS−GT‖")
        (self.l_eekf,) = self.ax_err.plot([], [], label="‖EKF−GT‖")
        self.ax_err.set_xlabel("time [s]")
        self.ax_err.set_ylabel("position error [m]")
        self.ax_err.set_title("Position Error vs Time")
        self.ax_err.grid(True)
        self.ax_err.legend(loc="best")

        # Lock for thread-safe buffer access (ROS callback thread <-> matplotlib anim thread)
        self._lock = threading.Lock()

        # Matplotlib animation timer (runs in main thread)
        self.ani = FuncAnimation(self.fig, self._on_animate, interval=int(self.update_dt * 1000))

    # --------- ROS callbacks ----------
    def on_gt(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        with self._lock:
            if self._t0_ns is None:
                self._t0_ns = stamp_ns
            t = (stamp_ns - self._t0_ns) * 1e-9
            self.gt_xy.append((x, y))
            self.t_s.append(t)
            self._last_gt = (x, y)

            # If we already have GPS/EKF for this moment, we can compute error immediately
            if self._last_gps is not None:
                gx, gy = self._last_gps
                self.err_gps.append(math.hypot(gx - x, gy - y))
            else:
                self.err_gps.append(float("nan"))

            if self._last_ekf is not None:
                ex, ey = self._last_ekf
                self.err_ekf.append(math.hypot(ex - x, ey - y))
            else:
                self.err_ekf.append(float("nan"))

    def on_gps(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        with self._lock:
            self.gps_xy.append((x, y))
            self._last_gps = (x, y)

    def on_ekf(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        with self._lock:
            self.ekf_xy.append((x, y))
            self._last_ekf = (x, y)

    # --------- Matplotlib animation callback ----------
    def _on_animate(self, _frame_idx):
        with self._lock:
            # XY data
            if self.gt_xy:
                gx, gy = zip(*self.gt_xy)
            else:
                gx, gy = [], []
            if self.gps_xy:
                gpx, gpy = zip(*self.gps_xy)
            else:
                gpx, gpy = [], []
            if self.ekf_xy:
                ex, ey = zip(*self.ekf_xy)
            else:
                ex, ey = [], []

            self.l_gt.set_data(gx, gy)
            self.l_gps.set_data(gpx, gpy)
            self.l_ekf.set_data(ex, ey)

            # Autoscale XY gently
            for ax in (self.ax_xy,):
                ax.relim()
                ax.autoscale_view(scalex=True, scaley=True)

            # Error data
            t = list(self.t_s)
            eg = list(self.err_gps)
            ee = list(self.err_ekf)
            self.l_egps.set_data(t, eg)
            self.l_eekf.set_data(t, ee)

            # Keep x-axis growing rightward; autoscale y
            if t:
                self.ax_err.set_xlim(left=max(0.0, t[0]), right=t[-1] if t[-1] > 5.0 else 5.0)
            self.ax_err.relim()
            self.ax_err.autoscale_view(scaley=True)

        return (self.l_gt, self.l_gps, self.l_ekf, self.l_egps, self.l_eekf)

    # --------- runner helpers ----------
    def show(self):
        plt.tight_layout()
        plt.show()


def main():
    # Run ROS in a background thread; keep matplotlib in the main thread
    rclpy.init()
    node = LivePlotter()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.show()  # blocks until the plot window is closed
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()


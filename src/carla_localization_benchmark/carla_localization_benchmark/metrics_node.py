import math, csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

def rmse(accum):
    if len(accum)==0: return float('nan')
    s = sum(v*v for v in accum)
    return math.sqrt(s/len(accum))

class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_node')
        self.declare_parameters('', [
            ('gt_odom_topic', '/carla/ego/ground_truth/odom'),
            ('fused_odom_topic', '/fusion/odom'),
            ('window_n', 300),             # ~6s at 50 Hz
            ('csv_log_path', ''),          # optional separate CSV for metrics
        ])
        p = self.get_parameter
        self.gt_topic = p('gt_odom_topic').value
        self.fused_topic = p('fused_odom_topic').value
        self.winN = int(p('window_n').value)
        self.csv_path = p('csv_log_path').value

        self.sub_gt = self.create_subscription(Odometry, self.gt_topic, self.on_gt, 10)
        self.sub_fused = self.create_subscription(Odometry, self.fused_topic, self.on_fused, 10)

        self.gt = None
        self.err_x2 = []
        self.err_y2 = []

        self.csv = open(self.csv_path, 'w') if self.csv_path else None
        if self.csv:
            self.csv_writer = csv.writer(self.csv)
            self.csv_writer.writerow(['stamp_ns', 'rmse_xy_m'])

    def on_gt(self, msg):
        self.gt = msg

    def on_fused(self, msg):
        if self.gt is None: return
        ex = msg.pose.pose.position.x - self.gt.pose.pose.position.x
        ey = msg.pose.pose.position.y - self.gt.pose.pose.position.y
        self.err_x2.append(ex*ex)
        self.err_y2.append(ey*ey)
        if len(self.err_x2) > self.winN:
            self.err_x2.pop(0); self.err_y2.pop(0)
        rmse_xy = math.sqrt((sum(self.err_x2)+sum(self.err_y2))/max(1,len(self.err_x2)))
        self.get_logger().info(f"RMSE(xy) over last {len(self.err_x2)} samples: {rmse_xy:.3f} m", throttle_duration_sec=1.0)
        if self.csv:
            self.csv_writer.writerow([msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec, rmse_xy])
            self.csv.flush()

def main():
    rclpy.init()
    rclpy.spin(MetricsNode())
    rclpy.shutdown()


#!/usr/bin/env python3

import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def wrap_angle(a):
    return (a + math.pi) % (2*math.pi) - math.pi

class BiasRandomWalk:
    def __init__(self, std, dt):
        self.std = std
        self.dt = dt
        self.bias = 0.0
    def step(self):
        self.bias += np.random.normal(0.0, self.std * math.sqrt(self.dt))
        return self.bias

def append_path(path_msg: Path, stamp, frame_id, x, y, yaw=0.0):
    ps = PoseStamped()
    ps.header.stamp = stamp
    ps.header.frame_id = frame_id
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.z = math.sin(yaw/2.0)
    ps.pose.orientation.w = math.cos(yaw/2.0)
    path_msg.header.stamp = stamp
    path_msg.header.frame_id = frame_id
    path_msg.poses.append(ps)
    return path_msg


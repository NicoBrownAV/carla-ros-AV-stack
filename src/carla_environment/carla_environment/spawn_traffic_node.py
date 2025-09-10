#!/usr/bin/env python3

import rclpy
import carla, random

from rclpy.node import Node

class SpawnTraffic(Node):
    def __init__(self):
        super().__init__('spawn_traffic')

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        # Params
        self.n_cars = self.declare_parameter('n_cars', 60).get_parameter_value().integer_value
        self.seed   = self.declare_parameter('seed', 7).get_parameter_value().integer_value
        self.tm_port = self.declare_parameter('tm_port', 8000).get_parameter_value().integer_value

        # Get Traffic Manager (prefer client.get_trafficmanager)
        try:
            self.tm = self.client.get_trafficmanager(self.tm_port)
        except Exception:
            # Fallback for odd builds; some forks attach on world
            if hasattr(self.world, 'get_trafficmanager'):
                self.tm = self.world.get_trafficmanager(self.tm_port)
            else:
                raise RuntimeError("Traffic Manager API not found. Use client.get_trafficmanager(port).")

        # Optional sync (recommended for benchmarking)
        try:
            self.tm.set_synchronous_mode(True)
        except Exception:
            pass  # if not supported, ignore

        self.timer = self.create_timer(0.1, self._spawn_once)
        self._done = False

    def _spawn_once(self):
        if self._done:
            return

        random.seed(int(self.seed))
        bp_lib = self.world.get_blueprint_library()
        vehicle_bps = bp_lib.filter('vehicle.*')

        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        count = 0
        for sp in spawn_points:
            bp = random.choice(vehicle_bps)
            # make them visible to TM
            try:
                bp.set_attribute('role_name', 'autopilot')
            except Exception:
                pass
            actor = self.world.try_spawn_actor(bp, sp)
            if actor:
                # IMPORTANT: pass the TM port you obtained
                actor.set_autopilot(True, self.tm.get_port())
                count += 1
                if count >= self.n_cars:
                    break

        self.get_logger().info(f"Spawned {count} vehicles with TM port {self.tm.get_port()}.")
        self._done = True

def main():
    rclpy.init()
    rclpy.spin(SpawnTraffic())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

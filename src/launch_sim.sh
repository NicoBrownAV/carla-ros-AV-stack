#!/bin/bash
set -euo pipefail

# === Step 1: Kill any existing CARLA processes ===
echo "[INFO] Killing existing CARLA processes..."
pkill -f CarlaUE4.sh || true
pkill -f CarlaUE4 || true
sleep 2  # Give time to clean up

# === Step 2: Start CARLA in the background ===
echo "[INFO] Launching CARLA simulator..."
~/Carla-Sim/CarlaUE4.sh &
CARLA_PID=$!

# === Step 3: Wait for CARLA to open port 2000 ===
echo "[INFO] Waiting for CARLA server to start on localhost:2000..."
while ! nc -z localhost 2000; do
  sleep 0.5
done
echo "[INFO] CARLA is up!"

# === Step 4: Launch ROS 2 ego vehicle + sensors ===
echo "[INFO] Launching ego vehicle and sensors..."
ros2 launch carla_environment spawn_vehicle.launch.py &
EGO_PID=$!
sleep 3

# === Step 5: Launch traffic, GT, tracker, evaluator ===
echo "[INFO] Launching traffic, ground truth, tracker, evaluator..."
ros2 launch carla_environment traffic_bench.launch.py &
BENCH_PID=$!
sleep 3

# === Step 6: Launch RViz in a new tab with your config ===
echo "[INFO] Launching RViz2 with sensor outputs..."
gnome-terminal --tab -- bash -c "
  source /opt/ros/humble/setup.bash
  source ~/carla_ws/install/setup.bash
  rviz2 -d ~/carla_ws/src/carla_environment/launch/sensor_output.rviz
  exec bash
"

# === Step 7: Trap cleanup ===
cleanup() {
  echo "[INFO] Cleaning up processes..."
  kill -9 $BENCH_PID $EGO_PID $CARLA_PID 2>/dev/null || true
}
trap cleanup EXIT

# Keep script alive until user interrupts
wait $BENCH_PID


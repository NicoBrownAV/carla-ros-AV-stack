#!/bin/bash

# === Step 1: Kill any existing CARLA processes ===
echo "[INFO] Killing existing CARLA processes..."

pkill -f CarlaUE4.sh
pkill -f CarlaUE4
sleep 2  # Give time to clean up

# === Step 2: Start CARLA in the background ===
echo "[INFO] Launching CARLA simulator..."

~/Carla-Sim/CarlaUE4.sh &

# === Step 3: Wait for CARLA to open port 2000 ===
echo "[INFO] Waiting for CARLA server to start on localhost:2000..."

while ! nc -z localhost 2000; do
  sleep 0.5
done

echo "[INFO] CARLA is up!"

# === Step 4: Launch ROS 2 vehicle spawn script ===
echo "[INFO] Launching vehicle and sensors..."

ros2 launch carla_environment spawn_vehicle.launch.py


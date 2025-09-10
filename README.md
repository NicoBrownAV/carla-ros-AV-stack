# carla-ROS2- Workspace-AV-stack
This workspace integrates the CARLA Simulator with ROS 2 (Humble) to run autonomous vehicle perception & tracking experiments.

## Features

- Ego vehicle with LiDAR, camera, radar, GPS sensors
- Traffic generation (vehicles + pedestrians)
- Camera object detection (YOLOv8)
- LiDAR clustering (DBSCAN) → 3D detections
- EKF multi-target tracker → publishes tracks and lead vehicle
- Ground truth publisher for evaluation
- Online evaluator → ADE/RMSE + runtime metrics
- Visualization in RViz (camera image, point cloud, detections, tracks)


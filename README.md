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

## Installation 
    sudo apt update
    sudo apt install ros-humble-vision-msgs \
                 python3-tf-transformations \
                 ros-humble-rviz2
    pip install ultralytics scikit-learn


## Build 
    cd ~/carla_ws
    colcon build --symlink-install
    . install/setup.bash

## Running the System
1. Launch Simulation (ego + traffic + perception + tracker)

       ./launch_sim.sh
   
- Normal mode: opens CARLA window and RViz
- Headless mode: no CARLA window, no RViz

      ./launch_sim.sh --headless

2. Record a Dataset

       ros2 bag record -o carla_dataset \
        /vehicle_camera/image_raw \
        /vehicle_camera/image_annotated \
        /vehicle_lidar/points \
        /detections_2d \
        /detections_3d \
        /tracks/ekf/poses \
        /tracks/ekf/lead \
        /gt/poses \
        /metrics/ekf/proc_ms \
        /tf /tf_static
## ROS 2 Nodes
- spawn_vehicle_w_sensors.py
    #### Spawns ego vehicle + LiDAR, camera, radar, GPS. Publishes ROS topics:
    - /vehicle_camera/image_raw, /vehicle_camera/camera_info
    - /vehicle_lidar/points
    - /vehicle_radar/detections
    - /tf transforms

    #### camera_yolo_node.py
    Runs YOLOv8 on camera images. Publishes:
    - /detections_2d (vision_msgs/Detection2DArray)
    - /vehicle_camera/image_annotated

    #### lidar_cluster_node.py
    Clusters LiDAR point cloud (DBSCAN). Publishes:
    - /detections_3d (PoseArray)

    - /detections_3d/markers (MarkerArray)

    #### ekf_tracker.py
    EKF multi-object tracker. Publishes:
    - /tracks/ekf/poses (PoseArray)
    - /tracks/ekf/lead (PoseStamped)
    - /tracks/ekf/markers (MarkerArray)
    - /metrics/ekf/proc_ms (Float32)

    #### carla_gt_publisher.py
    Publishes ground truth positions of all CARLA vehicles:
    - /gt/poses (PoseArray)
    - /gt/ids (Int32MultiArray)

    #### online_evaluator.py
    Associates tracks ↔ GT and logs metrics to ~/carla_bench_logs/eval_log.csv.

    #### Traffic generation
    - generate_traffic.py (from CARLA examples)  or spawn_traffic_node.py (ROS 2 wrapper)

## RViz Visualization
Displays to add:
- Image --> /vehicle_camera/image_annotated
- Pointcloud2 --> /vehicle_lidar/points
- Marker --> /detections_3d/markers
- PoseArray --> /tracks/ekf/poses
- Marker --> /tracks/ekf/markers
- Pose --> /tracks/ekf/lead
- TF --> see transforms
- Grid --> ground plane
Set Fixed Frame = base_link

## Notes
- LiDAR and radar Y-axis are flipped from CARLA → fixed with points[:,1] = -points[:,1].
- Use --headless mode for benchmarks (no rendering overhead).
- Evaluator writes ADE, RMSE, runtime metrics to CSV for comparison of different trackers.

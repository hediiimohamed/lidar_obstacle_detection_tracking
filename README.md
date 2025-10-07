# LiDAR Obstacle Detection & Tracking Node

## Overview

This ROS2 node performs 3D LiDAR-based obstacle detection and tracking using point cloud processing algorithms. It processes incoming LiDAR data to:

- Filter and segment the ground plane
- Cluster obstacle points
- Generate bounding boxes (axis-aligned or PCA-oriented)
- Track obstacles across frames

## Features

- Region of Interest (ROI) filtering
- Ground plane segmentation using RANSAC
- Euclidean clustering for obstacle detection
- Two bounding box generation methods:
  - Axis-Aligned Bounding Box (AABB)
  - PCA-oriented Bounding Box (more accurate orientation)
- Obstacle tracking between frames using the Hungarian algorithm
- Multiple visualization outputs:
  - Detected objects (Autoware format)
  - 3D bounding boxes
  - Segmented ground and obstacle point clouds

## Parameters

### Topics

| Parameter               | Description                            | Default              |
|------------------------|----------------------------------------|----------------------|
| `lidar_points_topic`   | Input LiDAR point cloud topic          | `/ouster/points`     |
| `cloud_ground_topic`   | Output ground point cloud topic        | `/cloud/ground`      |
| `cloud_clusters_topic` | Output obstacle clusters topic         | `/cloud/clusters`    |
| `vision_bboxes_topic`  | Output 3D bounding boxes topic         | `/bboxes/vision`     |
| `autoware_objects_topic` | Output Autoware detected objects topic | `/detected_objects` |
| `bbox_target_frame`    | Target frame for bounding boxes        | `base_link`          |

### Processing Parameters

| Parameter               | Description                                      | Default    |
|------------------------|--------------------------------------------------|------------|
| `use_pca_box`          | Use PCA-oriented boxes (true) or AABB (false)   | `true`     |
| `use_tracking`         | Enable obstacle tracking between frames         | `true`     |
| `voxel_grid_size`      | Voxel grid filter leaf size (m)                 | `0.1`      |
| `roi_max_x/y/z`        | ROI max bounds (m)                              | `50.0`     |
| `roi_min_x/y/z`        | ROI min bounds (m)                              | `-50.0`    |
| `ground_threshold`     | Ground plane distance threshold (m)             | `0.1`      |
| `cluster_threshold`    | Euclidean clustering distance (m)               | `0.5`      |
| `cluster_max_size`     | Max points per cluster                          | `10000`    |
| `cluster_min_size`     | Min points per cluster                          | `100`      |
| `displacement_threshold` | Tracking displacement threshold               | `1.0`      |
| `iou_threshold`        | Tracking IOU threshold                          | `0.5`      |

## Usage

### Launch the Node

```bash
ros2 launch lidar_detection_tracking obstacle_detector.launch.py
```

#### Modify Parameters at Runtime
```bash
ros2 param set /obstacle_detector use_pca_box false

```




##### Input/Output

Input :

- sensor_msgs/msg/PointCloud2: Raw LiDAR point cloud data

Output :

- Sensor_msgs/msg/PointCloud2: Ground points

- Sensor_msgs/msg/PointCloud2: Obstacle clusters

- Autoware_auto_perception_msgs/msg/DetectedObjects: Autoware standard perception message format

- Custom_msgs/msg/BoundingBox3DArray: 3D bounding boxes of detected obstacles 

- Visualization_msgs/msg/MarkerArray: RViz visualization markers

###### Dependencies
- ROS2 (Foxy or newer)

- PCL (Point Cloud Library)

- Eigen

- TF2

###### Lidar Obstacle Detection & Tracking ROS2

This project is a ROS2 adaptation of the original ROS1 repository:
[lidar_obstacle_detector](https://github.com/SS47816/lidar_obstacle_detector) by SS47816.

Main modifications:

Ported from ROS1 to ROS2

###### Licence

This project is released under the MIT License. The original project’s license is preserved.

© SS47816 et Hedi Oueslati

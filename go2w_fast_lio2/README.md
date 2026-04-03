# go2w_fast_lio2

Wrapper launch package for running FAST-LIO2 with Hesai PandarXT-16 LiDAR on Unitree Go2W.

## Overview

This package handles the full pipeline:

1. **Hesai LiDAR driver** (`hesai_ros_driver`) - publishes raw point cloud to `/lidar_points`
2. **Hesai-to-Velodyne converter** (`hesai_to_velodyne_converter`) - converts Hesai point cloud format to Velodyne-compatible format (`/velodyne_points`), because FAST_LIO expects a `time` field (float32, offset in seconds) instead of Hesai's `timestamp` field (float64, absolute)
3. **FAST-LIO2** (`fastlio_mapping`) - LiDAR-inertial odometry, subscribes to `/velodyne_points` and `/go2w/imu`
4. **Go2W robot description** - URDF, joint states, IMU publisher
5. **RViz2** (optional) - visualization

## Prerequisites

Make sure `livox_ros_driver2` and `fast_lio` are already built. See the main [README](../README.md) for build order.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select go2w_fast_lio2
```

## Usage

```bash
source ~/ros2_ws/install/setup.bash

# With RViz
ros2 launch go2w_fast_lio2 fast_lio2.launch.py

# Without RViz
ros2 launch go2w_fast_lio2 fast_lio2.launch.py rviz:=false

# Custom RViz config
ros2 launch go2w_fast_lio2 fast_lio2.launch.py rviz_config:=/path/to/custom.rviz
```

## Config Files

| File | Description |
|------|-------------|
| `config/hesai_config.yaml` | Hesai PandarXT-16 driver config (IP, port, transform, topics) |
| `config/fast_lio2_hesai.yaml` | FAST-LIO2 parameters tuned for Hesai XT-16 with Go2W IMU |

## Key Parameters (fast_lio2_hesai.yaml)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `lid_topic` | `/velodyne_points` | Converted LiDAR topic |
| `imu_topic` | `/go2w/imu` | Go2W IMU topic |
| `lidar_type` | `2` | Velodyne-style (after conversion) |
| `scan_line` | `16` | PandarXT-16 has 16 channels |
| `timestamp_unit` | `0` | Seconds (converter outputs offset in seconds) |
| `point_filter_num` | `4` | Downsample: keep 1 every 4 points |

## Published Topics (from FAST-LIO2)

| Topic | Type | Description |
|-------|------|-------------|
| `/cloud_registered` | PointCloud2 | Registered point cloud in world frame |
| `/cloud_registered_body` | PointCloud2 | Registered point cloud in body frame |
| `/Laser_map` | PointCloud2 | Accumulated map |
| `/Odometry` | Odometry | LiDAR-inertial odometry |
| `/path` | Path | Trajectory |

## TF Tree

```
camera_init -> body -> base_link -> base_footprint
                                 -> hesai_lidar
                                 -> imu
                                 -> (leg joints)
```

- `camera_init -> body`: published by FAST-LIO2
- `body -> base_link`: static transform (identity) published by this launch file
- `base_link -> ...`: published by robot_state_publisher from URDF

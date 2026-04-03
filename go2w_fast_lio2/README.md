# go2w_fast_lio2

Wrapper launch package for running FAST-LIO2 with Hesai PandarXT-16 LiDAR on Unitree Go2W.

## Overview

This package handles the full pipeline:

1. **Hesai LiDAR driver** (`hesai_ros_driver`) - publishes point cloud directly to `/lidar_points` in Velodyne-compatible format (field `time` as float32 offset from frame start)
2. **FAST-LIO2** (`fastlio_mapping`) - LiDAR-inertial odometry, subscribes to `/lidar_points` and `/go2w/imu`
3. **Go2W robot description** - URDF, joint states, IMU publisher
4. **RViz2** (optional) - visualization

## Changelog

### 2026-04-03: Performance optimizations & converter removal

**Problem:** FAST-LIO2 was running at low frequency due to several bottlenecks:

1. **Converter node overhead** - A separate `hesai_to_velodyne_converter` node was converting Hesai point cloud format (`timestamp` float64 absolute) to Velodyne format (`time` float32 offset). This added an extra ROS serialization/deserialization hop and caused message drops (~7 Hz output from ~8.4 Hz input).

2. **OpenMP disabled on ARM** - FAST-LIO2's CMakeLists.txt only enabled OpenMP parallelization for x86/AMD64 CPUs. On the Jetson (aarch64), the nearest-search loop in `h_share_model` ran single-threaded.

3. **QoS mismatch** - After removing the converter, FAST-LIO2 subscribed with `SensorDataQoS` (BEST_EFFORT) but the Hesai driver publishes with RELIABLE QoS. In ROS 2, a BEST_EFFORT subscriber cannot receive from a RELIABLE publisher.

**Fixes applied:**

| File | Change | Why |
|------|--------|-----|
| `HesaiLidar_ROS_2.0_to_velodyne_format/.../source_driver_ros2.hpp` | Changed output field from `timestamp` (FLOAT64) to `time` (FLOAT32 offset) | Driver now outputs Velodyne-compatible format directly, eliminating the converter node |
| `FAST_LIO/CMakeLists.txt` | Added `aarch64/ARM64` to OpenMP platform check | Enables multi-threaded nearest search on Jetson (2 threads on 4-core) |
| `FAST_LIO/src/laserMapping.cpp` | Changed subscriber QoS from `SensorDataQoS()` to depth 20 (RELIABLE) | Matches Hesai driver's RELIABLE QoS so messages are actually received |
| `go2w_fast_lio2/config/fast_lio2_hesai.yaml` | `lid_topic`: `/velodyne_points` -> `/lidar_points` | Subscribe directly to driver output |
| `go2w_fast_lio2/config/fast_lio2_hesai.yaml` | Tuned: `point_filter_num: 6`, `max_iteration: 2`, `filter_size_surf/map: 0.8`, `det_range: 50`, `dense_publish_en: false` | Reduce per-scan computation for higher frequency |
| `go2w_fast_lio2/launch/fast_lio2.launch.py` | Removed `hesai_to_velodyne_converter` node | No longer needed |

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
| `lid_topic` | `/lidar_points` | Direct from Hesai driver |
| `imu_topic` | `/go2w/imu` | Go2W IMU topic |
| `lidar_type` | `2` | Velodyne-style (Hesai driver now outputs compatible format) |
| `scan_line` | `16` | PandarXT-16 has 16 channels |
| `timestamp_unit` | `0` | Seconds (driver outputs offset in seconds) |
| `point_filter_num` | `6` | Downsample: keep 1 every 6 points |
| `max_iteration` | `2` | EKF iterations (reduced from 3 for speed) |
| `filter_size_surf` | `0.8` | Voxel downsample size in meters |
| `det_range` | `50.0` | Max detection range in meters |

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

#!/usr/bin/env python3
"""Project the downward-facing UTLidar cloud into a ground-filtered scan.

The generic pointcloud_to_laserscan node treats every point in a height slice as
an obstacle. With the Go2W L1 pointing down, that includes the floor. This node
uses the base-aligned transformed cloud and filters by height above
base_footprint before projecting to LaserScan.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class UTLidarGroundScan(Node):
    def __init__(self):
        super().__init__("utlidar_ground_scan")

        self.input_topic = self.declare_parameter(
            "input_cloud_topic", "/utlidar/transformed_cloud").value
        self.output_topic = self.declare_parameter(
            "output_scan_topic", "/scan").value
        self.output_frame = self.declare_parameter(
            "output_frame", "base_footprint").value

        # transform_cloud publishes points in base orientation but keeps the
        # lidar origin. These offsets move points into base_footprint.
        self.sensor_x = float(self.declare_parameter("sensor_x", 0.28945).value)
        self.sensor_y = float(self.declare_parameter("sensor_y", 0.0).value)
        self.sensor_z = float(self.declare_parameter("sensor_z", 0.353175).value)

        self.min_obstacle_height = float(
            self.declare_parameter("min_obstacle_height", 0.10).value)
        self.max_obstacle_height = float(
            self.declare_parameter("max_obstacle_height", 0.90).value)
        self.range_min = float(self.declare_parameter("range_min", 0.35).value)
        self.range_max = float(self.declare_parameter("range_max", 12.0).value)
        self.angle_min = float(
            self.declare_parameter("angle_min", -math.pi).value)
        self.angle_max = float(
            self.declare_parameter("angle_max", math.pi).value)
        self.angle_increment = float(
            self.declare_parameter("angle_increment", 0.00872664626).value)
        self.scan_time = float(self.declare_parameter("scan_time", 0.066).value)
        self.use_cloud_stamp = bool(
            self.declare_parameter("use_cloud_stamp", False).value)
        self.min_points_per_bin = int(
            self.declare_parameter("min_points_per_bin", 1).value)
        self.reliable_qos = bool(
            self.declare_parameter("reliable_qos", True).value)

        self.bin_count = int(
            math.floor((self.angle_max - self.angle_min) / self.angle_increment)
        ) + 1

        reliability = (QoSReliabilityPolicy.RELIABLE if self.reliable_qos
                       else QoSReliabilityPolicy.BEST_EFFORT)
        qos = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos)

        self.get_logger().info(
            f"utlidar_ground_scan: {self.input_topic} -> {self.output_topic} "
            f"frame={self.output_frame} z_filter=[{self.min_obstacle_height:.2f}, "
            f"{self.max_obstacle_height:.2f}]m range=[{self.range_min:.2f}, "
            f"{self.range_max:.2f}]m bins={self.bin_count}")

    def cloud_callback(self, msg: PointCloud2):
        if msg.width * msg.height == 0 or len(msg.data) == 0:
            return

        points = pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        if points.size == 0:
            return

        x = points["x"].astype(np.float32, copy=False) + self.sensor_x
        y = points["y"].astype(np.float32, copy=False) + self.sensor_y
        z = points["z"].astype(np.float32, copy=False) + self.sensor_z

        ranges_xy = np.hypot(x, y)
        angles = np.arctan2(y, x)

        mask = (
            (z >= self.min_obstacle_height) &
            (z <= self.max_obstacle_height) &
            (ranges_xy >= self.range_min) &
            (ranges_xy <= self.range_max) &
            (angles >= self.angle_min) &
            (angles <= self.angle_max)
        )
        if not np.any(mask):
            self.publish_scan(msg, None, None)
            return

        bins = np.floor(
            (angles[mask] - self.angle_min) / self.angle_increment
        ).astype(np.int32)
        ranges = ranges_xy[mask]

        scan_ranges = np.full(self.bin_count, np.inf, dtype=np.float32)
        counts = np.zeros(self.bin_count, dtype=np.int32)

        order = np.argsort(ranges)
        for idx in order:
            bin_idx = bins[idx]
            if bin_idx < 0 or bin_idx >= self.bin_count:
                continue
            counts[bin_idx] += 1
            if ranges[idx] < scan_ranges[bin_idx]:
                scan_ranges[bin_idx] = ranges[idx]

        if self.min_points_per_bin > 1:
            scan_ranges[counts < self.min_points_per_bin] = np.inf

        self.publish_scan(msg, scan_ranges, counts)

    def publish_scan(self, cloud_msg: PointCloud2, ranges, counts):
        scan = LaserScan()
        scan.header.stamp = (
            cloud_msg.header.stamp if self.use_cloud_stamp
            else self.get_clock().now().to_msg()
        )
        scan.header.frame_id = self.output_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + self.angle_increment * (self.bin_count - 1)
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        if ranges is None:
            scan.ranges = [math.inf] * self.bin_count
        else:
            scan.ranges = ranges.tolist()

        scan.intensities = [] if counts is None else counts.astype(np.float32).tolist()
        self.publisher.publish(scan)


def main():
    rclpy.init()
    node = UTLidarGroundScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

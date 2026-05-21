#!/usr/bin/env python3
"""Rotate /utlidar/cloud to base-aligned orientation and restamp it.

Why: the L1 firmware clock drifts (seconds) versus the host wall clock that
stamps /go2w/imu, which deadlocks LIO sync_packages and lets cloud stamps
go backwards. This node restamps every cloud at arrival using the ROS clock
and enforces strict monotonicity, then rotates points by R_base_lidar so the
output cloud has base orientation (translation is intentionally NOT applied,
per request "orientasinya saja").
"""
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from transforms3d.euler import euler2mat


class TransformCloud(Node):
    def __init__(self):
        super().__init__('transform_cloud')

        # Default RPY/xyz mirror the base -> utlidar_lidar joint in
        # go2w_description/xacro/go2w.xacro (calibrated 2026-05-15+).
        roll = self.declare_parameter('roll', -2.924065656505679).value
        pitch = self.declare_parameter('pitch', -0.14998090081098114).value
        yaw = self.declare_parameter('yaw', -0.9763385649215685).value
        tx = self.declare_parameter('translation_x', 0.28945).value
        ty = self.declare_parameter('translation_y', 0.0).value
        tz = self.declare_parameter('translation_z', -0.046825).value

        self.input_topic = self.declare_parameter(
            'input_topic', '/utlidar/cloud').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/utlidar/transformed_cloud').value
        self.output_frame_id = self.declare_parameter(
            'output_frame_id', 'utlidar_lidar_aligned').value
        self.parent_frame_id = self.declare_parameter(
            'parent_frame_id', 'base').value
        self.reliable_qos = self.declare_parameter(
            'reliable_qos', True).value
        self.publish_static_tf = self.declare_parameter(
            'publish_static_tf', True).value

        # R_base_lidar: maps a vector in lidar coords into base orientation.
        self.R = euler2mat(roll, pitch, yaw, 'sxyz').astype(np.float32)
        self.get_logger().info(
            f"R_base_lidar (rpy={roll:.6f}, {pitch:.6f}, {yaw:.6f}):\n{self.R}")

        self._last_stamp_ns = None  # monotonic guard

        reliability = (QoSReliabilityPolicy.RELIABLE if self.reliable_qos
                       else QoSReliabilityPolicy.BEST_EFFORT)
        qos = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2, self.output_topic, qos)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos)

        if self.publish_static_tf:
            self._static_tf = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_frame_id
            t.child_frame_id = self.output_frame_id
            t.transform.translation.x = float(tx)
            t.transform.translation.y = float(ty)
            t.transform.translation.z = float(tz)
            t.transform.rotation.w = 1.0  # identity: orientation matches parent
            self._static_tf.sendTransform(t)
            self.get_logger().info(
                f"static_tf {self.parent_frame_id} -> {self.output_frame_id} "
                f"xyz=({tx}, {ty}, {tz}) rpy=identity")

        self.get_logger().info(
            f"transform_cloud: {self.input_topic} -> {self.output_topic} "
            f"(frame={self.output_frame_id}, "
            f"reliability={'RELIABLE' if self.reliable_qos else 'BEST_EFFORT'})")

    def _monotonic_stamp(self):
        """Return a monotonically increasing wall-clock stamp (sec, nanosec).

        - Aligns cloud timestamps to the same clock domain as /go2w/imu
          (rclcpp::Clock::now()), eliminating the L1 firmware skew.
        - Guarantees stamps never go backwards or repeat, so downstream
          buffers (FAST-LIO/Point-LIO sync_packages) never time-loop.
        """
        now_ns = self.get_clock().now().nanoseconds
        if self._last_stamp_ns is not None and now_ns <= self._last_stamp_ns:
            now_ns = self._last_stamp_ns + 1000  # +1 us
        self._last_stamp_ns = now_ns
        return now_ns

    def cloud_callback(self, msg: PointCloud2):
        new_stamp_ns = self._monotonic_stamp()

        if msg.width * msg.height == 0 or len(msg.data) == 0:
            return

        # Structured ndarray view honoring point_step (preserves padding).
        structured = pc2.read_points(msg, field_names=None, skip_nans=False)
        if structured.size == 0:
            return

        # Rotate xyz by R_base_lidar: p_out = R @ p_in  =>  rows @ R.T
        xyz = np.stack(
            [structured['x'], structured['y'], structured['z']], axis=-1
        ).astype(np.float32, copy=False)
        xyz_rot = xyz @ self.R.T

        out_struct = structured.copy()
        out_struct['x'] = xyz_rot[:, 0]
        out_struct['y'] = xyz_rot[:, 1]
        out_struct['z'] = xyz_rot[:, 2]

        out = PointCloud2()
        out.header.stamp.sec = new_stamp_ns // 1_000_000_000
        out.header.stamp.nanosec = new_stamp_ns % 1_000_000_000
        out.header.frame_id = self.output_frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = out_struct.tobytes()
        self.cloud_pub.publish(out)


def main():
    rclpy.init()
    node = TransformCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

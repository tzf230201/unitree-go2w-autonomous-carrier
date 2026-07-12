#!/usr/bin/env python3
"""Restamp /utlidar/cloud, rotate to base orientation, assign ring by elevation.

Why: LIO-SAM imageProjection needs (a) host-clock timestamps to sync with
/go2w/imu (L1 firmware clock is +18.5 s skewed from host), (b) a valid ring
index per point to build its (N_SCAN x Horizon_SCAN) range image, and (c)
points expressed in a body-like frame whose +Z is gravity-up. The L1
internal frame's +Z is roughly world-down, which breaks LIO-SAM's
imuPreintegration gravity model (manifests as velocity exploding past
30 m/s within ~1.5 s while static).

Pipeline: rotate XYZ by R_base_lidar (calibrated rpy in go2w xacro),
synthesize ring by binning elevation in the rotated/base frame, restamp
to arrival-scan_period (Velodyne convention), publish in frame=base.
LIO-SAM extrinsic IMU<->lidar then = identity.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from transforms3d.euler import euler2mat


class RingCloud(Node):
    def __init__(self):
        super().__init__('ring_cloud')

        self.input_topic = self.declare_parameter(
            'input_topic', '/utlidar/cloud').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/utlidar/cloud_ringed').value
        # Frame after rotation is base-aligned. Use a distinct name from
        # baselinkFrame so LIO-SAM's imuPreintegration tCur gets a frame_id
        # populated via lookupTransform(lidarFrame, baselinkFrame) — when
        # both names match, LIO-SAM skips that lookup and broadcasts a TF
        # with empty parent (TF_NO_FRAME_ID errors).
        self.output_frame_id = self.declare_parameter(
            'output_frame_id', 'lidar_base').value
        self.n_scan = int(self.declare_parameter('n_scan', 32).value)
        # After R_base_lidar rotation: measured elev range in base frame is
        # [-84, +15] deg (L1 mounted on head, points down-forward). Bracket
        # with [-90, +20] so binning is uniform without clipping pile-ups.
        self.elev_min_deg = float(
            self.declare_parameter('elev_min_deg', -90.0).value)
        self.elev_max_deg = float(
            self.declare_parameter('elev_max_deg', 20.0).value)
        # base -> utlidar_lidar joint (calibrated, go2w_description xacro).
        roll = self.declare_parameter('roll', -2.924065656505679).value
        pitch = self.declare_parameter('pitch', -0.14998090081098114).value
        yaw = self.declare_parameter('yaw', -0.9763385649215685).value
        # L1 measured ~15.4 Hz -> ~65 ms. Header stamp = arrival - scan_period
        # so it lands on scan START (Velodyne convention; matches per-point
        # `time` ∈ [0, scan_period]).
        self._scan_period_ns = int(
            self.declare_parameter('scan_period_sec', 0.065).value * 1e9)
        self.reliable_qos = self.declare_parameter(
            'reliable_qos', True).value

        reliability = (QoSReliabilityPolicy.RELIABLE if self.reliable_qos
                       else QoSReliabilityPolicy.BEST_EFFORT)
        qos = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # R_base_lidar: p_base = R @ p_lidar
        self.R = euler2mat(roll, pitch, yaw, 'sxyz').astype(np.float32)
        self._last_stamp_ns = None
        self._elev_span = self.elev_max_deg - self.elev_min_deg

        self.cloud_pub = self.create_publisher(
            PointCloud2, self.output_topic, qos)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos)

        self.get_logger().info(
            f"ring_cloud: {self.input_topic} -> {self.output_topic} "
            f"(frame={self.output_frame_id}, N_SCAN={self.n_scan}, "
            f"elev=[{self.elev_min_deg:.2f}, {self.elev_max_deg:.2f}] deg, "
            f"R_base_lidar rpy=({roll:.4f}, {pitch:.4f}, {yaw:.4f}))")

    def _monotonic_stamp(self):
        arrival_ns = self.get_clock().now().nanoseconds
        new_stamp_ns = arrival_ns - self._scan_period_ns
        if self._last_stamp_ns is not None and new_stamp_ns <= self._last_stamp_ns:
            new_stamp_ns = self._last_stamp_ns + 1000
        self._last_stamp_ns = new_stamp_ns
        return new_stamp_ns

    def cloud_callback(self, msg: PointCloud2):
        new_stamp_ns = self._monotonic_stamp()

        if msg.width * msg.height == 0 or len(msg.data) == 0:
            return

        structured = pc2.read_points(msg, field_names=None, skip_nans=False)
        if structured.size == 0:
            return

        # Rotate xyz by R_base_lidar: rows @ R.T  =>  p_base = R @ p_lidar
        xyz = np.stack(
            [structured['x'], structured['y'], structured['z']], axis=-1
        ).astype(np.float32, copy=False)
        xyz_rot = xyz @ self.R.T
        xr = xyz_rot[:, 0]
        yr = xyz_rot[:, 1]
        zr = xyz_rot[:, 2]

        # ring index 0..N_SCAN-1 from elevation (in base frame, post-rotation)
        r_xy = np.sqrt(xr * xr + yr * yr)
        safe = r_xy > 1e-3
        elev_deg = np.zeros_like(zr)
        elev_deg[safe] = np.degrees(np.arctan2(zr[safe], r_xy[safe]))
        elev_deg[~safe] = self.elev_min_deg
        bin_idx = ((elev_deg - self.elev_min_deg) / self._elev_span
                   * self.n_scan).astype(np.int32)
        np.clip(bin_idx, 0, self.n_scan - 1, out=bin_idx)

        out_struct = structured.copy()
        out_struct['x'] = xr
        out_struct['y'] = yr
        out_struct['z'] = zr
        # LIO-SAM expects uint16 ring (utility.hpp velodyne struct)
        if out_struct['ring'].dtype != np.uint16:
            ring_field = bin_idx.astype(out_struct['ring'].dtype, copy=False)
        else:
            ring_field = bin_idx.astype(np.uint16, copy=False)
        out_struct['ring'] = ring_field

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
    node = RingCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

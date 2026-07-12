#!/usr/bin/env python3
"""Diagnostic node — prints concrete numbers for /go2w/imu and /utlidar/cloud
to verify rate, stationary bias, and cloud frame conventions.

Run while robot is STATIONARY for first 5-10 seconds. Then any movement is OK.
Prints a summary every 3 seconds.
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class DataDiag(Node):
    def __init__(self):
        super().__init__('data_diag')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Imu, '/go2w/imu', self.imu_cb, qos)
        self.create_subscription(PointCloud2, '/utlidar/cloud',
                                 self.raw_cloud_cb, qos)
        self.create_subscription(PointCloud2, '/utlidar/transformed_cloud',
                                 self.tf_cloud_cb, qos)

        self._imu_t = []
        self._imu_accel = []
        self._imu_gyro = []

        self._raw_cloud_t = []
        self._tf_cloud_t = []

        self._last_raw_sample = None
        self._last_tf_sample = None

        self._t0 = time.monotonic()
        self.create_timer(3.0, self.report)
        self.get_logger().info(
            "data_diag running. KEEP ROBOT STATIONARY for first reports.")

    def imu_cb(self, msg: Imu):
        self._imu_t.append(time.monotonic())
        self._imu_accel.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        self._imu_gyro.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])

    def raw_cloud_cb(self, msg: PointCloud2):
        self._raw_cloud_t.append(time.monotonic())
        self._last_raw_sample = self._sample_cloud(msg)

    def tf_cloud_cb(self, msg: PointCloud2):
        self._tf_cloud_t.append(time.monotonic())
        self._last_tf_sample = self._sample_cloud(msg)

    def _sample_cloud(self, msg: PointCloud2):
        try:
            pts = pc2.read_points(msg, field_names=None, skip_nans=False)
            if pts.size == 0:
                return None
            t_field = pts['time'] if 'time' in pts.dtype.names else None
            return {
                'n_points': pts.size,
                'first_xyz': (float(pts[0]['x']), float(pts[0]['y']),
                              float(pts[0]['z'])),
                'last_xyz': (float(pts[-1]['x']), float(pts[-1]['y']),
                             float(pts[-1]['z'])),
                'x_range': (float(pts['x'].min()), float(pts['x'].max())),
                'y_range': (float(pts['y'].min()), float(pts['y'].max())),
                'z_range': (float(pts['z'].min()), float(pts['z'].max())),
                't_first': float(t_field[0]) if t_field is not None else None,
                't_last': float(t_field[-1]) if t_field is not None else None,
                't_min': float(t_field.min()) if t_field is not None else None,
                't_max': float(t_field.max()) if t_field is not None else None,
                'header_stamp': (msg.header.stamp.sec +
                                 msg.header.stamp.nanosec * 1e-9),
                'arrival': time.time(),
            }
        except Exception as e:
            return {'error': str(e)}

    def _rate(self, ts, window_s=3.0):
        if len(ts) < 2:
            return 0.0
        now = time.monotonic()
        recent = [t for t in ts if now - t < window_s]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])

    def report(self):
        elapsed = time.monotonic() - self._t0
        print(f"\n=== data_diag @ t={elapsed:.1f}s ===")

        # IMU
        imu_rate = self._rate(self._imu_t)
        if self._imu_accel:
            a = np.array(self._imu_accel[-200:])  # last ~400ms at 500Hz
            g = np.array(self._imu_gyro[-200:])
            print(f"/go2w/imu  rate={imu_rate:6.1f} Hz  N_recent={len(a)}")
            print(f"  accel mean=[{a.mean(0)[0]:+.4f} {a.mean(0)[1]:+.4f} "
                  f"{a.mean(0)[2]:+.4f}] m/s^2   |mean|={np.linalg.norm(a.mean(0)):.4f} "
                  f"(expect 9.81 stationary)")
            print(f"  accel std =[{a.std(0)[0]:.4f} {a.std(0)[1]:.4f} "
                  f"{a.std(0)[2]:.4f}]")
            print(f"  gyro  mean=[{g.mean(0)[0]:+.5f} {g.mean(0)[1]:+.5f} "
                  f"{g.mean(0)[2]:+.5f}] rad/s   (expect 0 stationary)")
            print(f"  gyro  std =[{g.std(0)[0]:.5f} {g.std(0)[1]:.5f} "
                  f"{g.std(0)[2]:.5f}]")
        else:
            print(f"/go2w/imu  NO DATA")

        # Raw cloud
        raw_rate = self._rate(self._raw_cloud_t)
        print(f"\n/utlidar/cloud  rate={raw_rate:5.2f} Hz")
        if self._last_raw_sample:
            s = self._last_raw_sample
            if 'error' in s:
                print(f"  ERROR: {s['error']}")
            else:
                print(f"  N={s['n_points']}  header.stamp={s['header_stamp']:.6f} "
                      f"arrival={s['arrival']:.6f} skew={s['arrival']-s['header_stamp']:+.4f}s")
                print(f"  xyz ranges: x=[{s['x_range'][0]:+.2f},{s['x_range'][1]:+.2f}]  "
                      f"y=[{s['y_range'][0]:+.2f},{s['y_range'][1]:+.2f}]  "
                      f"z=[{s['z_range'][0]:+.2f},{s['z_range'][1]:+.2f}]")
                print(f"  first point xyz=({s['first_xyz'][0]:+.3f}, "
                      f"{s['first_xyz'][1]:+.3f}, {s['first_xyz'][2]:+.3f})")
                if s['t_first'] is not None:
                    print(f"  per-point `time` field: first={s['t_first']:.6f} "
                          f"last={s['t_last']:.6f} min={s['t_min']:.6f} max={s['t_max']:.6f}")

        # Transformed cloud
        tf_rate = self._rate(self._tf_cloud_t)
        print(f"\n/utlidar/transformed_cloud  rate={tf_rate:5.2f} Hz")
        if self._last_tf_sample:
            s = self._last_tf_sample
            if 'error' in s:
                print(f"  ERROR: {s['error']}")
            else:
                print(f"  N={s['n_points']}  header.stamp={s['header_stamp']:.6f} "
                      f"arrival={s['arrival']:.6f} skew={s['arrival']-s['header_stamp']:+.4f}s")
                print(f"  xyz ranges: x=[{s['x_range'][0]:+.2f},{s['x_range'][1]:+.2f}]  "
                      f"y=[{s['y_range'][0]:+.2f},{s['y_range'][1]:+.2f}]  "
                      f"z=[{s['z_range'][0]:+.2f},{s['z_range'][1]:+.2f}]")
                print(f"  first point xyz=({s['first_xyz'][0]:+.3f}, "
                      f"{s['first_xyz'][1]:+.3f}, {s['first_xyz'][2]:+.3f})")

        # trim old samples to keep memory bounded
        cutoff = time.monotonic() - 30.0
        self._imu_t = [t for t in self._imu_t if t > cutoff]
        self._imu_accel = self._imu_accel[-15000:]
        self._imu_gyro = self._imu_gyro[-15000:]
        self._raw_cloud_t = [t for t in self._raw_cloud_t if t > cutoff]
        self._tf_cloud_t = [t for t in self._tf_cloud_t if t > cutoff]


def main():
    rclpy.init()
    node = DataDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

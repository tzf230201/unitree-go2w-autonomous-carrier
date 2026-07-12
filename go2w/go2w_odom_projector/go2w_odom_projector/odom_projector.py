import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class OdomProjector(Node):
    def __init__(self) -> None:
        super().__init__("go2w_odom_projector")

        self.input_topic = self.declare_parameter("input_topic", "/Odometry").value
        self.output_topic = self.declare_parameter("output_topic", "/odom").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        self.zero_initial_pose = self.declare_parameter("zero_initial_pose", True).value
        self.use_input_stamp = self.declare_parameter("use_input_stamp", False).value

        self.initialized = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self.odom_callback, 20
        )

        self.get_logger().info(
            f"Projecting {self.input_topic} -> {self.output_topic} "
            f"(zero_initial_pose={self.zero_initial_pose})"
        )

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        if not self.initialized:
            self.initial_x = x
            self.initial_y = y
            self.initial_yaw = yaw
            self.initialized = True

        projected_x = x
        projected_y = y
        projected_yaw = yaw
        twist_x = msg.twist.twist.linear.x
        twist_y = msg.twist.twist.linear.y

        if self.zero_initial_pose:
            dx = x - self.initial_x
            dy = y - self.initial_y
            cos_yaw0 = math.cos(self.initial_yaw)
            sin_yaw0 = math.sin(self.initial_yaw)

            projected_x = cos_yaw0 * dx + sin_yaw0 * dy
            projected_y = -sin_yaw0 * dx + cos_yaw0 * dy
            projected_yaw = normalize_angle(yaw - self.initial_yaw)

            rotated_twist_x = cos_yaw0 * twist_x + sin_yaw0 * twist_y
            rotated_twist_y = -sin_yaw0 * twist_x + cos_yaw0 * twist_y
            twist_x = rotated_twist_x
            twist_y = rotated_twist_y

        odom_msg = Odometry()
        if self.use_input_stamp:
            odom_msg.header.stamp = msg.header.stamp
        else:
            odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = projected_x
        odom_msg.pose.pose.position.y = projected_y
        odom_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(projected_yaw)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Keep planar covariances from the source and strongly de-emphasize 3D terms.
        odom_msg.pose.covariance = list(msg.pose.covariance)
        odom_msg.pose.covariance[14] = 1e-6
        odom_msg.pose.covariance[21] = 1e6
        odom_msg.pose.covariance[28] = 1e6

        odom_msg.twist.twist.linear.x = twist_x
        odom_msg.twist.twist.linear.y = twist_y
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = msg.twist.twist.angular.z
        odom_msg.twist.covariance = list(msg.twist.covariance)

        self.publisher.publish(odom_msg)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom_msg.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = projected_x
            tf_msg.transform.translation.y = projected_y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

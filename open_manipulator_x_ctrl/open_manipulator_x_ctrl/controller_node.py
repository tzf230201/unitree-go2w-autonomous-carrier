"""
ROS 2 node for OpenManipulator-X position control.

Topics
------
  Pub:  /open_manipulator_x/joint_states    sensor_msgs/JointState   (50 Hz)
  Sub:  /open_manipulator_x/joint_command   sensor_msgs/JointState
        (name + position; gripper optional. Missing joints keep current goal.)
  Sub:  /open_manipulator_x/pose_command    geometry_msgs/PoseStamped
        (x,y,z + orientation; we extract pitch from the y-axis component of the
        quaternion. Position in metres, frame_id ignored.)
  Sub:  /open_manipulator_x/gripper_command std_msgs/Float64
        (target gripper joint position in radians)

Services
--------
  /open_manipulator_x/enable_torque  std_srvs/SetBool
  /open_manipulator_x/go_home        std_srvs/Trigger
  /open_manipulator_x/go_ready       std_srvs/Trigger

Safety
------
On startup the node opens the bus, reads present positions, pre-loads them as
goal positions, slows the Dynamixel profile, then enables torque. This keeps
the arm in place when torque turns on (no snap). Only after that does it start
accepting commands.
"""

from __future__ import annotations

import math
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger

from .dxl_driver import DxlBus, default_om_x_motors
from .kinematics import (
    forward_kinematics,
    inverse_kinematics,
    IKError,
    POSE_HOME_UP,
    POSE_READY_FORWARD,
)


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_NAME = "gripper"


def _quat_to_pitch(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract pitch (rotation about world +y axis) from a quaternion.

    For pure pitch quaternions q = (0, sin(p/2), 0, cos(p/2)) this is exact;
    for general quaternions it returns the standard intrinsic pitch.
    """
    # ZYX intrinsic Euler — pitch component
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


class OmxControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("open_manipulator_x_ctrl")

        # ---- parameters ----
        self.declare_parameter("device", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("include_gripper", True)
        self.declare_parameter("state_rate_hz", 50.0)
        self.declare_parameter("startup_slow_vel", 30)
        self.declare_parameter("startup_slow_acc", 10)
        self.declare_parameter("run_vel", 100)
        self.declare_parameter("run_acc", 30)
        self.declare_parameter("settle_seconds", 0.5)
        self.declare_parameter("elbow_up", True)

        device = self.get_parameter("device").value
        baud = int(self.get_parameter("baudrate").value)
        self._include_gripper = bool(self.get_parameter("include_gripper").value)
        self._run_vel = int(self.get_parameter("run_vel").value)
        self._run_acc = int(self.get_parameter("run_acc").value)
        self._elbow_up = bool(self.get_parameter("elbow_up").value)

        # ---- bus ----
        self.motors = default_om_x_motors(include_gripper=self._include_gripper)
        self.bus = DxlBus(device, baud, self.motors, logger=self.get_logger())
        self.bus.open()

        ping_status = self.bus.ping_all()
        self.get_logger().info(f"Ping: {ping_status}")
        missing = [i for i, ok in ping_status.items() if not ok]
        if missing:
            raise RuntimeError(f"Dynamixels not responding: ids={missing}")

        # ---- gentle torque-on ----
        present = self.bus.gentle_torque_on(
            slow_vel=int(self.get_parameter("startup_slow_vel").value),
            slow_acc=int(self.get_parameter("startup_slow_acc").value),
            run_vel=self._run_vel,
            run_acc=self._run_acc,
            settle_s=float(self.get_parameter("settle_seconds").value),
        )
        # mapping {name: dxl_id}
        self._id_by_name = {m.name: m.dxl_id for m in self.motors}
        self._goal_by_name: Dict[str, float] = {
            m.name: present[m.dxl_id] for m in self.motors
        }
        self._lock = threading.Lock()

        # ---- pub/sub ----
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_state = self.create_publisher(
            JointState, "/open_manipulator_x/joint_states", qos
        )
        self.create_subscription(
            JointState,
            "/open_manipulator_x/joint_command",
            self._on_joint_cmd,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            "/open_manipulator_x/pose_command",
            self._on_pose_cmd,
            qos,
        )
        self.create_subscription(
            Float64,
            "/open_manipulator_x/gripper_command",
            self._on_gripper_cmd,
            qos,
        )

        self.create_service(SetBool, "/open_manipulator_x/enable_torque", self._srv_torque)
        self.create_service(Trigger, "/open_manipulator_x/go_home", self._srv_home)
        self.create_service(Trigger, "/open_manipulator_x/go_ready", self._srv_ready)

        rate_hz = float(self.get_parameter("state_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._publish_state)

        self.get_logger().info("OM-X controller ready.")

    # ---------------- state ----------------
    def _publish_state(self) -> None:
        try:
            with self._lock:
                positions = self.bus.read_positions_rad()
        except Exception as exc:
            self.get_logger().warn(f"read positions failed: {exc}")
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [m.name for m in self.motors]
        msg.position = [positions[m.dxl_id] for m in self.motors]
        self._pub_state.publish(msg)

    # ---------------- joint command ----------------
    def _on_joint_cmd(self, msg: JointState) -> None:
        if not msg.name or len(msg.position) != len(msg.name):
            self.get_logger().warn("joint_command: name/position size mismatch")
            return
        wanted: Dict[int, float] = {}
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name not in self._id_by_name:
                    self.get_logger().warn(f"joint_command: unknown joint '{name}'")
                    continue
                self._goal_by_name[name] = pos
                wanted[self._id_by_name[name]] = pos
            if wanted:
                self._write_goals(wanted)

    # ---------------- pose / IK command ----------------
    def _on_pose_cmd(self, msg: PoseStamped) -> None:
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        pitch = _quat_to_pitch(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        try:
            q1, q2, q3, q4 = inverse_kinematics(x, y, z, pitch, elbow_up=self._elbow_up)
        except IKError as exc:
            self.get_logger().warn(f"IK fail for ({x:.3f},{y:.3f},{z:.3f},p={pitch:.2f}): {exc}")
            return
        self.get_logger().info(
            f"IK ok: target=({x:.3f},{y:.3f},{z:.3f},p={pitch:.2f}) -> "
            f"q=({q1:.3f},{q2:.3f},{q3:.3f},{q4:.3f})"
        )
        with self._lock:
            self._goal_by_name["joint1"] = q1
            self._goal_by_name["joint2"] = q2
            self._goal_by_name["joint3"] = q3
            self._goal_by_name["joint4"] = q4
            self._write_goals({
                self._id_by_name["joint1"]: q1,
                self._id_by_name["joint2"]: q2,
                self._id_by_name["joint3"]: q3,
                self._id_by_name["joint4"]: q4,
            })

    # ---------------- gripper ----------------
    def _on_gripper_cmd(self, msg: Float64) -> None:
        if not self._include_gripper:
            self.get_logger().warn("gripper disabled (include_gripper=false)")
            return
        with self._lock:
            self._goal_by_name[GRIPPER_NAME] = msg.data
            self._write_goals({self._id_by_name[GRIPPER_NAME]: msg.data})

    # ---------------- services ----------------
    def _srv_torque(self, req: SetBool.Request, res: SetBool.Response):
        try:
            with self._lock:
                if req.data:
                    self.bus.gentle_torque_on(run_vel=self._run_vel, run_acc=self._run_acc)
                else:
                    self.bus.set_torque(False)
            res.success = True
            res.message = "torque " + ("on" if req.data else "off")
        except Exception as exc:
            res.success = False
            res.message = f"{exc}"
        return res

    def _srv_home(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self._goto_joint_pose(POSE_HOME_UP, label="HOME_UP")
            res.success = True
            res.message = "moving to HOME_UP"
        except Exception as exc:
            res.success = False
            res.message = f"{exc}"
        return res

    def _srv_ready(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self._goto_joint_pose(POSE_READY_FORWARD, label="READY_FORWARD")
            res.success = True
            res.message = "moving to READY_FORWARD"
        except Exception as exc:
            res.success = False
            res.message = f"{exc}"
        return res

    # ---------------- helpers ----------------
    def _goto_joint_pose(self, q: tuple, label: str) -> None:
        self.get_logger().info(f"Going to {label}: q={q}")
        wanted: Dict[int, float] = {
            self._id_by_name["joint1"]: q[0],
            self._id_by_name["joint2"]: q[1],
            self._id_by_name["joint3"]: q[2],
            self._id_by_name["joint4"]: q[3],
        }
        with self._lock:
            self._goal_by_name["joint1"] = q[0]
            self._goal_by_name["joint2"] = q[1]
            self._goal_by_name["joint3"] = q[2]
            self._goal_by_name["joint4"] = q[3]
            self._write_goals(wanted)

    def _write_goals(self, wanted: Dict[int, float]) -> None:
        try:
            self.bus.write_goal_positions_rad(wanted)
        except Exception as exc:
            self.get_logger().warn(f"write goal failed: {exc}")

    # ---------------- shutdown ----------------
    def destroy_node(self):
        try:
            self.get_logger().info("Shutting down — leaving torque enabled. "
                                   "Call /enable_torque false to release.")
            self.bus.safe_shutdown()
            self.bus.close()
        finally:
            return super().destroy_node()


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    node = None
    try:
        node = OmxControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

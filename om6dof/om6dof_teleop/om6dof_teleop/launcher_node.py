"""F3-button watchdog for the Go2W remote.

Subscribes to `/wirelesscontroller`. When the F3 button (bit 7) rises:
  - if the teleop launch is NOT running → spawn `ros2 launch om6dof_teleop
    teleop.launch.py` as a child process
  - if it IS running → send SIGINT to the whole process group so the teleop
    shuts down gracefully

The child process inherits stdout/stderr, so when this node is launched
under systemd, everything ends up in `journalctl`.

Designed to be auto-started at Jetson boot via a systemd unit, so the user
can power the robot on and just tap F3 on the remote to start the arm.
"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_go.msg import WirelessController

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


BTN_F3 = 1 << 7

LAUNCH_CMD = ["ros2", "launch", "om6dof_teleop", "teleop.launch.py"]

# Dynamixel addresses we touch at boot for the partial-torque setup.
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
PROTOCOL_VERSION = 2.0


class ArmLauncher(Node):
    def __init__(self) -> None:
        super().__init__("om6dof_teleop_launcher")

        self.declare_parameter("debounce_seconds", 0.5)
        self.declare_parameter("stop_timeout_seconds", 5.0)
        self.debounce = float(self.get_parameter("debounce_seconds").value)
        self.stop_timeout = float(self.get_parameter("stop_timeout_seconds").value)

        # ---- boot-time torque setup ----
        # On launcher startup (= Jetson boot when running via systemd):
        # torque ON the IDs in `boot_torque_on_ids`, torque OFF the IDs in
        # `boot_torque_off_ids`. IDs in neither list are left untouched.
        self.declare_parameter("device", "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("boot_torque_on_ids", [31, 24])
        self.declare_parameter("boot_torque_off_ids", [32, 33, 35, 26, 37])

        self._apply_boot_torque_state()

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            WirelessController, "/wirelesscontroller", self._on_remote, qos
        )

        self.lock = threading.Lock()
        self.child: Optional[subprocess.Popen] = None
        self.prev_keys = 0
        self.last_toggle_t = 0.0

        # Periodically reap the child if it died on its own (crash, Ctrl-C, …)
        self.create_timer(1.0, self._monitor_child)

        self.get_logger().info(
            "Arm launcher ready — tap F3 on the Go2W remote to start/stop "
            "om6dof_teleop teleop."
        )

    # ---------------- boot-time torque state ----------------
    def _apply_boot_torque_state(self) -> None:
        """Open the bus, ping, set per-ID torque state, release the bus.

        Designed to run ONCE in __init__ before the spawn loop starts. The
        port is released afterwards so a later F3 → teleop_node can grab it.
        Errors are logged but never raised — a missing bus must not prevent
        the launcher from listening for F3.
        """
        device = str(self.get_parameter("device").value)
        baud = int(self.get_parameter("baudrate").value)
        on_ids: List[int] = [int(x) for x in self.get_parameter("boot_torque_on_ids").value]
        off_ids: List[int] = [int(x) for x in self.get_parameter("boot_torque_off_ids").value]
        all_ids = list(dict.fromkeys(on_ids + off_ids))  # de-dup, preserve order
        if not all_ids:
            self.get_logger().info("boot torque: no IDs configured, skipping")
            return

        self.get_logger().info(
            f"boot torque: ON={on_ids}  OFF={off_ids}  (device={device})"
        )
        port = PortHandler(device)
        pkt = PacketHandler(PROTOCOL_VERSION)
        try:
            if not port.openPort():
                self.get_logger().error(
                    f"boot torque: cannot open {device}; skipping"
                )
                return
            if not port.setBaudRate(baud):
                self.get_logger().error(
                    f"boot torque: setBaudRate {baud} failed; skipping"
                )
                return

            live: List[int] = []
            for mid in all_ids:
                ok = False
                for _ in range(3):
                    _, rc, err = pkt.ping(port, mid)
                    if rc == COMM_SUCCESS and err == 0:
                        ok = True
                        break
                    time.sleep(0.1)
                if ok:
                    live.append(mid)
                else:
                    self.get_logger().warn(f"  ID {mid}: no response (skipped)")

            # Auto-reboot any motor that we want to torque ON but currently has
            # a Hardware Error flag, otherwise the torque write will be ignored.
            for mid in list(set(on_ids) & set(live)):
                he, rc, _ = pkt.read1ByteTxRx(port, mid, ADDR_HARDWARE_ERROR)
                if rc == COMM_SUCCESS and he != 0:
                    self.get_logger().warn(
                        f"  ID {mid}: HW error 0x{he:02X} → reboot"
                    )
                    pkt.reboot(port, mid)
                    time.sleep(1.5)

            for mid in live:
                target = 1 if mid in on_ids else 0
                rc, err = pkt.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, target)
                if rc == COMM_SUCCESS and err == 0:
                    self.get_logger().info(
                        f"  ID {mid}: torque {'ON' if target else 'OFF'}"
                    )
                else:
                    self.get_logger().warn(
                        f"  ID {mid}: torque write failed (rc={rc} err={err})"
                    )
        except Exception as exc:
            self.get_logger().error(f"boot torque: unexpected error: {exc}")
        finally:
            try:
                port.closePort()
            except Exception:
                pass

    # ---------------- remote callback ----------------
    def _on_remote(self, msg: WirelessController) -> None:
        keys = int(msg.keys)
        rising = (~self.prev_keys) & keys
        self.prev_keys = keys
        if rising & BTN_F3:
            now = time.monotonic()
            if now - self.last_toggle_t < self.debounce:
                return
            self.last_toggle_t = now
            self._toggle()

    # ---------------- child process management ----------------
    def _toggle(self) -> None:
        with self.lock:
            # Aggressively reap a dead child here too — don't wait for the 1Hz
            # _monitor_child timer. Otherwise a quick F3 right after the
            # teleop's F1 park sequence could see a stale reference and be
            # treated as a stop.
            if self.child is not None:
                rc = self.child.poll()
                if rc is not None:
                    self.get_logger().info(
                        f"  previous teleop already exited (rc={rc}); "
                        f"clearing reference"
                    )
                    self.child = None

            if self.child is not None:
                self.get_logger().info("F3 → stopping teleop")
                self._kill_child_locked()
            else:
                self.get_logger().info("F3 → starting teleop")
                self._spawn_child_locked()

    def _spawn_child_locked(self) -> None:
        try:
            self.child = subprocess.Popen(
                LAUNCH_CMD,
                env=os.environ.copy(),
                preexec_fn=os.setsid,  # own process group → can SIGINT the tree
            )
            self.get_logger().info(f"  spawned pid={self.child.pid}")
        except Exception as exc:
            self.get_logger().error(f"failed to spawn teleop: {exc}")
            self.child = None

    def _kill_child_locked(self) -> None:
        if self.child is None:
            return
        try:
            pgid = os.getpgid(self.child.pid)
            os.killpg(pgid, signal.SIGINT)
            try:
                self.child.wait(timeout=self.stop_timeout)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    "teleop did not exit on SIGINT; sending SIGTERM"
                )
                os.killpg(pgid, signal.SIGTERM)
                try:
                    self.child.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        finally:
            self.child = None

    def _monitor_child(self) -> None:
        with self.lock:
            if self.child is None:
                return
            rc = self.child.poll()
            if rc is not None:
                self.get_logger().info(f"teleop exited (rc={rc})")
                self.child = None

    # ---------------- shutdown ----------------
    def destroy_node(self):
        with self.lock:
            if self.child is not None and self.child.poll() is None:
                self.get_logger().info("launcher shutting down → stopping teleop")
                self._kill_child_locked()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

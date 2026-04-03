#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState

BUTTON_MAP = {
    0:  "R1",
    1:  "L1",
    2:  "Start",
    3:  "Select",
    4:  "R2",
    5:  "L2",
    6:  "F1",
    7:  "F2",
    8:  "A",
    9:  "B",
    10: "X",
    11: "Y",
    12: "Up",
    13: "Right",
    14: "Down",
    15: "Left",
}


class RemoteMonitor(Node):
    def __init__(self):
        super().__init__("remote_monitor")
        self.sub = self.create_subscription(LowState, "/lowstate", self.callback, 10)
        self.prev_buttons = set()
        self.get_logger().info("Monitoring remote controller... Press Ctrl+C to stop.")

    def callback(self, msg):
        wr = msg.wireless_remote

        # Button bitmap from byte 2-3 (little-endian uint16)
        key_value = wr[2] | (wr[3] << 8)

        # Decode joystick axes (byte 7, 11, 15 are the main axis bytes, 128=center)
        lx = wr[7] - 128   # left stick X
        ly = wr[6] - 128   # left stick Y (placeholder offset)
        rx = wr[11] - 128  # right stick X
        ry = wr[15] - 128  # right stick Y

        # Decode pressed buttons
        pressed = set()
        for bit, name in BUTTON_MAP.items():
            if key_value & (1 << bit):
                pressed.add(name)

        # Only print when something changes or buttons are pressed
        if pressed != self.prev_buttons:
            if pressed:
                self.get_logger().info(
                    f"Buttons: {', '.join(sorted(pressed))}  "
                    f"(raw=0x{key_value:04X})  "
                    f"Sticks: lx={lx:+4d} ly={ly:+4d} rx={rx:+4d} ry={ry:+4d}"
                )
            else:
                self.get_logger().info("Released all buttons")
            self.prev_buttons = pressed


def main():
    rclpy.init()
    node = RemoteMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

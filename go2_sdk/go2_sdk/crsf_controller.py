import threading
from enum import IntEnum

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Joy


class PacketsTypes(IntEnum):
    RC_CHANNELS_PACKED = 0x16
    SYNC_BYTE = 0xC8


def crc8_dvb_s2(crc, a) -> int:
    crc = crc ^ a
    for ii in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc = crc << 1
    return crc & 0xFF


def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc


def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]


def n(val):
    res = val - 174
    res = res / 1632
    if res > 1.0:
        res = 1.0
    elif res < 0.0:
        res = 0.0
    return round(res, 2)


class CrsfJoyBridge(Node):
    def __init__(self):
        super().__init__("crsf_joy_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baud_rate", 420000)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("deadband", 0.3)

        self.port = self.get_parameter("serial_port").value
        self.baud = self.get_parameter("baud_rate").value
        self.deadband = self.get_parameter("deadband").value
        self.publish_rate = self.get_parameter("publish_rate").value

        self.joy_pub = self.create_publisher(Joy, "joy", 10)

        self.serial_thread = None
        self.running = False

        self.latest_joy = Joy()
        self.latest_joy.header.frame_id = "crsf_controller"
        self.latest_joy.axes = [0.0] * 4  # LLR, LUD, RLR, RUD (centered at 0)
        self.latest_joy.buttons = [0] * 6  # SWA through SWF

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joy)

        self.start_serial()

        self.get_logger().info(
            f"CRSF Joy Bridge started on {self.port} at {self.baud} baud"
        )

    def start_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.running = True
            self.serial_thread = threading.Thread(
                target=self.serial_reader, daemon=True
            )
            self.serial_thread.start()
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

    def serial_reader(self):
        input_buffer = bytearray()

        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    input_buffer.extend(self.ser.read(self.ser.in_waiting))

                while len(input_buffer) > 2:
                    expected_len = input_buffer[1] + 2
                    if expected_len > 64 or expected_len < 4:
                        input_buffer = bytearray()
                    elif len(input_buffer) >= expected_len:
                        single = input_buffer[:expected_len]
                        input_buffer = input_buffer[expected_len:]

                        if single[0] == PacketsTypes.SYNC_BYTE:
                            if crsf_validate_frame(single):
                                self.handle_crsf_packet(single[2], single)
                    else:
                        break

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

    def handle_crsf_packet(self, ptype, data):
        if ptype == PacketsTypes.RC_CHANNELS_PACKED:
            packet = data[2:-1]
            packet = packet[1:-1]  # remove type and crc
            packet_bin_8 = ["{0:08b}".format(i)[::-1] for i in packet]
            packet_bin_full = "".join(packet_bin_8)
            packet_bin_11 = [packet_bin_full[11 * i : 11 * (i + 1)] for i in range(16)]
            rc_packet = [int(b[::-1], 2) for b in packet_bin_11]

            # Filter out noise
            if max(rc_packet) > 2000:
                return

            # Convert to Joy message
            # Map sticks to -1.0 to +1.0 range (centered at 0)
            lud = (n(rc_packet[2]) - 0.5) * 2.0  # Left stick up/down
            llr = (n(rc_packet[3]) - 0.5) * 2.0  # Left stick left/right
            rud = (n(rc_packet[0]) - 0.5) * 2.0  # Right stick up/down
            rlr = (n(rc_packet[1]) - 0.5) * -2.0  # Right stick left/right

            if abs(lud) < self.get_parameter("deadband").value:
                lud = 0.0

            # Map switches to buttons (0 or 1)
            swa = 1 if n(rc_packet[4]) > 0.6 else 0
            swb_val = n(rc_packet[5])
            swb = 0 if swb_val <= 0.33 else 2 if swb_val >= 0.66 else 1
            swc_val = n(rc_packet[6])
            swc = 0 if swc_val <= 0.33 else 2 if swc_val >= 0.66 else 1
            swd = 1 if n(rc_packet[7]) > 0.6 else 0
            swe = 1 if n(rc_packet[8]) > 0.6 else 0
            swf = 1 if n(rc_packet[9]) > 0.6 else 0

            # Update latest state
            self.latest_joy.axes = [llr, lud, rlr, rud]  # Standard gamepad axis order
            self.latest_joy.buttons = [swa, swb, swc, swd, swe, swf]

    def publish_joy(self):
        self.latest_joy.header.stamp = self.get_clock().now().to_msg()
        self.joy_pub.publish(self.latest_joy)

    def destroy_node(self):
        self.running = False
        if self.serial_thread:
            self.serial_thread.join(timeout=1.0)
        if hasattr(self, "ser"):
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CrsfJoyBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

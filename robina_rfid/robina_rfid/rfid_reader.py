import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

# Serial communication settings
SERIAL_PORT = "/dev/ttyUSB1"  # Update if needed
BAUD_RATE = 115200

# Commands
START_TAG_READING_CMD = bytes.fromhex("43 4D 02 02 02 00 00 00 00")
STOP_TAG_READING_CMD  = bytes.fromhex("43 4D 03 02 02 00 00 00 00")
SET_ANTENNA_1_CMD = bytes.fromhex(
    "43 4D 86 02 1D 00 00 00"
    "00"
    "04"
    "01"
    "01 1E 00 C8 00 64"
    "02 1E 00 C8 00 64"
    "03 1E 00 C8 00 64"
    "04 1E 00 C8 00 64"
    "09"
)

def parse_tags(raw_bytes):
    tags = []
    i = 0
    while i < len(raw_bytes) - 10:
        if raw_bytes[i:i+2] != b'\x43\x4D':
            i += 1
            continue
        try:
            length = raw_bytes[i+4] + (raw_bytes[i+5] << 8)
            frame_end = i + 8 + length + 1
            frame = raw_bytes[i:frame_end]

            if len(frame) < 20:
                i += 1
                continue

            tag_len = frame[9]
            tag_id = frame[11:11+tag_len].hex().upper()
            tags.append(tag_id)
            i = frame_end
        except:
            i += 1
    return tags

class RFIDReaderNode(Node):
    def __init__(self):
        super().__init__('rfid_reader_node')
        self.publisher_ = self.create_publisher(String, 'tag_info', 10)
        self.serial_port = SERIAL_PORT
        self.baud_rate = BAUD_RATE

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            return

        self.ser.write(SET_ANTENNA_1_CMD)
        time.sleep(0.2)
        self.ser.write(START_TAG_READING_CMD)
        self.get_logger().info("Started tag reading")
        self.buffer = bytearray()
        self.create_timer(0.5, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            self.buffer += self.ser.read(self.ser.in_waiting)

        tags = parse_tags(self.buffer)
        self.buffer.clear()

        for tag in tags:
            self.publisher_.publish(String(data=tag))
            self.get_logger().info(f"Published tag: {tag}")

    def destroy_node(self):
        self.ser.write(STOP_TAG_READING_CMD)
        self.ser.close()
        self.get_logger().info("Stopped tag reading and closed serial")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RFIDReaderNode()
    if hasattr(node, 'ser') and node.ser.is_open:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

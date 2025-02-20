import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class RFIDReaderNode(Node):
    def __init__(self):
        super().__init__('rfid_reader_node')
        self.serial_port = '/dev/ttyUSB1'
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            return
        
        self.publisher_ = self.create_publisher(String, 'tag_info', 10)
        self.create_timer(0.1, self.read_tag)
        
        self.send_at_command("AT+ANT=1")
        self.send_at_command("AT+PWR=27,0,0,0")
        
        self.tag_buffer = set()
        self.read_count = 0

    def send_at_command(self, command):
        self.ser.write((command + '\r\n').encode())
        self.ser.readline()

    def read_tag(self):
        self.send_at_command("AT+INV")
        
        while True:
            response = self.ser.readline().decode('utf-8').strip()
            if "+INV:" in response:
                self.tag_buffer.add(response.split("+INV:")[-1].strip())
            elif "OK" in response or "ERROR" in response:
                break
        
        self.read_count += 1
        if self.read_count >= 10:
            if self.tag_buffer:
                self.publisher_.publish(String(data=' '.join(self.tag_buffer)))
            self.tag_buffer.clear()
            self.read_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = RFIDReaderNode()
    if node.ser.is_open:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


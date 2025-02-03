import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial  # Assuming the reader communicates over UART

class RFIDReaderNode(Node):
	def __init__(self):
    	super().__init__(rfid_node)
    	self.publisher_ = self.create_publisher(String, '/rfid_node/tags', 10)
    	self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust as needed
    	self.timer = self.create_timer(0.1, self.read_rfid)

	def read_rfid(self):
    	if self.ser.in_waiting > 0:
        	tag_data = self.ser.readline().decode('utf-8').strip()
        	if tag_data:
            	msg = String()
            	msg.data = tag_data
            	self.publisher_.publish(msg)
            	self.get_logger().info(f"Published RFID Tag: {tag_data}")

def main(args=None):
	rclpy.init(args=args)
	node = RFIDReaderNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import String for a single string message
import serial

class RFIDReaderNode(Node):
    def __init__(self):
        super().__init__('rfid_reader_node')

        # Serial port configuration
        self.serial_port = '/dev/ttyUSB0'  # Update with your serial port!!!
        self.baud_rate = 115200  # Match the reader's baud rate

        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            return  # Stop execution if serial connection fails

        # Create publisher for tag data
        self.publisher_ = self.create_publisher(String, 'tag_info', 10)  # Use String for a single string message

        # Set up a timer to read tags periodically
        self.timer = self.create_timer(0.05, self.read_tag)  # Read every 0.1 seconds

        # Initialize the RFID reader with AT commands
        self.send_at_command("AT+ANT=1")  # Enable antenna
        self.send_at_command("AT+PWR=32,0,0,0")  # Set power

        # Buffer to store unique tags over 1 second
        self.tag_buffer = set()  # Use a set to ensure uniqueness
        self.read_count = 0  # Counter to track the number of reads

    def send_at_command(self, command):
        """Send AT command to the RFID reader."""
        self.ser.write((command + '\r\n').encode())  # Send the command
        self.ser.readline()  # Read and discard the response

    def read_tag(self):
        """Read tag data from the RFID reader."""
        self.send_at_command("AT+INV")  # Start inventory scan

        while True:
            response = self.ser.readline().decode('utf-8').strip()  # Read response line by line

            if "+INV:" in response:
                # Extract the tag ID from the response
                tag_id = response.split("+INV:")[-1].strip()
                self.tag_buffer.add(tag_id)  # Add the tag ID to the set (duplicates are ignored)
            elif "OK" in response:
                # End of response, break the loop
                break
            elif "ERROR" in response:
                # Handle error case
                self.get_logger().error(f"Error during inventory scan: {response}")
                break

        self.read_count += 1

        # If 1 second has passed (10 reads at 0.1-second intervals), publish the buffer
        if self.read_count >= 10:
            if self.tag_buffer:
                # Create a single string with unique tags separated by commas
                tag_string_msg = String()
                tag_string_msg.data = ', '.join(self.tag_buffer)  # Join unique tags into a single string

                # Publish the string of tags
                self.publisher_.publish(tag_string_msg)

            # Reset the buffer and counter
            self.tag_buffer = set()
            self.read_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = RFIDReaderNode()

    if node.ser.is_open:  # Ensure the node is running only if the serial connection was successful
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

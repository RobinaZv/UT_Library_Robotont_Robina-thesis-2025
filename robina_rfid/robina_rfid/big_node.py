import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry  # Import Odometry message type
import subprocess
import time

class BigNode(Node):
    def __init__(self):
        super().__init__('big_node')
        
        # Launch rfid_reader node
        self.get_logger().info("Starting rfid_reader node...")
        self.rfid_process = subprocess.Popen(["ros2", "run", "robina_rfid", "rfid_reader"], 
                                             stdout=subprocess.PIPE, 
                                             stderr=subprocess.PIPE, 
                                             text=True)
        time.sleep(2)  # Give some time for the rfid_reader node to start

        self.subscription = self.create_subscription(
            String,
            'tag_info',
            self.tag_callback,
            10
        )

        # Subscribe to the odom topic to get position data
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace with the correct topic name if it's different
            self.odom_callback,
            10
        )

        self.subscription  # Prevent unused variable warning
        
        # Initialize variables to store the latest data
        self.latest_tag = ""
        self.latest_position = (0.0, 0.0, 0.0)

        # Timer to print information every second
        self.timer = self.create_timer(1.0, self.print_data)  # 1 second interval

    def tag_callback(self, msg):
        # Update the latest tag information
        self.latest_tag = msg.data

    def odom_callback(self, msg):
        # Update the latest position data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.latest_position = (x, y, z)

    def print_data(self):
        # Print both the tag and position data once every second
        print(f"Tag: {self.latest_tag} | x,y,z={self.latest_position[0]},{self.latest_position[1]},{self.latest_position[2]}")

    def destroy_node(self):
        self.get_logger().info("Shutting down rfid_reader node...")
        self.rfid_process.terminate()  # Stop the rfid_reader process
        self.rfid_process.wait()  # Ensure it fully stops
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BigNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


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
        # Launch locator node
        self.get_logger().info("Starting locator node...")
        self.loc_process = subprocess.Popen(["ros2", "run", "robina_rfid", "tag_location_processor"], 
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

        self.publisher = self.create_publisher(String, '/tag_loc', 10)
        
        self.subscription  # Prevent unused variable warning
        
        # Initialize variables to store the latest data
        self.latest_tags = []
        self.latest_position = (0.0, 0.0, 0.0)
        self.last_published_data = ""

        # Timer to publish information every second
        self.timer = self.create_timer(1.0, self.publish_data)  # 1 second interval

    def tag_callback(self, msg):
        # Update the latest tag information, ignoring tags that start with '<'
        tags = [tag.strip() for tag in msg.data.split(',') if tag.strip() and not tag.strip().startswith('<')]
        if tags:
            self.latest_tags = tags

    def odom_callback(self, msg):
        # Update the latest position data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.latest_position = (x, y, z)

    def publish_data(self):
        if self.latest_tags:
            tag_loc_msg = String()
            tag_ids = ' '.join(self.latest_tags)  # Join tags with spaces
            position_str = f"{self.latest_position[0]} {self.latest_position[1]} {self.latest_position[2]}"
            new_data = f"{tag_ids} {position_str}"
            
            if new_data != self.last_published_data:
                tag_loc_msg.data = new_data  # Assign data before publishing
                self.publisher.publish(tag_loc_msg)
                print(f"{new_data}")
                self.last_published_data = new_data

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


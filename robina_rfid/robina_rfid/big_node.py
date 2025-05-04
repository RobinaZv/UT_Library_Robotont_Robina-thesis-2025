import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Float32
from nav_msgs.msg import Odometry
import subprocess
import time

class BigNode(Node):
    def __init__(self):
        super().__init__('big_node')

        # Launch rfid_reader node
        self.get_logger().info("Starting rfid_reader node...")
        self.rfid_process = subprocess.Popen(
            ["ros2", "run", "robina_rfid", "rfid_reader"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        # Launch locator node
        self.get_logger().info("Starting locator node...")
        self.loc_process = subprocess.Popen(
            ["ros2", "run", "robina_rfid", "tag_location_processor"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        time.sleep(2)  # Give some time for the rfid_reader node to start

        self.subscription = self.create_subscription(
            String,
            'tag_info',
            self.tag_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.z_subscription = self.create_subscription(
            Float32,
            '/z_height',
            self.z_callback,
            10
        )

        self.publisher = self.create_publisher(String, '/tag_loc', 10)

        # Initialize with z = 1.5
        self.z_height = 1.5
        self.z_overridden = False  # Tracks if z was changed from the initial value
        self.latest_tags = []
        self.latest_position = (0.0, 0.0, self.z_height)
        self.last_published_data = ""

        self.timer = self.create_timer(1.0, self.publish_data)

    def tag_callback(self, msg):
        tags = [tag.strip() for tag in msg.data.split(',') if tag.strip() and not tag.strip().startswith('<')]
        if tags:
            self.latest_tags = tags

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Use internally tracked z_height instead of msg.pose.pose.position.z
        self.latest_position = (x, y, self.z_height)

    def z_callback(self, msg):
        if not self.z_overridden and msg.data != 1.5:
            self.get_logger().info(f"Overriding initial z_height with value: {msg.data}")
            self.z_height = msg.data
            self.z_overridden = True
        elif self.z_overridden:
            self.z_height = msg.data
        # Update latest_position z
        x, y, _ = self.latest_position
        self.latest_position = (x, y, self.z_height)

    def publish_data(self):
        if self.latest_tags:
            tag_ids = ' '.join(self.latest_tags)
            position_str = f"{self.latest_position[0]} {self.latest_position[1]} {self.latest_position[2]}"
            new_data = f"{tag_ids} {position_str}"

            if new_data != self.last_published_data:
                msg = String()
                msg.data = new_data
                self.publisher.publish(msg)
                print(new_data)
                self.last_published_data = new_data

    def destroy_node(self):
        self.get_logger().info("Shutting down rfid_reader node...")
        self.rfid_process.terminate()
        self.rfid_process.wait()
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

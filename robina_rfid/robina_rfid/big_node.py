import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import subprocess
import time
import math

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

        # Subscriptions
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

        self.waypoint_subscription = self.create_subscription(
            String,
            '/waypoints',
            self.waypoint_callback,
            10
        )

        # Publishers
        self.publisher = self.create_publisher(String, '/tag_loc', 10)
        self.steps_publisher = self.create_publisher(String, '/steps', 10)

        # Initialize state
        self.latest_tags = []
        self.latest_position = (0.0, 0.0, 0.0)
        self.last_published_data = ""
        self.current_waypoint = None
        self.waypoint_reached = False  # Prevent re-triggering

        # Timer to publish tag info every second
        self.timer = self.create_timer(1.0, self.publish_data)

    def tag_callback(self, msg):
        tags = [tag.strip() for tag in msg.data.split(',') if tag.strip() and not tag.strip().startswith('<')]
        if tags:
            self.latest_tags = tags

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.latest_position = (x, y, z)

        # Log the position
        self.get_logger().info(f"Received odometry: ({x:.2f}, {y:.2f}, {z:.2f})")

        # If we have a current waypoint, check distance
        if self.current_waypoint and not self.waypoint_reached:
            wx, wy = self.current_waypoint
            x_distance = abs(x - wx)
            y_distance = abs(y - wy)

            # Log x and y distances
            self.get_logger().info(f"Distance to waypoint ({wx:.2f}, {wy:.2f}): x_distance = {x_distance:.2f}, y_distance = {y_distance:.2f}")

            # Check if either x or y distance is less than 1 meter
            if x_distance < 1.0 or y_distance < 1.0:
                self.get_logger().info(f"Waypoint reached at ({wx:.2f}, {wy:.2f}). Sending '-40000' to /steps.")
                msg = String()
                msg.data = "-40000"
                self.steps_publisher.publish(msg)
                self.waypoint_reached = True

    def waypoint_callback(self, msg):
        try:
            x_str, y_str = msg.data.split(',')
            self.current_waypoint = (float(x_str.strip()), float(y_str.strip()))
            self.waypoint_reached = False  # Reset flag for new waypoint
            self.get_logger().info(f"Received new waypoint: {self.current_waypoint}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse waypoint: {msg.data}. Error: {str(e)}")

    def publish_data(self):
        if self.latest_tags:
            tag_loc_msg = String()
            tag_ids = ' '.join(self.latest_tags)
            position_str = f"{self.latest_position[0]} {self.latest_position[1]} {self.latest_position[2]}"
            new_data = f"{tag_ids} {position_str}"

            if new_data != self.last_published_data:
                tag_loc_msg.data = new_data
                self.publisher.publish(tag_loc_msg)
                self.get_logger().info(f"Published tag location data: {new_data}")
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


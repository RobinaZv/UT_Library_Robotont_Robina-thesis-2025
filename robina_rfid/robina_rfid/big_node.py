import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
import subprocess
import time

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point

class BigNode(Node):
    def __init__(self):
        super().__init__('big_node')

        # Launch rfid_reader node
        self.get_logger().info("Starting rfid_reader node...")
        self.rfid_process = subprocess.Popen(
            ["ros2", "run", "robina_rfid", "rfid_reader"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True
        )

        # Launch tag_location_processor node
        self.get_logger().info("Starting tag_location_processor node...")
        self.loc_process = subprocess.Popen(
            ["ros2", "run", "robina_rfid", "tag_location_processor"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True
        )

        time.sleep(2)  # Allow background nodes to start

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(String, 'tag_info', self.tag_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/z_height', self.z_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(String, '/tag_loc', 10)

        # Internal state
        self.z_height = 1.5
        self.z_overridden = False
        self.latest_position = (0.0, 0.0, self.z_height)
        self.latest_tag_positions = []  # list of (tag_id, x, y, z)

        self.timer = self.create_timer(1.0, self.publish_data)

    def tag_callback(self, msg):
        tags = [tag.strip() for tag in msg.data.split(',') if tag.strip() and not tag.strip().startswith('<')]
        if tags:
            x, y, z = self.latest_position
            for tag in tags:
                self.latest_tag_positions.append((tag, x, y, z))

    def odom_callback(self, msg):
        try:
            point = PointStamped()
            point.header = msg.header
            point.point = msg.pose.pose.position

            # Transform robot position from odom → map
            transform = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time()
            )
            transformed_point = do_transform_point(point, transform)
            x = transformed_point.point.x
            y = transformed_point.point.y
            self.latest_position = (x, y, self.z_height)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform failed: {str(e)}")

    def z_callback(self, msg):
        if not self.z_overridden and msg.data != 1.5:
            self.get_logger().info(f"Overriding initial z_height with: {msg.data}")
            self.z_overridden = True

        self.z_height = msg.data
        x, y, _ = self.latest_position
        self.latest_position = (x, y, self.z_height)

    def publish_data(self):
        if not self.latest_tag_positions:
            return

        for tag_id, x, y, z in self.latest_tag_positions:
            msg = String()
            msg.data = f"{tag_id} {x:.2f} {y:.2f} {z:.2f}"
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

        self.latest_tag_positions.clear()

    def destroy_node(self):
        self.get_logger().info("Shutting down background nodes...")
        self.rfid_process.terminate()
        self.loc_process.terminate()
        self.rfid_process.wait()
        self.loc_process.wait()
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

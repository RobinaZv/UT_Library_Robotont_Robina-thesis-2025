import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')

        # Create a subscriber to the /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Name of the topic you want to subscribe to
            self.listener_callback,
            10
        )

        # Create a publisher for the /filtered_scan topic
        self.publisher = self.create_publisher(
            LaserScan,
            '/filtered_scan',  # The topic to publish filtered scan data
            10
        )

    def listener_callback(self, msg):
        # Process the LaserScan message and filter out values between 0 and 180 degrees
        # LaserScan ranges are in degrees from 0 to 360, but we'll be filtering 0-180.

        filtered_ranges = []

        for i, range_value in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment  # Get the angle of the current scan point
            if 0 <= angle <= math.radians(180):  # Filter out values between 0 and 180 degrees
                filtered_ranges.append(float('inf'))  # Set these values to infinity
            else:
                filtered_ranges.append(range_value)  # Keep the original value

        # Now, modify the LaserScan message with the filtered ranges
        msg.ranges = filtered_ranges

        # Log the filtered scan data (optional)
        self.get_logger().info(f"Filtered scan data: {msg.ranges[:10]}...")  # Print the first 10 ranges for debugging

        # Publish the filtered LaserScan message to the /filtered_scan topic
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    laser_filter_node = LaserFilterNode()
    rclpy.spin(laser_filter_node)
    laser_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

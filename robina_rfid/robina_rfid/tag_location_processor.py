import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

class TagLocationProcessor(Node):
    def __init__(self):
        super().__init__('tag_location_processor')

        # Subscribe to /tag_loc topic
        self.subscription = self.create_subscription(
            String,
            '/tag_loc',
            self.tag_loc_callback,
            10
        )

        self.tag_positions = {}  # Dictionary to store tag locations

        # Timer to save median values to file every 5 seconds
        self.median_timer = self.create_timer(5.0, self.save_medians_to_file)

    def tag_loc_callback(self, msg):
        """ Callback to process incoming tag location messages. """
        data = msg.data.strip()
        if not data:
            return

        # Extract tag IDs and coordinates
        try:
            parts = data.split()
            tags = parts[:-3]  # All except last three are tag IDs
            x, y = map(float, parts[-3:-1])  # Last three are x, y; z will be added as 0.0
            z = 0.0
        except ValueError:
            self.get_logger().warn(f"Invalid message format: {data}")
            return

        # Discard messages with <NO TAGS FOUND>
        tags = [tag for tag in tags if tag != "<NO TAGS FOUND>"]

        # Store each tag's position history separately
        for tag in tags:
            if tag not in self.tag_positions:
                self.tag_positions[tag] = []
            self.tag_positions[tag].append((x, y, z))

        if tags:
            self.get_logger().info(f"Updated positions for {tags}")

    def calculate_medians_and_extremes(self):
        """ Compute the median position and furthest x, y, z from median for each tag. """
        results = {}
        for tag, positions in self.tag_positions.items():
            positions_np = np.array(positions)
            median_x = np.median(positions_np[:, 0])
            median_y = np.median(positions_np[:, 1])
            median_z = np.median(positions_np[:, 2])

            # Find furthest x, y, z from median
            furthest_index = np.argmax(np.linalg.norm(positions_np - np.array([median_x, median_y, median_z]), axis=1))
            furthest_x, furthest_y, furthest_z = positions_np[furthest_index]

            results[tag] = (median_x, median_y, median_z, furthest_x, furthest_y, furthest_z)
        return results

    def save_medians_to_file(self):
        """ Save the latest median positions and extreme values to a file. """
        results = self.calculate_medians_and_extremes()
        with open("tag_medians.txt", "w") as f:
            for tag, (med_x, med_y, med_z, furthest_x, furthest_y, furthest_z) in results.items():
                f.write(f"{tag} {med_x} {med_y} {med_z} {furthest_x} {furthest_y} {furthest_z}\n")
        self.get_logger().info("Median values and extremes saved to tag_medians.txt")


def main(args=None):
    rclpy.init(args=args)
    node = TagLocationProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_medians_to_file()  # Save medians before shutting down
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


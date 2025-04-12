import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import numpy as np

class TagLocationProcessor(Node):
    def __init__(self):
        super().__init__('tag_location_processor')

        # Subscriptions
        self.subscription = self.create_subscription(
            String,
            '/tag_loc',
            self.tag_loc_callback,
            10
        )
        self.z_subscription = self.create_subscription(
            Float32,
            '/z_height',
            self.z_height_callback,
            10
        )

        # Tag position buffer
        self.tag_positions = {}  # Dictionary to store tag locations
        self.current_z = 0.0     # Default z height

        # Timer to periodically save results
        self.median_timer = self.create_timer(5.0, self.save_medians_to_file)

    def z_height_callback(self, msg: Float32):
        """ Updates the current z-height from /z_height topic. """
        self.current_z = msg.data
        self.get_logger().info(f"Updated z_height: {self.current_z:.2f}")

    def tag_loc_callback(self, msg: String):
        """ Processes incoming tag location messages. """
        data = msg.data.strip()
        if not data:
            return

        try:
            parts = data.split()
            tags = parts[:-3]  # All except last three are tag IDs
            x, y = map(float, parts[-3:-1])
            z = self.current_z  # Use latest z from /z_height
        except ValueError:
            self.get_logger().warn(f"Invalid message format: {data}")
            return

        # Filter out invalid tags
        tags = [tag for tag in tags if tag != "<NO TAGS FOUND>"]

        for tag in tags:
            if tag not in self.tag_positions:
                self.tag_positions[tag] = []
            self.tag_positions[tag].append((x, y, z))

        if tags:
            self.get_logger().info(f"Stored position for tags: {tags}")

    def calculate_medians_and_extremes(self):
        """ Compute the median and furthest point per tag. """
        results = {}
        for tag, positions in self.tag_positions.items():
            positions_np = np.array(positions)
            median = np.median(positions_np, axis=0)
            furthest_idx = np.argmax(np.linalg.norm(positions_np - median, axis=1))
            furthest = positions_np[furthest_idx]
            results[tag] = (*median, *furthest)
        return results

    def save_medians_to_file(self):
        """ Save results to file. """
        results = self.calculate_medians_and_extremes()
        with open("tag_medians.txt", "w") as f:
            for tag, (med_x, med_y, med_z, furthest_x, furthest_y, furthest_z) in results.items():
                f.write(f"{tag} {med_x} {med_y} {med_z} {furthest_x} {furthest_y} {furthest_z}\n")
        self.get_logger().info("Saved median values and extremes to tag_medians.txt")

def main(args=None):
    rclpy.init(args=args)
    node = TagLocationProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_medians_to_file()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

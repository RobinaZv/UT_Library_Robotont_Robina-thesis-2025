import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

class TagLocationProcessor(Node):
    def __init__(self):
        super().__init__('tag_location_processor')

        self.subscription = self.create_subscription(
            String,
            '/tag_loc',
            self.tag_loc_callback,
            10
        )

        self.tag_positions = {}  # Store tag locations per tag ID

        self.median_timer = self.create_timer(5.0, self.save_medians_to_file)

    def tag_loc_callback(self, msg):
        data = msg.data.strip()
        self.get_logger().info(f"Received data: {data}")

        if not data:
            return

        try:
            parts = data.split()
            if len(parts) < 3:
                self.get_logger().warn(f"Too few elements in data: {data}")
                return

            tags = parts[:-3]  # All except last three are tag IDs
            x = float(parts[-3])
            y = float(parts[-2])
            z = float(parts[-1]) if len(parts) >= 3 else 1.5  # Default to 1.5 if not provided
        except ValueError:
            self.get_logger().warn(f"Invalid message format: {data}")
            return

        tags = [tag for tag in tags if tag != "<NO TAGS FOUND>"]

        for tag in tags:
            if tag not in self.tag_positions:
                self.tag_positions[tag] = []
            self.tag_positions[tag].append((x, y, z))

        self.get_logger().info(f"Updated positions for {tags}")

    def calculate_medians_and_extremes(self):
        results = {}
        for tag, positions in self.tag_positions.items():
            positions_np = np.array(positions)
            median_x = np.median(positions_np[:, 0])
            median_y = np.median(positions_np[:, 1])
            median_z = np.median(positions_np[:, 2])

            furthest_index = np.argmax(np.linalg.norm(positions_np - np.array([median_x, median_y, median_z]), axis=1))
            furthest_x, furthest_y, furthest_z = positions_np[furthest_index]

            results[tag] = (median_x, median_y, median_z, furthest_x, furthest_y, furthest_z)
        return results

    def save_medians_to_file(self):
        results = self.calculate_medians_and_extremes()
        self.get_logger().info(f"Calculated medians and extremes: {results}")

        with open("tag_medians.txt", "w") as f:
            for tag, (med_x, med_y, med_z, furthest_x, furthest_y, furthest_z) in results.items():
                f.write(f"{tag} {med_x} {med_y} {med_z} {furthest_x} {furthest_y} {furthest_z}\n")
            f.flush()

        self.get_logger().info("Median values and extremes saved to tag_medians.txt")

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

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
            tag_part, coords_part = data.rsplit(" x,y,z=", 1)
            x, y, z = map(float, coords_part.split(","))
        except ValueError:
            self.get_logger().warn(f"Invalid message format: {data}")
            return

        # Store each tag's position history separately
        tags = [tag.strip() for tag in tag_part.split(",")]
        for tag in tags:
            if tag not in self.tag_positions:
                self.tag_positions[tag] = []
            self.tag_positions[tag].append((x, y, z))

        self.get_logger().info(f"Updated positions for {tags}")

    def calculate_medians(self):
        """ Compute the median position for each tag separately. """
        median_positions = {}
        for tag, positions in self.tag_positions.items():
            positions_np = np.array(positions)
            median_x = np.median(positions_np[:, 0])
            median_y = np.median(positions_np[:, 1])
            median_z = np.median(positions_np[:, 2])
            median_positions[tag] = (median_x, median_y, median_z)
        return median_positions

    def save_medians_to_file(self):
        """ Save the latest median positions to a file, ensuring each tag is on a separate line. """
        median_positions = self.calculate_medians()
        with open("tag_medians.txt", "w") as f:
            for tag, (x, y, z) in median_positions.items():
                f.write(f"{tag} {x} {y} {z}\n")
        self.get_logger().info("Median values saved to tag_medians.txt")

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


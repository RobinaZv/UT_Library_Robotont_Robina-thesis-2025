import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
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

        self.marker_publisher = self.create_publisher(Marker, 'marker', 10)
        self.tag_positions = {}  # Store tag locations per tag ID

        self.median_timer = self.create_timer(5.0, self.save_medians_and_publish_markers)

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
            z = float(parts[-1]) if len(parts) >= 3 else 1.5  # Default z

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

    def save_medians_and_publish_markers(self):
        results = self.calculate_medians_and_extremes()
        self.get_logger().info(f"Calculated medians and extremes: {results}")

        # Save to file
        with open("tag_medians.txt", "w") as f:
            for tag, (med_x, med_y, med_z, furthest_x, furthest_y, furthest_z) in results.items():
                f.write(f"{tag} {med_x} {med_y} {med_z} {furthest_x} {furthest_y} {furthest_z}\n")
            f.flush()
        self.get_logger().info("Saved tag medians to file")

        # Publish markers
        for i, (tag, (med_x, med_y, med_z, furthest_x, furthest_y, furthest_z)) in enumerate(results.items()):
            scale_x = abs(furthest_x - med_x) * 2
            scale_y = abs(furthest_y - med_y) * 2
            scale_z = abs(furthest_z - med_z) * 2

            # SPHERE Marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.id = i * 2
            marker.action = Marker.ADD
            marker.scale.x = scale_x if scale_x > 0 else 0.1
            marker.scale.y = scale_y if scale_y > 0 else 0.1
            marker.scale.z = 0.01
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.2
            marker.pose.position.x = med_x
            marker.pose.position.y = med_y
            marker.pose.position.z = med_z
            self.marker_publisher.publish(marker)

            # TEXT Marker
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.id = i * 2 + 1
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.1
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.pose.position.x = med_x
            text_marker.pose.position.y = med_y
            text_marker.pose.position.z = med_z + 0.02
            text_marker.text = tag
            self.marker_publisher.publish(text_marker)

def main(args=None):
    rclpy.init(args=args)
    node = TagLocationProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_medians_and_publish_markers()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

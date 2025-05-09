#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import os

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'marker', 10)
        self.markers = []
        
        # Path to tag_medians.txt in colcon_ws
        workspace_path = os.path.expanduser('~/colcon_ws')
        file_path = os.path.join(workspace_path, 'tag_medians.txt')
        
        self.load_markers(file_path)
        self.timer = self.create_timer(0.5, self.publish_markers)
    
    def load_markers(self, file_path):
        try:
            with open(file_path, 'r') as file:
                for i, line in enumerate(file.readlines()):
                    parts = line.strip().split()
                    if len(parts) != 7:
                        self.get_logger().warn(f"Skipping invalid line: {line}")
                        continue
                    
                    tag_name, med_x, med_y, med_z, furthest_x, furthest_y, furthest_z = (
                        parts[0], float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
                    )
                    
                    # Calculate scale based on furthest x, y, and z deviations
                    scale_x = abs(furthest_x - med_x) * 2  # Diameter in x-direction
                    scale_y = abs(furthest_y - med_y) * 2  # Diameter in y-direction
                    scale_z = abs(furthest_z - med_z) * 2  # Diameter in z-direction
                    
                    # Create ellipsoid marker
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.type = Marker.SPHERE  # Ellipsoid is a scaled sphere
                    marker.id = i * 2
                    marker.action = Marker.ADD
                    marker.scale.x = scale_x if scale_x > 0 else 0.1
                    marker.scale.y = scale_y if scale_y > 0 else 0.1
                    marker.scale.z = 0.01
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.2  # Transparent marker
                    marker.pose.position.x = med_x
                    marker.pose.position.y = med_y
                    marker.pose.position.z = med_z
                    
                    # Create text marker
                    text_marker = Marker()
                    text_marker.header.frame_id = 'map'
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.id = i * 2 + 1
                    text_marker.action = Marker.ADD
                    text_marker.scale.z = 0.1  # Text size
                    text_marker.color.r = 1.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 1.0
                    text_marker.color.a = 1.0
                    text_marker.pose.position.x = med_x
                    text_marker.pose.position.y = med_y
                    text_marker.pose.position.z = med_z + 0.02  # Offset above the ellipsoid
                    text_marker.text = tag_name
                    
                    self.markers.append(marker)
                    self.markers.append(text_marker)
            
            self.get_logger().info(f"Loaded {len(self.markers)//2} markers from file.")
        except Exception as e:
            self.get_logger().error(f"Error loading markers: {e}")
    
    def publish_markers(self):
        for marker in self.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

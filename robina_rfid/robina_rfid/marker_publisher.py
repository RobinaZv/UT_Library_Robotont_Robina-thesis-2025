#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'marker', 10)  

        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.type = Marker.SPHERE
        self.marker.id = 0
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.position.x = 1.0935969948768616
        self.marker.pose.position.y = -0.38438349962234497
        self.marker.pose.position.z = 0.0
        self.get_logger().info("Publishing marker: tag1")
        
        self.timer = self.create_timer(0.5, self.publish_marker)
    
    def publish_marker(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()

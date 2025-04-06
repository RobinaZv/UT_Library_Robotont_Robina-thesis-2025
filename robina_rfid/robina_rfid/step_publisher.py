import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class StepPublisherNode(Node):
   def __init__(self):
       super().__init__('step_publisher_node')
       self.publisher_ = self.create_publisher(Int32, '/steps', 10)


       # Hardcoded step value (100 steps)
       self.steps = 200


   def publish_steps(self):
       msg = Int32()
       msg.data = self.steps
       self.publisher_.publish(msg)
       self.get_logger().info(f'Published steps: {msg.data}')


def main(args=None):
   rclpy.init(args=args)
   node = StepPublisherNode()


   # Publishing steps in a loop
   while rclpy.ok():
       node.publish_steps()
       rclpy.spin_once(node)
       node.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))


   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial


class HeightChangerNode(Node):
   def __init__(self):
       super().__init__('height_changer_node')
       self.subscription = self.create_subscription(
           Int32,
           '/steps',
           self.steps_callback,
           10
       )


       # Adjust your serial port if necessary
       self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)


   def steps_callback(self, msg):
       steps = msg.data
       self.get_logger().info(f'Sending steps to Arduino: {steps}')
       self.send_steps_to_arduino(steps)


   def send_steps_to_arduino(self, steps):
       # Send the step count followed by newline
       self.serial_port.write(f"{steps}\n".encode('utf-8'))


def main(args=None):
   rclpy.init(args=args)
   node = HeightChangerNode()


   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()


if __name__ == '__main__':
   main()

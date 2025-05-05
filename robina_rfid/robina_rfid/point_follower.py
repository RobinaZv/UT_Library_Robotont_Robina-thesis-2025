import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32, Float32
import time
import threading


class PointFollower(Node):
    def __init__(self):
        super().__init__('point_follower')

        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.steps_publisher = self.create_publisher(Int32, '/steps', 10)
        self.z_height_publisher = self.create_publisher(Float32, '/z_height', 10)

        self.points = []
        self.navigation_thread = None

        self.publish_z_height(1.4)
        self.get_logger().info('Initial z_height of 1.4 published')
        self.get_logger().info('Point Follower Node Initialized')

    def point_callback(self, msg):
        point = msg.point
        self.points.append((point.x, point.y))
        self.get_logger().info(f'Point received: ({point.x}, {point.y})')

        if len(self.points) == 4 and self.navigation_thread is None:
            self.navigation_thread = threading.Thread(target=self.navigate_loop)
            self.navigation_thread.start()

    def navigate_loop(self):
        # Go from point 0 → 1 → 2 → 3
        for i in range(4):
            self.go_and_wait(i, wait_time=5)

        # At point 4 (index 3), do long wait, spin stepper, lower z
        self.get_logger().info("At 4th point: initiating 140 second wait with stepper activity...")
        self.publish_steps(-50000)
        self.wait_with_log(110)
        self.publish_z_height(1.0)

        # Go back: 2 → 1 → 0
        for i in reversed(range(3)):
            self.go_and_wait(i, wait_time=5)

        self.get_logger().info("Returned to point 1. Cycle complete.")

    def go_and_wait(self, index, wait_time):
        x, y = self.points[index]
        self.send_goal(x, y)
        self.wait_with_log(wait_time)

    def send_goal(self, x, y):
        self.get_logger().info(f'Sending goal to ({x}, {y})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('Goal reached.')

    def publish_steps(self, step_count):
        msg = Int32()
        msg.data = step_count
        self.steps_publisher.publish(msg)
        self.get_logger().info(f"Published steps: {step_count}")

    def publish_z_height(self, height):
        msg = Float32()
        msg.data = height
        self.z_height_publisher.publish(msg)
        self.get_logger().info(f"Published z_height: {height}")

    def wait_with_log(self, seconds):
        self.get_logger().info(f"Waiting {seconds} seconds...")
        for i in range(seconds, 0, -1):
            self.get_logger().info(f'{i}...')
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = PointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

        # Immediately publish z_height = 1.5 on startup
        self.publish_z_height(1.5)
        self.get_logger().info('Initial z_height of 1.5 published')
        self.get_logger().info('Point Follower Node Initialized')

    def point_callback(self, msg):
        point = msg.point
        self.points.append((point.x, point.y))
        self.get_logger().info(f'Point received: ({point.x}, {point.y})')

        if len(self.points) == 2 and self.navigation_thread is None:
            self.navigation_thread = threading.Thread(target=self.navigate_loop)
            self.navigation_thread.start()

    def navigate_loop(self):
        navigation_order = [0, 1, 0, 1]  # Visit: first, second, first, second again

        for step_idx, point_idx in enumerate(navigation_order):
            x, y = self.points[point_idx]
            self.send_goal(x, y)

            if step_idx == 0:  # First visit to point 1
                self.publish_steps(-400)
                self.wait_with_log(5)
                self.publish_z_height(1.5)

            elif step_idx == 1:  # First visit to point 2
                self.publish_steps(-60000)
                self.wait_with_log(100)
                self.publish_z_height(1.0)

            elif step_idx == 2:  # Second visit to point 1
                self.publish_steps(-60000)
                self.wait_with_log(100)
                self.publish_z_height(0.5)

            elif step_idx == 3:  # Final visit to point 2
                self.get_logger().info("Final point reached. Cycle complete.")

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

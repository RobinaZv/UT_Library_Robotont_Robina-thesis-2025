import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
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
        self.points = []
        self.current_index = 0
        self.navigation_thread = None
        self.get_logger().info('Point Follower Node Initialized')

    def point_callback(self, msg):
        point = msg.point
        self.points.append((point.x, point.y))
        self.get_logger().info(f'Point received: ({point.x}, {point.y})')

        if len(self.points) == 2 and self.navigation_thread is None:
            self.navigation_thread = threading.Thread(target=self.navigate_loop)
            self.navigation_thread.start()

    def navigate_loop(self):
        while rclpy.ok():
            for idx in [0, 1, 0]:  # Go to point 1, then 2, then back to 1
                x, y = self.points[idx]
                self.send_goal(x, y)

                self.get_logger().info(f"Waiting 10 seconds at point {idx + 1}")
                for i in range(10, 0, -1):
                    self.get_logger().info(f'{i}...')
                    time.sleep(1)

            self.get_logger().info("Navigation cycle complete.")
            break  # End loop after one full cycle

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


def main(args=None):
    rclpy.init(args=args)
    node = PointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

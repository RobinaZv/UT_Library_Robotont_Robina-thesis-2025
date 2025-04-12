import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32, Float32  # Added Float32 for z_height
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
        self.steps_publisher = self.create_publisher(Int32, '/steps', 10)  # ✅ Int32
        self.z_height_publisher = self.create_publisher(Float32, '/z_height', 10)  # Publisher for z_height

        self.points = []
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
        for idx in [0, 1, 0]:  # Go to point 1, then 2, then back to 1
            x, y = self.points[idx]
            self.send_goal(x, y)

            # ✅ Publish -20000 to /steps as Int32
            msg = Int32()
            msg.data = -4000
            self.steps_publisher.publish(msg)
            self.get_logger().info(f"Published steps: {msg.data}")

            # ✅ Countdown 10 seconds
            self.get_logger().info(f"Waiting 10 seconds at point {idx + 1}")
            for i in range(60, 0, -1):
                self.get_logger().info(f'{i}...')
                time.sleep(1)

            # Publish z_height when the robot reaches the first or second point
            if idx == 0:  # Reached first point
                self.publish_z_height(1.5)
            elif idx == 1:  # Reached second point
                self.publish_z_height(1.0)

        self.get_logger().info("Navigation cycle complete.")

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

    def publish_z_height(self, height):
        """ Publish the z height to the /z_height topic. """
        msg = Float32()
        msg.data = height
        self.z_height_publisher.publish(msg)
        self.get_logger().info(f"Published z_height: {height}")


def main(args=None):
    rclpy.init(args=args)
    node = PointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

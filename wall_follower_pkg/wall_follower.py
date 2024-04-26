import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.return_subscription = self.create_subscription(Bool, '/return_to_start', self.return_callback, 10)
        self.start_subscription = self.create_subscription(Bool, '/start', self.start_callback, 10)
        self.twist = Twist()
        self.min_distance = 0.5  # Minimum distance to maintain from the wall
        self.max_distance = 1.5  # Maximum distance to maintain from the wall
        self.start_pose = None
        self.return_to_start = False
        self.start_robot = False

    def scan_callback(self, msg):
        if not self.start_robot:
            self.get_logger().info("Waiting for start command...")
            return

        # Get the range values from the laser scan
        ranges = msg.ranges

        # Get the distances from different directions
        forward_distance = ranges[0]
        left_forward_distance = ranges[45]
        right_forward_distance = ranges[315]
        left_distance = ranges[90]
        right_distance = ranges[270]
        left_rear_distance = ranges[135]
        right_rear_distance = ranges[225]

        self.get_logger().info(f"Distances:")
        self.get_logger().info(f"  Forward: {forward_distance:.2f}")
        self.get_logger().info(f"  Left Forward: {left_forward_distance:.2f}")
        self.get_logger().info(f"  Right Forward: {right_forward_distance:.2f}")
        self.get_logger().info(f"  Left: {left_distance:.2f}")
        self.get_logger().info(f"  Right: {right_distance:.2f}")
        self.get_logger().info(f"  Left Rear: {left_rear_distance:.2f}")
        self.get_logger().info(f"  Right Rear: {right_rear_distance:.2f}")

        # Set the linear and angular velocities based on the distances
        if forward_distance > self.min_distance:
            self.twist.linear.x = 0.2  # Move forward
            if left_distance < self.min_distance:
                self.twist.angular.z = -0.5  # Turn right to maintain distance from left wall
                self.get_logger().info("Too close to the left wall, turning right")
            elif left_distance > self.max_distance:
                self.twist.angular.z = 0.5  # Turn left to maintain distance from left wall
                self.get_logger().info("Too far from the left wall, turning left")
            else:
                self.twist.angular.z = 0.0  # Go straight
                self.get_logger().info("Moving forward, maintaining distance from left wall")
        else:
            self.twist.linear.x = 0.0  # Stop
            if left_rear_distance > right_rear_distance:
                self.twist.angular.z = 0.5  # Turn left
                self.get_logger().info("Obstacle detected, turning left")
            else:
                self.twist.angular.z = -0.5  # Turn right
                self.get_logger().info("Obstacle detected, turning right")

        # Publish the velocity commands
        self.publisher.publish(self.twist)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        if self.start_pose is None:
            self.start_pose = self.current_pose
            self.get_logger().info("Start position set")

    def start_callback(self, msg):
        self.start_robot = msg.data
        if self.start_robot:
            self.get_logger().info("Received start command, robot starts moving")

    def send_goal(self):
        self.get_logger().info('Sending Goal')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = self.start_pose.position.x
        goal_pose.pose.pose.position.y = self.start_pose.position.y
        goal_pose.pose.pose.position.z = 0.0
        goal_pose.pose.pose.orientation.x = 0.0
        goal_pose.pose.pose.orientation.y = 0.0
        goal_pose.pose.pose.orientation.z = 0.0
        goal_pose.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info('Navigation failed with status: {0}'.format(status))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def return_callback(self, msg):
        self.return_to_start = msg.data
        if self.return_to_start:
            self.get_logger().info("Received command to return to start position")
            if self.start_pose is None:
                self.get_logger().warning("Start pose is not set. Cannot return to start position.")
                return
            self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

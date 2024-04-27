import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")
        self.set_parameters()
        self.create_subscriptions()
        self.create_publishers()
        self.log_message("Wall Follower Node Initialized")

    def set_parameters(self):
        self.max_wall_dist = 0.55  # Desired wall distance threshold
        self.curr_state = 0
        self.wall_found = False
        self.start_received = False

    def create_subscriptions(self):
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.start_subscription = self.create_subscription(
            Bool, "/start", self.start_callback, 10
        )

    def create_publishers(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def scan_callback(self, msg):
        regions = self.process_laser_scan(msg)
        self.take_action(regions)

    def start_callback(self, msg):
        if msg.data:
            self.log_message("Starting wall following")
            self.start_received = True
        else:
            self.log_message("Stopping wall following")
            self.start_received = False

    def process_laser_scan(self, msg):
        length = len(msg.ranges)
        each_div = length // 8
        front_start = length - each_div // 2
        front_end = each_div // 2

        regions = {
            "front": min(min(msg.ranges[front_start:]), min(msg.ranges[:front_end])),
            "left": min(msg.ranges[each_div : 2 * each_div]),
            "fleft": min(msg.ranges[each_div // 2 : each_div]),
            "fright": min(msg.ranges[front_start - each_div // 2 : front_start]),
            "right": min(msg.ranges[length - 2 * each_div : length - each_div]),
        }
        return regions

    def take_action(self, regions):
        if not self.wall_found:
            if (
                regions["front"] > self.max_wall_dist
                and regions["right"] > self.max_wall_dist
                and regions["left"] > self.max_wall_dist
            ):
                self.curr_state = 0  # Find wall
            elif regions["front"] < self.max_wall_dist:
                self.curr_state = 1  # Wall found, turn left
                self.log_message("wall found, turn left")
                self.wall_found = True
        else:
            if (
                regions["right"] > self.max_wall_dist
                and regions["front"] > self.max_wall_dist
            ):
                self.curr_state = 3  # Move diagonally right
                self.log_message("Move diagonally right")
            elif (
                regions["right"] > self.max_wall_dist
                and regions["fright"] > self.max_wall_dist
                and regions["front"] > self.max_wall_dist
            ):
                self.curr_state = 3  # Turn right
                self.log_message("Turn right")
            elif regions["front"] > self.max_wall_dist:
                self.curr_state = 2  # Go straight
                self.log_message("Go straight")
            elif regions["front"] < self.max_wall_dist:
                self.curr_state = 1  # Turn left
                self.log_message("Turn left")
            elif (
                regions["front"] < self.max_wall_dist
                and regions["fleft"] < self.max_wall_dist
                and regions["fright"] < self.max_wall_dist
            ):
                self.curr_state = 1  # Dead end, turn left
                self.log_message("Deade end, turn left")

    def run(self):
        self.log_message("Wall Follower Started")
        while rclpy.ok():
            if self.start_received:
                vel_cmd = self.get_velocity_command()
                self.cmd_vel_publisher.publish(vel_cmd)
            else:
                self.log_message("Waiting for start message...")

            rclpy.spin_once(self, timeout_sec=0.1)

    def get_velocity_command(self):
        vel_cmd = Twist()
        if self.curr_state == 0:  # Find wall
            vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = 0.0
        elif self.curr_state == 1:  # Turn left
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = 0.45
        elif self.curr_state == 2:  # Go straight
            vel_cmd.linear.x = 0.12
            vel_cmd.angular.z = -0.02
        elif self.curr_state == 3:  # Turn right or move diagonally right
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = -0.45
        else:
            self.log_message("Unknown state", "warn")
        return vel_cmd

    def log_message(self, message, level="info"):
        if level == "info":
            self.get_logger().info(message)
        elif level == "warn":
            self.get_logger().warn(message)
        elif level == "error":
            self.get_logger().error(message)

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    wall_follower.run()
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

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
        self.create_subscribers()
        self.create_publishers()
        self.log_message("Wall Follower Node Initialized")

    def set_parameters(self):
        self.threshold = 0.65  # wall distance threshold
        self.curr_state = 0  #start with find a wall
        self.wall_found = False  #inital with wall not find
        self.start_received = False  # wait to start
    #create subscribers
    def create_subscribers(self):
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.start_subscription = self.create_subscription(
            Bool, "/start", self.start_callback, 10
        )
    #create publisher
    def create_publishers(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def scan_callback(self, msg):
        sections = self.divide_sections(msg)
        self.make_move(sections)

    def start_callback(self, msg):
        if msg.data:
            self.log_message("Starting wall following")
            self.start_received = True
        else:
            self.log_message("Stopping wall following")
            self.start_received = False
    #divide laser read into 5 sections
    def divide_sections(self, msg):
        length = len(msg.ranges)
        each_div = length // 8
        front_start = length - each_div // 2
        front_end = each_div // 2

        sections = {
            "front": min(min(msg.ranges[front_start:]), min(msg.ranges[:front_end])),
            "left": min(msg.ranges[each_div : 2 * each_div]),
            "fleft": min(msg.ranges[each_div // 2 : each_div]),
            "fright": min(msg.ranges[front_start - each_div // 2 : front_start]),
            "right": min(msg.ranges[length - 2 * each_div : length - each_div]),
        }
        return sections

    def make_move(self, sections):
        if not self.wall_found:
            if (
                sections["front"] > self.threshold
                and sections["right"] > self.threshold
                and sections["left"] > self.threshold
            ):
                self.curr_state = 0  # Find wall
            elif sections["front"] < self.threshold:
                self.curr_state = 1  # found wall, turn left
                self.log_message("found wall, turn left")
                self.wall_found = True
        else:
            if (
                sections["right"] > self.threshold
                and sections["front"] > self.threshold
            ):
                self.curr_state = 3  # make right U turn
                self.log_message("make right U turn")
            elif (
                sections["right"] > self.threshold
                and sections["fright"] > self.threshold
                and sections["front"] > self.threshold
            ):
                self.curr_state = 3  # turn right
                self.log_message("turn right")
            elif sections["front"] > self.threshold:
                self.curr_state = 2  # move forward
                self.log_message("move forward")
            elif sections["front"] < self.threshold:
                self.curr_state = 1  # turn left
                self.log_message("turn left")
            elif (
                sections["front"] < self.threshold
                and sections["fleft"] < self.threshold
                and sections["fright"] < self.threshold
            ):
                self.curr_state = 1  # Dead end, turn left
                self.log_message("deade end, turn left")

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
        if self.curr_state == 0:  # find a wall
            vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = 0.0
        elif self.curr_state == 1:  # turn left
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.25
        elif self.curr_state == 2:  # move forward
            vel_cmd.linear.x = 0.07
            vel_cmd.angular.z = 0.0
        elif self.curr_state == 3:  # right U turn
            vel_cmd.linear.x = 0.08
            vel_cmd.angular.z = -0.25
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

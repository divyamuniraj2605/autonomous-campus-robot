#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_robot
        )

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

    def scan_callback(self, msg):

        total = len(msg.ranges)

        if total == 0:
            return

        # Front: around 0 degrees
        front_ranges = list(msg.ranges[0:20]) + list(msg.ranges[-20:])

        # Left: around +90 degrees
        left_ranges = list(msg.ranges[70:110])

        # Right: around -90 degrees
        right_ranges = list(msg.ranges[250:290])

        self.front_distance = self.get_min_distance(front_ranges)
        self.left_distance = self.get_min_distance(left_ranges)
        self.right_distance = self.get_min_distance(right_ranges)

    def get_min_distance(self, ranges):

        valid_ranges = [
            r for r in ranges
            if r > 0.12 and r < 12.0
        ]

        if not valid_ranges:
            return float('inf')

        return min(valid_ranges)

    def control_robot(self):

        cmd = Twist()

        obstacle_distance = 0.8

        if self.front_distance < obstacle_distance:

            # Stop forward motion and turn toward
            # the side with more free space.

            cmd.linear.x = 0.0

            if self.left_distance > self.right_distance:
                cmd.angular.z = 0.5
            else:
                cmd.angular.z = -0.5

            self.get_logger().info(
                'Obstacle detected - turning'
            )

        else:

            # No obstacle in front
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

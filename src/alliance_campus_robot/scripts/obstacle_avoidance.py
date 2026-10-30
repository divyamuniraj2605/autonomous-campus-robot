#!/usr/bin/env python3

import math
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

        # Safety settings
        self.obstacle_distance = 1.5

        # LiDAR distances
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.get_logger().info(
            'Obstacle avoidance node started'
        )

    def scan_callback(self, msg):

        front = []
        left = []
        right = []

        for i, distance in enumerate(msg.ranges):

            if not math.isfinite(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)

            # Wider front detection
            if -45 <= angle_deg <= 45:
                front.append(distance)

            # Left side
            elif 45 < angle_deg <= 100:
                left.append(distance)

            # Right side
            elif -100 <= angle_deg < -45:
                right.append(distance)

        if front:
            self.front_distance = min(front)
        else:
            self.front_distance = float('inf')

        if left:
            self.left_distance = min(left)
        else:
            self.left_distance = float('inf')

        if right:
            self.right_distance = min(right)
        else:
            self.right_distance = float('inf')

    def control_robot(self):

        cmd = Twist()

        self.get_logger().info(
            f'Front: {self.front_distance:.2f} m | '
            f'Left: {self.left_distance:.2f} m | '
            f'Right: {self.right_distance:.2f} m'
        )

        # Obstacle detected in front
        if self.front_distance < self.obstacle_distance:

            # Stop forward movement
            cmd.linear.x = 0.0

            # Turn toward the side with more space
            if self.left_distance > self.right_distance:

                cmd.angular.z = 1.0

                self.get_logger().info(
                    'FRONT BLOCKED - TURNING LEFT'
                )

            else:

                cmd.angular.z = -1.0

                self.get_logger().info(
                    'FRONT BLOCKED - TURNING RIGHT'
                )

        # Front is clear
        else:

            # Move slowly
            cmd.linear.x = 0.10
            cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

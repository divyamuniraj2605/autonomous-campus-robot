#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RoadObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.x = None
        self.y = None
        self.yaw = None

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.last_scan_time = 0.0
        self.last_odom_time = 0.0

        # Keep the robot center away from road edges.
        self.margin = 0.8

        # Road rectangles: xmin, xmax, ymin, ymax.
        self.roads = [
            (-4.0 + self.margin, 4.0 - self.margin,
             -34.5 + self.margin, 30.5 - self.margin),

            (-40.0 + self.margin, 40.0 - self.margin,
             -3.5 + self.margin, 3.5 - self.margin),

            (-40.0 + self.margin, 40.0 - self.margin,
             22.0 + self.margin, 28.0 - self.margin),
        ]

        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.cmd_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.timer = self.create_timer(0.1, self.control_robot)

        self.last_status = ''

        self.get_logger().info(
            'Safety-first road controller started. '
            'Forward motion only; obstacles cause a stop.'
        )

    def sector_min(self, msg, low_angle, high_angle):
        values = []

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if low_angle <= angle <= high_angle:
                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                ):
                    values.append(distance)

        return min(values) if values else float('inf')

    def scan_callback(self, msg):
        # LaserScan angles are measured from the sensor's forward axis.
        self.front_distance = self.sector_min(
            msg, math.radians(-25), math.radians(25)
        )

        self.left_distance = self.sector_min(
            msg, math.radians(60), math.radians(120)
        )

        self.right_distance = self.sector_min(
            msg, math.radians(-120), math.radians(-60)
        )

        self.last_scan_time = time.monotonic()

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y

        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(sin_yaw, cos_yaw)
        self.last_odom_time = time.monotonic()

    def is_on_road(self, x, y):
        for xmin, xmax, ymin, ymax in self.roads:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return True
        return False

    def forward_path_is_safe(self, distance=0.8):
        if self.x is None or self.y is None or self.yaw is None:
            return False

        # Check multiple points along the proposed path.
        step = 0.1
        count = int(distance / step)

        for i in range(1, count + 1):
            d = i * step

            test_x = self.x + d * math.cos(self.yaw)
            test_y = self.y + d * math.sin(self.yaw)

            if not self.is_on_road(test_x, test_y):
                return False

        return True

    def publish_stop(self):
        self.cmd_publisher.publish(Twist())

    def report_stop(self, reason):
        if self.last_status != reason:
            self.get_logger().warn(reason)
            self.last_status = reason

    def control_robot(self):
        now = time.monotonic()

        # Require fresh odometry.
        if (
            self.last_odom_time == 0.0
            or now - self.last_odom_time > 1.0
            or self.x is None
            or self.y is None
            or self.yaw is None
        ):
            self.publish_stop()
            self.report_stop('Stopping: odometry unavailable or stale.')
            return

        # Require fresh LiDAR data.
        if (
            self.last_scan_time == 0.0
            or now - self.last_scan_time > 1.0
        ):
            self.publish_stop()
            self.report_stop('Stopping: LiDAR data unavailable or stale.')
            return

        # Stop if the robot center is outside the safe road rectangles.
        if not self.is_on_road(self.x, self.y):
            self.publish_stop()
            self.report_stop(
                'Stopping: robot is outside the permitted road area.'
            )
            return

        # Stop if the proposed forward path leaves the permitted road.
        if not self.forward_path_is_safe(0.8):
            self.publish_stop()
            self.report_stop(
                'Stopping: road boundary ahead. No automatic turn attempted.'
            )
            return

        # Stop rather than turning blindly around an obstacle.
        if self.front_distance < 0.8:
            self.publish_stop()
            self.report_stop(
                'Stopping: obstacle detected ahead. Waiting for a safe route.'
            )
            return

        # Conservative test speed.
        cmd = Twist()
        cmd.linear.x = 0.10
        cmd.angular.z = 0.0
        self.cmd_publisher.publish(cmd)

        self.last_status = 'moving'


def main(args=None):
    rclpy.init(args=args)
    node = RoadObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop commands before exiting.
        for _ in range(3):
            node.publish_stop()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

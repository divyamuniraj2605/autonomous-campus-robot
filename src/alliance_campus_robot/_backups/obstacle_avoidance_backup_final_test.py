#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # LiDAR
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Robot odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Single command publisher
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_robot
        )

        # LiDAR distances
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # Robot pose
        self.x = None
        self.y = None
        self.yaw = None

        # Road safety margin.
        # The robot must stay this far inside the road boundary.
        self.road_margin = 0.8

        # Road dimensions from the campus world.
        #
        # Main vertical road:
        #   x = -4 to +4
        #   y = -34.5 to +30.5
        #
        # Central horizontal road:
        #   x = -40 to +40
        #   y = -3.5 to +3.5
        #
        # North horizontal road:
        #   x = -40 to +40
        #   y = 22 to 28

        self.main_road = (
            -4.0 + self.road_margin,
             4.0 - self.road_margin,
            -34.5 + self.road_margin,
            30.5 - self.road_margin
        )

        self.central_road = (
            -40.0 + self.road_margin,
             40.0 - self.road_margin,
            -3.5 + self.road_margin,
             3.5 - self.road_margin
        )

        self.north_road = (
            -40.0 + self.road_margin,
             40.0 - self.road_margin,
             22.0 + self.road_margin,
             28.0 - self.road_margin
        )

        self.last_road_warning = False

        self.get_logger().info(
            'Road-only obstacle avoidance started'
        )

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

    def odom_callback(self, msg):

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.x = position.x
        self.y = position.y

        # Quaternion -> yaw
        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )

        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def get_min_distance(self, ranges):

        valid_ranges = [
            r for r in ranges
            if math.isfinite(r) and r > 0.12 and r < 12.0
        ]

        if not valid_ranges:
            return float('inf')

        return min(valid_ranges)

    def inside_rectangle(self, x, y, rectangle):

        xmin, xmax, ymin, ymax = rectangle

        return (
            xmin <= x <= xmax
            and
            ymin <= y <= ymax
        )

    def is_on_road(self, x, y):

        return (
            self.inside_rectangle(x, y, self.main_road)
            or
            self.inside_rectangle(x, y, self.central_road)
            or
            self.inside_rectangle(x, y, self.north_road)
        )

    def forward_position_is_safe(self):

        if self.x is None or self.y is None or self.yaw is None:
            return False

        # Look ahead before allowing forward motion.
        look_ahead = 0.6

        future_x = self.x + look_ahead * math.cos(self.yaw)
        future_y = self.y + look_ahead * math.sin(self.yaw)

        return self.is_on_road(future_x, future_y)

    def turn_toward_road(self):

        if self.x is None or self.y is None:
            return 0.0

        # Find the closest safe road direction.
        candidates = []

        # Main road center line
        main_x = max(-3.2, min(3.2, self.x))
        candidates.append((main_x, self.y))

        # Central road center line
        central_y = max(-2.7, min(2.7, self.y))
        candidates.append((self.x, central_y))

        # North road center line
        north_y = max(22.8, min(27.2, self.y))
        candidates.append((self.x, north_y))

        # Choose the closest candidate.
        target_x, target_y = min(
            candidates,
            key=lambda p: math.hypot(p[0] - self.x, p[1] - self.y)
        )

        target_angle = math.atan2(
            target_y - self.y,
            target_x - self.x
        )

        angle_error = target_angle - self.yaw

        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi

        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        if angle_error > 0:
            return 0.5

        return -0.5

    def control_robot(self):

        cmd = Twist()

        # We cannot make a safe road decision without odometry.
        if self.x is None or self.y is None or self.yaw is None:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_publisher.publish(cmd)
            return

        # ---------------------------------------------------------
        # ROAD SAFETY
        # ---------------------------------------------------------

        if not self.is_on_road(self.x, self.y):

            # Robot is already outside the safe road area.
            # Do NOT move forward.
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_toward_road()

            if not self.last_road_warning:
                self.get_logger().warn(
                    'Robot outside safe road area - stopping and turning toward road'
                )
                self.last_road_warning = True

            self.cmd_publisher.publish(cmd)
            return

        self.last_road_warning = False

        # If moving forward would cross the road boundary,
        # stop forward movement.
        if not self.forward_position_is_safe():

            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_toward_road()

            self.get_logger().warn(
                'Road boundary ahead - preventing movement onto grass'
            )

            self.cmd_publisher.publish(cmd)
            return

        # ---------------------------------------------------------
        # OBSTACLE AVOIDANCE
        # ---------------------------------------------------------

        obstacle_distance = 0.8

        if self.front_distance < obstacle_distance:

            # Stop forward motion and turn toward
            # the side with more free space.

            cmd.linear.x = 0.0

            if self.left_distance > self.right_distance:
                cmd.angular.z = 0.5
            else:
                cmd.angular.z = -0.5

        else:

            # Road is safe and no obstacle is directly ahead.
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

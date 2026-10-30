#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RoadObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
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

        self.x = None
        self.y = None
        self.yaw = None

        # Safety margin from the black-road edge.
        self.margin = 0.8

        # Campus road geometry.
        #
        # Main vertical road:
        # x = -4 ... +4
        # y = -34.5 ... +30.5
        #
        # Central horizontal road:
        # x = -40 ... +40
        # y = -3.5 ... +3.5
        #
        # North horizontal road:
        # x = -40 ... +40
        # y = 22 ... +28

        self.main_road = (
            -4.0 + self.margin,
             4.0 - self.margin,
            -34.5 + self.margin,
            30.5 - self.margin
        )

        self.central_road = (
            -40.0 + self.margin,
             40.0 - self.margin,
            -3.5 + self.margin,
             3.5 - self.margin
        )

        self.north_road = (
            -40.0 + self.margin,
             40.0 - self.margin,
             22.0 + self.margin,
             28.0 - self.margin
        )

        self.last_message = ""

        self.get_logger().info(
            'Campus road-only obstacle avoidance started'
        )

    # ------------------------------------------------------------
    # LiDAR
    # ------------------------------------------------------------

    def scan_callback(self, msg):

        if not msg.ranges:
            return

        front_ranges = (
            list(msg.ranges[0:20])
            + list(msg.ranges[-20:])
        )

        left_ranges = list(msg.ranges[70:110])

        right_ranges = list(msg.ranges[250:290])

        self.front_distance = self.get_min_distance(front_ranges)
        self.left_distance = self.get_min_distance(left_ranges)
        self.right_distance = self.get_min_distance(right_ranges)

    def get_min_distance(self, ranges):

        valid = [
            r for r in ranges
            if math.isfinite(r)
            and r > 0.12
            and r < 12.0
        ]

        if not valid:
            return float('inf')

        return min(valid)

    # ------------------------------------------------------------
    # ODOMETRY
    # ------------------------------------------------------------

    def odom_callback(self, msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y

        siny = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.yaw = math.atan2(siny, cosy)

    # ------------------------------------------------------------
    # ROAD CHECKING
    # ------------------------------------------------------------

    def inside(self, x, y, road):

        xmin, xmax, ymin, ymax = road

        return (
            xmin <= x <= xmax
            and
            ymin <= y <= ymax
        )

    def is_on_road(self, x, y):

        return (
            self.inside(x, y, self.main_road)
            or
            self.inside(x, y, self.central_road)
            or
            self.inside(x, y, self.north_road)
        )

    def point_ahead_is_safe(self, distance):

        if self.x is None or self.y is None or self.yaw is None:
            return False

        test_x = self.x + distance * math.cos(self.yaw)
        test_y = self.y + distance * math.sin(self.yaw)

        return self.is_on_road(test_x, test_y)

    # ------------------------------------------------------------
    # ROAD-SAFE TURNING
    # ------------------------------------------------------------

    def find_safe_turn(self):

        if self.x is None or self.y is None or self.yaw is None:
            return 0.0

        # Test possible directions.
        # We prefer small turns first.
        candidates = [
            (math.radians(45), 0.5),
            (math.radians(-45), -0.5),
            (math.radians(90), 0.5),
            (math.radians(-90), -0.5),
            (math.radians(135), 0.5),
            (math.radians(-135), -0.5),
            (math.pi, 0.5),
        ]

        for relative_angle, angular_speed in candidates:

            test_angle = self.yaw + relative_angle

            test_x = self.x + 1.0 * math.cos(test_angle)
            test_y = self.y + 1.0 * math.sin(test_angle)

            if self.is_on_road(test_x, test_y):
                return angular_speed

        # No safe direction found.
        return 0.0

    # ------------------------------------------------------------
    # MAIN CONTROLLER
    # ------------------------------------------------------------

    def control_robot(self):

        cmd = Twist()

        # Wait for odometry.
        if (
            self.x is None
            or self.y is None
            or self.yaw is None
        ):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_publisher.publish(cmd)
            return

        # --------------------------------------------------------
        # SAFETY 1:
        # Robot must remain on a road.
        # --------------------------------------------------------

        if not self.is_on_road(self.x, self.y):

            cmd.linear.x = 0.0
            cmd.angular.z = self.find_safe_turn()

            if self.last_message != "grass":
                self.get_logger().warn(
                    'GREEN FIELD DETECTED - stopping robot'
                )
                self.last_message = "grass"

            self.cmd_publisher.publish(cmd)
            return

        # --------------------------------------------------------
        # SAFETY 2:
        # Do not allow the robot to drive across the road boundary.
        # --------------------------------------------------------

        if not self.point_ahead_is_safe(0.8):

            cmd.linear.x = 0.0
            cmd.angular.z = self.find_safe_turn()

            if self.last_message != "boundary":
                self.get_logger().warn(
                    'ROAD EDGE AHEAD - preventing movement onto grass'
                )
                self.last_message = "boundary"

            self.cmd_publisher.publish(cmd)
            return

        # --------------------------------------------------------
        # SAFETY 3:
        # LiDAR obstacle avoidance.
        # --------------------------------------------------------

        obstacle_distance = 0.8

        if self.front_distance < obstacle_distance:

            cmd.linear.x = 0.0

            if self.left_distance > self.right_distance:
                cmd.angular.z = 0.5
            else:
                cmd.angular.z = -0.5

            if self.last_message != "obstacle":
                self.get_logger().info(
                    'OBSTACLE DETECTED - turning'
                )
                self.last_message = "obstacle"

        else:

            # Road is safe and no obstacle is ahead.
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

            self.last_message = "moving"

        self.cmd_publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = RoadObstacleAvoidance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

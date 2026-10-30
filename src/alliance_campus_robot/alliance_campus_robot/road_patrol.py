#!/usr/bin/env python3
"""Automatic mapping: drive slowly along the campus road centrelines while
SLAM Toolbox builds the map.  Stop-turn-drive motion (no turning while moving)
keeps skid-steer slip low, and the robot stops for obstacles seen by the LiDAR.

Navigation uses Gazebo's exact robot pose (/ground_truth/odom, simulation
only) so wheel slip cannot throw the robot off the road. SLAM still builds the
map from the LiDAR.  Waypoints are WORLD coordinates.
"""
import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Road centrelines: main road x=0, central road y=0, north road y=25.
WAYPOINTS = [
    (0.0, -30.0),
    (0.0, 0.0),
    (-35.0, 0.0), (0.0, 0.0), (35.0, 0.0), (0.0, 0.0),
    (0.0, 26.0),
    (-35.0, 26.0), (0.0, 26.0), (35.0, 26.0), (0.0, 26.0),
    (0.0, 0.0),
    (0.0, -30.0),
]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(v, lim):
    return max(-lim, min(lim, v))


class RoadPatrol(Node):
    def __init__(self):
        super().__init__('road_patrol')
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.4)
        self.declare_parameter('tolerance', 0.6)
        self.declare_parameter('obstacle_distance', 0.9)
        self.declare_parameter('save_map_path', os.path.expanduser('~/campus_map'))

        self.v_max = self.get_parameter('linear_speed').value
        self.w_max = self.get_parameter('angular_speed').value
        self.tol = self.get_parameter('tolerance').value
        self.obs = self.get_parameter('obstacle_distance').value
        self.save_path = self.get_parameter('save_map_path').value

        self.targets = list(WAYPOINTS)  # world coordinates
        self.idx = 1  # waypoint 0 is the spawn point

        self.odom_pose = None
        self.front = float('inf')
        self.last_scan = 0.0
        self.blocked_since = None
        self.finished = False
        self.warned = ''

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.loop)
        self.get_logger().info(
            f'Road patrol started: {len(self.targets) - 1} waypoints, '
            f'{self.v_max} m/s. Waiting for /ground_truth/odom...')

    # ---------------- sensors ----------------
    def on_scan(self, msg):
        vals = []
        for i, d in enumerate(msg.ranges):
            a = wrap(msg.angle_min + i * msg.angle_increment)
            if abs(a) <= math.radians(25) and math.isfinite(d) \
                    and msg.range_min <= d <= msg.range_max:
                vals.append(d)
        self.front = min(vals) if vals else float('inf')
        self.last_scan = time.monotonic()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (p.x, p.y, yaw)

    def get_pose(self):
        return self.odom_pose  # exact world pose from Gazebo

    # ---------------- control ----------------
    def stop(self):
        self.cmd_pub.publish(Twist())

    def warn_once(self, text):
        if text != self.warned:
            self.get_logger().warn(text)
            self.warned = text

    def loop(self):
        if self.finished:
            self.stop()
            return
        pose = self.get_pose()
        if pose is None or time.monotonic() - self.last_scan > 1.0:
            self.stop()
            self.warn_once('Waiting for pose / LiDAR data...')
            return

        x, y, yaw = pose
        tx, ty = self.targets[self.idx]
        dist = math.hypot(tx - x, ty - y)

        if dist < self.tol:
            self.get_logger().info(
                f'Reached waypoint {self.idx}/{len(self.targets) - 1}')
            self.idx += 1
            self.stop()
            if self.idx >= len(self.targets):
                self.finish()
            return

        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        cmd = Twist()
        if abs(err) > 0.25:
            # turn in place, slowly
            w = clip(0.8 * err, self.w_max)
            if abs(w) < 0.12:
                w = math.copysign(0.12, err)
            cmd.angular.z = w
        else:
            if self.front < self.obs:
                self.stop()
                if self.blocked_since is None:
                    self.blocked_since = time.monotonic()
                self.warn_once(f'Obstacle ahead ({self.front:.2f} m) at x={x:.1f} y={y:.1f} - waiting.')
                if time.monotonic() - self.blocked_since > 30.0:
                    self.get_logger().error(
                        'Blocked for 30 s. Clear the obstacle or restart.')
                return
            self.blocked_since = None
            self.warned = ''
            cmd.linear.x = min(self.v_max, 0.15 + 0.3 * dist)
            cmd.angular.z = clip(0.8 * err, 0.25)
        self.cmd_pub.publish(cmd)

    def finish(self):
        self.finished = True
        self.stop()
        self.get_logger().info(f'Patrol complete. Saving map to {self.save_path}')
        try:
            subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                              '-f', self.save_path])
        except Exception as e:
            self.get_logger().error(
                f'Auto-save failed ({e}); save manually with map_saver_cli.')


def main(args=None):
    rclpy.init(args=args)
    node = RoadPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

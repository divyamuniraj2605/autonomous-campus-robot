#!/usr/bin/env python3
"""Publishes map->odom from Gazebo's exact pose (simulation only), replacing AMCL.
Do not run together with known_pose_mapper (it publishes the same transform)."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Localizer(Node):
    def __init__(self):
        super().__init__('ground_truth_localizer')
        self.pose = None
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_pose, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Localizer up: map->odom from /ground_truth/odom')

    def on_pose(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def tick(self):
        if self.pose is None:
            return
        try:
            t = self.buf.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception:
            return
        xo, yo = t.transform.translation.x, t.transform.translation.y
        th = self.pose[2] - yaw_of(t.transform.rotation)
        c, s = math.cos(th), math.sin(th)
        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id, out.child_frame_id = 'map', 'odom'
        out.transform.translation.x = self.pose[0] - (c * xo - s * yo)
        out.transform.translation.y = self.pose[1] - (s * xo + c * yo)
        out.transform.rotation.z, out.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self.br.sendTransform(out)


def main(args=None):
    rclpy.init(args=args)
    n = Localizer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

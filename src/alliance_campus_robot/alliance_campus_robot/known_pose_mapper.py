#!/usr/bin/env python3
"""Occupancy-grid mapping with KNOWN robot pose (simulation only).

Uses Gazebo's exact pose (/ground_truth/odom, world frame) to ray-cast each
LiDAR scan into a grid, so wheel slip / SLAM drift cannot smear the map.
Publishes /map and map->odom TF (map == Gazebo world frame) and autosaves
~/campus_map.pgm + ~/campus_map.yaml.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class KnownPoseMapper(Node):
    def __init__(self):
        super().__init__('known_pose_mapper')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('size_m', 100.0)        # map is size x size, centred on world origin
        self.declare_parameter('min_range', 0.30)      # ignore self-hits
        self.declare_parameter('save_path', os.path.expanduser('~/campus_map'))

        self.res = self.get_parameter('resolution').value
        size = self.get_parameter('size_m').value
        self.min_range = self.get_parameter('min_range').value
        self.save_path = self.get_parameter('save_path').value
        self.n = int(size / self.res)
        self.origin = -size / 2.0
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)

        self.pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, '/ground_truth/odom', self.on_pose, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)

        self.create_timer(0.05, self.publish_tf)
        self.create_timer(2.0, self.publish_map)
        self.create_timer(30.0, self.save_map)
        self.get_logger().info(
            f'Known-pose mapper: {self.n}x{self.n} cells @ {self.res} m. '
            'Waiting for /ground_truth/odom and /scan...')

    def on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    # map -> odom so RViz can show the robot model in the map frame
    def publish_tf(self):
        if self.pose is None:
            return
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception:
            return
        xo, yo = t.transform.translation.x, t.transform.translation.y
        tho = yaw_of(t.transform.rotation)
        xb, yb, thb = self.pose
        th = thb - tho
        c, s = math.cos(th), math.sin(th)
        tx = xb - (c * xo - s * yo)
        ty = yb - (s * xo + c * yo)
        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.child_frame_id = 'odom'
        out.transform.translation.x = tx
        out.transform.translation.y = ty
        out.transform.rotation.z = math.sin(th / 2)
        out.transform.rotation.w = math.cos(th / 2)
        self.tf_broadcaster.sendTransform(out)

    def on_scan(self, msg):
        if self.pose is None:
            return
        px, py, pyaw = self.pose
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = pyaw + msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        rmax = msg.range_max
        hit = np.isfinite(ranges) & (ranges >= max(msg.range_min, self.min_range)) \
            & (ranges < rmax - 0.05)
        length = np.where(np.isfinite(ranges), np.minimum(ranges, rmax), rmax)
        valid = np.isfinite(ranges) | True
        cosa, sina = np.cos(angles), np.sin(angles)

        steps = int(rmax / self.res) + 1
        ts = (np.arange(1, steps + 1) * self.res).astype(np.float32)       # (steps,)
        free_mask = ts[None, :] < (length[:, None] - self.res)              # (beams, steps)
        free_mask &= ts[None, :] > self.min_range
        xs = px + ts[None, :] * cosa[:, None]
        ys = py + ts[None, :] * sina[:, None]
        ix = ((xs - self.origin) / self.res).astype(np.int32)
        iy = ((ys - self.origin) / self.res).astype(np.int32)
        inb = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n) & free_mask
        self.logodds[iy[inb], ix[inb]] -= 0.35

        hx = px + length * cosa
        hy = py + length * sina
        hix = ((hx - self.origin) / self.res).astype(np.int32)
        hiy = ((hy - self.origin) / self.res).astype(np.int32)
        ok = hit & (hix >= 0) & (hix < self.n) & (hiy >= 0) & (hiy < self.n)
        self.logodds[hiy[ok], hix[ok]] += 1.2
        np.clip(self.logodds, -5.0, 5.0, out=self.logodds)

    def grid_values(self):
        g = np.full((self.n, self.n), -1, dtype=np.int8)
        g[self.logodds < -0.5] = 0
        g[self.logodds > 0.8] = 100
        return g

    def publish_map(self):
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'map'
        g.info.resolution = self.res
        g.info.width = self.n
        g.info.height = self.n
        g.info.origin.position.x = self.origin
        g.info.origin.position.y = self.origin
        g.info.origin.orientation.w = 1.0
        g.data = self.grid_values().flatten().tolist()
        self.map_pub.publish(g)

    def save_map(self):
        if self.pose is None:
            return
        g = self.grid_values()
        img = np.full(g.shape, 205, dtype=np.uint8)   # unknown
        img[g == 0] = 254                              # free
        img[g == 100] = 0                              # occupied
        img = np.flipud(img)                           # PGM row 0 = top
        with open(self.save_path + '.pgm', 'wb') as f:
            f.write(f'P5\n{self.n} {self.n}\n255\n'.encode())
            f.write(img.tobytes())
        with open(self.save_path + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(self.save_path)}.pgm\n'
                    f'mode: trinary\nresolution: {self.res}\n'
                    f'origin: [{self.origin}, {self.origin}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        self.get_logger().info(f'Map saved to {self.save_path}.pgm/.yaml')


def main(args=None):
    rclpy.init(args=args)
    node = KnownPoseMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_map()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Bridge between the Gazebo GUI overlay and Nav2.

The Gazebo side (libcampus_labels.so) sends the clicked destination, e.g.
"CENTRAL LIBRARY" or "STOP", as a UDP datagram to 127.0.0.1:5005.
This node turns that into a Nav2 NavigateToPose goal on the road-only map.

Also accepts text on the ROS topic /campus/destination (std_msgs/String), so
you can test without the GUI:
    ros2 topic pub --once /campus/destination std_msgs/msg/String "{data: library}"
Progress is published on /campus/status (std_msgs/String).
"""
import os
import socket

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from alliance_campus_robot.campus_nav_utils import load_locations, load_pgm_map, resolve_goal

STOP_WORDS = ('stop', 'cancel', 'halt')


class GuiBridge(Node):
    def __init__(self):
        super().__init__('gui_bridge')
        pkg = get_package_share_directory('alliance_campus_robot')
        self.declare_parameter('port', int(os.environ.get('CAMPUS_GUI_PORT', '5005')))
        self.declare_parameter('map_yaml', os.path.join(pkg, 'maps', 'campus_nav_map.yaml'))

        self.locations = load_locations(os.path.join(pkg, 'config', 'locations.yaml'))
        self.free, self.res, self.ox, self.oy = load_pgm_map(
            self.get_parameter('map_yaml').value)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/campus/status', 10)
        self.create_subscription(String, '/campus/destination', self.on_topic, 10)

        port = self.get_parameter('port').value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', port))
        self.sock.setblocking(False)
        self.create_timer(0.1, self.poll_socket)

        self.goal_handle = None
        self.dest_name = ''
        self.last_fb = 0.0
        self.status(f'GUI bridge ready on UDP {port}. Click a place in the Gazebo panel.')

    # ---------------------------------------------------------------- input
    def poll_socket(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
            except BlockingIOError:
                return
            self.on_command(data.decode('utf-8', errors='ignore').strip())

    def on_topic(self, msg):
        self.on_command(msg.data.strip())

    def on_command(self, text):
        if not text:
            return
        if text.lower() in STOP_WORDS:
            self.cancel('Stopped by user.')
            return
        self.send_goal(text)

    # ----------------------------------------------------------------- nav2
    def send_goal(self, text):
        res = resolve_goal(text, self.locations, self.free, self.res, self.ox, self.oy)
        if res[0] is None:
            self.status(res[1])
            return
        name, x, y, moved = res
        if not self.client.server_is_ready():
            self.status('Nav2 is not running. Launch with mode:=nav first.')
            return
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()      # replace the old goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        self.dest_name = name
        self.status(f'Going to {name} -> ({x:.1f}, {y:.1f})')
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(self.on_accepted)

    def on_accepted(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.status(f'Nav2 rejected the goal to {self.dest_name}.')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, fb):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_fb > 3.0:
            self.last_fb = now
            self.status(f'{self.dest_name}: {fb.feedback.distance_remaining:.1f} m remaining')

    def on_result(self, fut):
        status = fut.result().status
        if status == 4:
            self.status(f'Arrived at {self.dest_name}.')
        elif status == 6:
            self.status(f'Could not reach {self.dest_name} (Nav2 aborted).')
        self.goal_handle = None

    def cancel(self, why):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.status(why)

    def status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = GuiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

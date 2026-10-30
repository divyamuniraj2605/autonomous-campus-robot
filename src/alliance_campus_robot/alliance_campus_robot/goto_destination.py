#!/usr/bin/env python3
"""Type a place name ('library', 'food court', ...) and the robot drives there.
One-shot:  ros2 run alliance_campus_robot goto_destination --ros-args -p destination:="library"
"""
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from alliance_campus_robot.campus_nav_utils import (
    find_location, load_locations, load_pgm_map, snap_to_free)


class Goto(Node):
    def __init__(self):
        super().__init__('goto_destination')
        pkg = get_package_share_directory('alliance_campus_robot')
        self.declare_parameter('destination', '')
        self.declare_parameter('map_yaml', os.path.join(pkg, 'maps', 'campus_nav_map.yaml'))
        self.locations = load_locations(os.path.join(pkg, 'config', 'locations.yaml'))
        self.free, self.res, self.ox, self.oy = load_pgm_map(
            self.get_parameter('map_yaml').value)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.last_print = 0.0

    def feedback(self, fb):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_print > 3.0:
            self.last_print = now
            self.get_logger().info(
                f'  distance remaining: {fb.feedback.distance_remaining:.1f} m')

    def go(self, text):
        found = find_location(text, self.locations)
        if not found:
            print('Location not found. Try: ' +
                  ', '.join(v['display_name'] for v in self.locations.values()))
            return
        key, loc = found
        snapped = snap_to_free(self.free, self.res, self.ox, self.oy, loc['x'], loc['y'])
        if snapped is None:
            print(f'No drivable spot near {loc["display_name"]}.')
            return
        x, y, moved = snapped
        note = f' (moved {moved:.1f} m onto the road)' if moved > 0.3 else ''
        print(f'Going to {loc["display_name"]} -> ({x:.1f}, {y:.1f}){note}')

        if not self.client.wait_for_server(timeout_sec=10.0):
            print('Nav2 is not running. Start navigation.launch.py first.')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        fut = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle or not handle.accepted:
            print('Goal rejected by Nav2 (is the point reachable?).')
            return
        res_fut = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, res_fut)
        except KeyboardInterrupt:
            handle.cancel_goal_async()
            print('\nCancelled.')
            return
        status = res_fut.result().status
        print({4: f'Arrived at {loc["display_name"]}.', 5: 'Cancelled.',
               6: 'Failed - Nav2 aborted the goal.'}.get(status, f'Finished (status {status}).'))


def main(args=None):
    rclpy.init(args=args)
    node = Goto()
    dest = node.get_parameter('destination').value
    try:
        if dest:
            node.go(dest)
        else:
            print('Places:', ', '.join(v['display_name'] for v in node.locations.values()))
            while rclpy.ok():
                text = input('\nWhere to? (Ctrl-C to quit) ')
                if text.strip():
                    node.go(text)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Autonomous navigation demo for the Jackal sim using Nav2.

Launch the sim first (separate terminal):
    ros2 launch driftbot_bringup sim.launch.py start_rviz:=true enable_nav2:=true

Then run this script. It sends a sequence of navigation goals through the corridor.

Run:  source /opt/ros/jazzy/setup.zsh && source ros2_ws/install/setup.zsh
      python3 scripts/sim_nav2_demo.py
"""

import math
import os
import time

import rclpy
import rclpy.parameter
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# Waypoints through the corridor (x, y, yaw)
# Corridor: x in [-2, 6], y in [-1.2, 1.2]
# Boxes at (3.5, 0.8) and (4.8, -0.7)
WAYPOINTS = [
    (0.0, 0.0, 0.0),      # start center
    (2.0, 0.0, 0.0),      # straight
    (4.0, 0.0, 0.0),      # between boxes
    (6.0, 0.0, 0.0),      # near front wall
    (4.0, -0.5, -0.5),    # back down right side
    (2.0, 0.0, 0.0),      # return center
    (0.0, 0.0, 0.0),      # return start
]


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Nav2Demo(Node):
    def __init__(self):
        super().__init__('sim_nav2_demo', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.results = []

    def send_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            return True
        else:
            self.get_logger().error(f'Goal failed with status: {result.status}')
            return False

    def run(self):
        self.get_logger().info('Waiting for Nav2 action server...')
        # Wait longer for Nav2 to fully start up
        if not self.client.wait_for_server(timeout_sec=120.0):
            self.get_logger().error('Nav2 action server not available after 120s')
            return

        self.get_logger().info('Nav2 action server ready, starting waypoints')
        for i, (x, y, yaw) in enumerate(WAYPOINTS):
            self.get_logger().info(f'=== Waypoint {i+1}/{len(WAYPOINTS)} ===')
            success = self.send_goal(x, y, yaw)
            self.results.append((x, y, yaw, success))
            if not success:
                self.get_logger().warn('Waypoint failed, continuing to next...')
            time.sleep(2.0)  # Give more time between waypoints

        self.get_logger().info('=== Navigation demo complete ===')
        for i, (x, y, yaw, ok) in enumerate(self.results):
            status = 'OK' if ok else 'FAILED'
            self.get_logger().info(f'  WP {i+1}: ({x:.2f}, {y:.2f}, {yaw:.2f}) -> {status}')


def main():
    rclpy.init()
    node = Nav2Demo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
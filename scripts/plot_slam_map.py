#!/usr/bin/env python3
"""Drive the Gazebo-simulated DriftBot and capture the SLAM occupancy grid as a PNG.

Launch the sim first (separate terminal):
    ros2 launch driftbot_bringup sim.launch.py start_rviz:=false

Then run this script. It:
  1. Drives the robot through the cardboard corridor (arc + straight legs)
  2. Records /odom trajectory while driving
  3. Captures the final /map (nav_msgs/OccupancyGrid) and the trajectory
  4. Saves docs/img/slam_map_sim.png

Run:  source /opt/ros/jazzy/setup.zsh && source ros2_ws/install/setup.zsh
      python3 scripts/plot_slam_map.py
"""

import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

OUT = os.path.join(os.path.dirname(__file__), '..', 'docs', 'img',
                   'slam_map_sim.png')


def drive_and_capture():
    rclpy.init()
    node = rclpy.create_node('map_capture')

    odom = []
    got_map = {}

    def on_map(msg):
        got_map['msg'] = msg

    def on_odom(msg):
        odom.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    node.create_subscription(OccupancyGrid, '/map', on_map, 10)
    node.create_subscription(Odometry, '/odom', on_odom, 50)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # wait for the first map + odom
    print('waiting for /map and /odom ...')
    t0 = node.get_clock().now()
    while rclpy.ok() and ('msg' not in got_map or len(odom) < 5):
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - t0).nanoseconds > 60e9:
            raise SystemExit('timeout waiting for /map + /odom — '
                             'is the sim running?')

    # drive: two arcs + a straight leg through the corridor
    print('driving ...')
    for linear, angular, dur in [
        (0.15, 0.0, 6.0),   # straight ahead
        (0.15, 0.35, 6.0),   # arc left
        (0.15, 0.0, 4.0),   # straight
        (0.15, -0.35, 5.0),  # arc right
        (0.10, 0.0, 5.0),   # slow straight
    ]:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        t_end = node.get_clock().now() + Duration(seconds=dur)
        while node.get_clock().now() < t_end:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)

    # ALWAYS stop (gz AckermannSteering latches the last cmd_vel)
    print('stopping ...')
    stop = Twist()
    for _ in range(20):
        pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)

    # let SLAM process the last scans
    print('letting SLAM settle ...')
    t_end = node.get_clock().now() + Duration(seconds=4.0)
    while node.get_clock().now() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f'capturing map (drove {len(odom)} odom samples) ...')
    grid = got_map['msg']

    # ── plot ─────────────────────────────────────────────────────────────
    a = np.array(grid.data, dtype=np.int8).reshape(grid.info.height,
                                                    grid.info.width)
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y

    fig, ax = plt.subplots(figsize=(10, 8))
    cmap = matplotlib.colors.ListedColormap(
        ['#e2e8f0', '#ffffff', '#1e293b', '#f59e0b'])
    norm = matplotlib.colors.BoundaryNorm([-1.5, -0.5, 0.5, 90, 256], 4)
    ax.imshow(a, origin='lower', cmap=cmap, norm=norm,
              extent=[ox, ox + a.shape[1] * res,
                      oy, oy + a.shape[0] * res])
    # occupied = dark, unknown = light gray, free = white, cost = amber

    if odom:
        xs, ys = zip(*odom)
        ax.plot(xs, ys, '-', color='#dc2626', lw=2.0,
                label=f'odom trajectory ({len(odom)} samples)')
        ax.plot(xs[0], ys[0], 'o', color='#16a34a', ms=9,
                label='start')
        ax.plot(xs[-1], ys[-1], 's', color='#7c3aed', ms=9,
                label='end')

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('SLAM occupancy grid — Gazebo Harmonic sim\n'
                 'slam_toolbox async SLAM, 3-beam rotating ToF /scan '
                 f'({a.shape[1]}x{a.shape[0]} cells @ {res:.2f} m/cell, '
                 'cardboard corridor world)')
    ax.legend(loc='upper right', fontsize=9)
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(OUT, dpi=150)
    plt.close(fig)

    occupied = int((a > 50).sum())
    free = int((a == 0).sum())
    unknown = int((a < 0).sum())
    print(f'wrote {OUT}')
    print(f'map {a.shape[1]}x{a.shape[0]} cells: '
          f'{occupied} occupied, {free} free, {unknown} unknown')
    rclpy.shutdown()


if __name__ == '__main__':
    drive_and_capture()

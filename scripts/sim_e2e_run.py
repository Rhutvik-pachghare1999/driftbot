#!/usr/bin/env python3
"""End-to-end Gazebo simulation run: drive, measure, capture results, compare vs ground truth.

Run the sim first (separate terminal):
    ros2 launch driftbot_bringup sim.launch.py start_rviz:=false record_bag:=true

Then:
    python3 scripts/sim_e2e_run.py

What it does:
  1. Verifies every ROS topic in the stack is live (waits for /map)
  2. Measures live publish rates for 5 s (all key topics)
  3. Drives a coverage path (straights + arcs through the corridor)
     while recording the /odom trajectory
  4. Stops the robot (gz drive latches the last cmd_vel)
  5. Captures the final SLAM /map OccupancyGrid
  6. Compares SLAM map against ground-truth corridor world (IoU, wall RMSE)
  7. Writes results:
       - docs/img/slam_map_sim.png     map + trajectory figure
       - docs/maps/sim_corridor_map.pgm/.yaml  nav2-format map files
       - prints a results summary (map size, occupied cells, path length, IoU, RMSE)
"""

import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, Range, Imu, Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
IMG = os.path.join(REPO, 'docs', 'img')
MAPS = os.path.join(REPO, 'docs', 'maps')
os.makedirs(IMG, exist_ok=True)
os.makedirs(MAPS, exist_ok=True)

RATE_TOPICS = [
    ('/scan', LaserScan),
    ('/servo/position', Float32),
    ('/tof/sensor_0', Range),
    ('/tof/sensor_1', Range),
    ('/tof/sensor_2', Range),
    ('/imu/data', Imu),
    ('/camera/image_raw', Image),
    ('/odom', Odometry),
    ('/map', OccupancyGrid),
]

# Coverage path: (linear m/s, angular rad/s, duration s)
DRIVE_PLAN = [
    (0.15, 0.0, 8.0),    # straight down the corridor
    (0.12, 0.45, 6.0),    # arc left
    (0.15, 0.0, 6.0),     # straight
    (0.12, -0.45, 6.0),   # arc right
    (0.15, 0.0, 6.0),     # straight
    (0.10, 0.5, 5.0),     # slower arc left
    (0.15, 0.0, 5.0),     # straight
    (0.10, -0.5, 4.0),    # arc right
    (0.12, 0.0, 6.0),     # final straight
]

# Ground-truth corridor world parameters (from driftbot_corridor.sdf)
# Corridor: 8.6m long, walls at y = ±1.25, floor from x = -0.3 to x = 8.3
GT_CORRIDOR_X_MIN = -0.3
GT_CORRIDOR_X_MAX = 8.3
GT_CORRIDOR_Y_MIN = -1.25
GT_CORRIDOR_Y_MAX = 1.25
GT_WALL_THICKNESS = 0.1  # walls are 0.1m thick


def build_ground_truth_map(resolution, origin_x, origin_y, width, height):
    """Build ground-truth occupancy grid from known corridor geometry.

    Returns array of same shape as SLAM map: 0=free, 100=occupied, -1=unknown.
    """
    gt = np.full((height, width), -1, dtype=np.int8)

    # World coordinates of each cell center
    xs = origin_x + (np.arange(width) + 0.5) * resolution
    ys = origin_y + (np.arange(height) + 0.5) * resolution

    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            # Outside corridor bounds = unknown (not mapped)
            if x < GT_CORRIDOR_X_MIN - 2.0 or x > GT_CORRIDOR_X_MAX + 2.0 \
               or y < GT_CORRIDOR_Y_MIN - 2.0 or y > GT_CORRIDOR_Y_MAX + 2.0:
                gt[iy, ix] = -1
            elif x < GT_CORRIDOR_X_MIN or x > GT_CORRIDOR_X_MAX \
                 or y < GT_CORRIDOR_Y_MIN or y > GT_CORRIDOR_Y_MAX:
                # Outside corridor but in sensor range = free
                gt[iy, ix] = 0
            else:
                # Inside corridor: check if in wall
                in_left_wall = (GT_CORRIDOR_Y_MIN <= y <= GT_CORRIDOR_Y_MIN + GT_WALL_THICKNESS)
                in_right_wall = (GT_CORRIDOR_Y_MAX - GT_WALL_THICKNESS <= y <= GT_CORRIDOR_Y_MAX)
                if in_left_wall or in_right_wall:
                    gt[iy, ix] = 100
                else:
                    gt[iy, ix] = 0

    return gt


def compute_iou(slam_map, gt_map):
    """Compute IoU for occupied cells (slam >= 65, gt == 100)."""
    slam_occ = slam_map >= 65
    gt_occ = gt_map == 100
    intersection = np.logical_and(slam_occ, gt_occ).sum()
    union = np.logical_or(slam_occ, gt_occ).sum()
    if union == 0:
        return 0.0
    return intersection / union


def compute_wall_rmse(slam_map, gt_map, resolution, origin_x, origin_y):
    """Compute RMSE of wall positions (occupied cells vs ground truth walls)."""
    slam_occ = np.argwhere(slam_map >= 65)
    gt_occ = np.argwhere(gt_map == 100)

    if len(slam_occ) == 0 or len(gt_occ) == 0:
        return float('inf')

    # Convert to world coordinates
    slam_world = np.column_stack([
        origin_x + (slam_occ[:, 1] + 0.5) * resolution,
        origin_y + (slam_occ[:, 0] + 0.5) * resolution,
    ])
    gt_world = np.column_stack([
        origin_x + (gt_occ[:, 1] + 0.5) * resolution,
        origin_y + (gt_occ[:, 0] + 0.5) * resolution,
    ])

    # For each SLAM occupied cell, find nearest GT wall cell
    from scipy.spatial import KDTree
    tree = KDTree(gt_world)
    dists, _ = tree.query(slam_world, k=1)
    return np.sqrt(np.mean(dists**2))


def main():
    rclpy.init()
    node = rclpy.create_node('sim_e2e_run')

    odom = []
    rates = {t: 0 for t, _ in RATE_TOPICS}
    got = {}

    def mk(sub_topic):
        def cb(msg):
            rates[sub_topic] += 1
            if sub_topic == '/odom':
                odom.append((msg.pose.pose.position.x,
                             msg.pose.pose.position.y))
            elif sub_topic == '/map' and '/map' not in got:
                got['/map'] = msg
        return cb

    for t, mt in RATE_TOPICS:
        node.create_subscription(mt, t, mk(t), 20)
    cmd = node.create_publisher(Twist, '/cmd_vel', 10)

    # 1. wait for first map + odom
    print('[1/7] waiting for /map and /odom ...', flush=True)
    t0 = node.get_clock().now()
    while rclpy.ok() and ('/map' not in got or len(odom) < 10):
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - t0).nanoseconds > 90e9:
            raise SystemExit('TIMEOUT waiting for /map + /odom — '
                             'is the sim running? (ros2 launch driftbot_bringup '
                             'sim.launch.py start_rviz:=false)')
    print(f'    ok: /map alive, {len(odom)} odom samples', flush=True)

    # 2. measure rates for 5 s
    print('[2/7] measuring topic rates for 5 s ...', flush=True)
    for t in rates:
        rates[t] = 0
    t_end = node.get_clock().now() + Duration(seconds=5.0)
    while node.get_clock().now() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    measured_rates = {t: n / 5.0 for t, n in rates.items()}
    print('    measured rates:', flush=True)
    for t, r in measured_rates.items():
        print(f'      {t:<20} {r:6.2f} Hz', flush=True)

    # 3. drive the coverage path
    print('[3/7] driving coverage path (52 s) ...', flush=True)
    path_len = 0.0
    for linear, angular, dur in DRIVE_PLAN:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        t_end = node.get_clock().now() + Duration(seconds=dur)
        prev = odom[-1] if odom else None
        while node.get_clock().now() < t_end:
            cmd.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            if prev is not None and odom:
                path_len += math.hypot(odom[-1][0] - prev[0],
                                       odom[-1][1] - prev[1])
                prev = odom[-1]

    # 4. stop (gz drive latches last cmd_vel!)
    print('[4/7] stopping (zero Twist x20) ...', flush=True)
    stop = Twist()
    for _ in range(20):
        cmd.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)

    # 5. let SLAM settle, capture final map
    print('[5/7] letting SLAM settle 6 s, capturing final /map ...',
          flush=True)
    got.pop('/map', None)  # take a fresh final map
    t_end = node.get_clock().now() + Duration(seconds=6.0)
    while node.get_clock().now() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    deadline = node.get_clock().now() + Duration(seconds=10.0)
    while '/map' not in got and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if '/map' not in got:
        raise SystemExit('no /map captured — SLAM not publishing')
    grid = got['/map']
    print(f'    map: {grid.info.width}x{grid.info.height} cells '
          f'@ {grid.info.resolution:.3f} m/cell', flush=True)

    # 6. compare SLAM map against ground truth
    print('[6/7] comparing SLAM map against ground truth ...', flush=True)
    a = np.array(grid.data, dtype=np.int8).reshape(grid.info.height,
                                                    grid.info.width)
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y

    gt = build_ground_truth_map(res, ox, oy, grid.info.width, grid.info.height)
    iou = compute_iou(a, gt)
    rmse = compute_wall_rmse(a, gt, res, ox, oy)

    print(f'    IoU (occupied): {iou:.3f}', flush=True)
    print(f'    Wall RMSE: {rmse:.3f} m', flush=True)

    # 7. write results
    print('[7/7] writing results ...', flush=True)

    # nav2 map_server format: PGM (0=occupied, 254=free, 205=unknown)
    pgm = np.full(a.shape, 205, dtype=np.uint8)
    pgm[a >= 65] = 0
    pgm[(a >= 0) & (a < 65)] = 254
    with open(os.path.join(MAPS, 'sim_corridor_map.pgm'), 'wb') as f:
        f.write(f'P5\n{a.shape[1]} {a.shape[0]}\n255\n'.encode())
        f.write(pgm.tobytes())
    with open(os.path.join(MAPS, 'sim_corridor_map.yaml'), 'w') as f:
        f.write(
            f'image: sim_corridor_map.pgm\n'
            f'resolution: {res}\n'
            f'origin: [{ox}, {oy}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
        )
    print('    docs/maps/sim_corridor_map.pgm/.yaml', flush=True)

    # figure: map + trajectory + ground truth walls
    fig, ax = plt.subplots(figsize=(10, 8))
    cmap = matplotlib.colors.ListedColormap(
        ['#e2e8f0', '#ffffff', '#1e293b', '#f59e0b'])
    norm = matplotlib.colors.BoundaryNorm([-1.5, -0.5, 0.5, 65, 256], 4)
    ax.imshow(a, origin='lower', cmap=cmap, norm=norm,
              extent=[ox, ox + a.shape[1] * res,
                      oy, oy + a.shape[0] * res])
    if odom:
        xs, ys = zip(*odom)
        ax.plot(xs, ys, '-', color='#dc2626', lw=2.0,
                label=f'odom trajectory ({len(odom)} pts, '
                      f'{path_len:.1f} m driven)')
        ax.plot(xs[0], ys[0], 'o', color='#16a34a', ms=10, label='start')
        ax.plot(xs[-1], ys[-1], 's', color='#7c3aed', ms=10, label='end')

    # Draw ground truth walls
    ax.axhline(GT_CORRIDOR_Y_MIN, color='#000000', lw=1.5, ls='--', alpha=0.5, label='GT walls')
    ax.axhline(GT_CORRIDOR_Y_MAX, color='#000000', lw=1.5, ls='--', alpha=0.5)
    ax.axvline(GT_CORRIDOR_X_MIN, color='#000000', lw=1.5, ls='--', alpha=0.5)
    ax.axvline(GT_CORRIDOR_X_MAX, color='#000000', lw=1.5, ls='--', alpha=0.5)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('SLAM occupancy grid — Gazebo Harmonic sim '
                 f'({a.shape[1]}x{a.shape[0]} cells @ {res:.2f} m/cell)\n'
                 f'IoU={iou:.3f}  Wall RMSE={rmse:.3f}m  '
                 'slam_toolbox async SLAM, 3-beam rotating ToF /scan, '
                 'cardboard corridor world')
    ax.legend(loc='upper right', fontsize=9)
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'slam_map_sim.png'), dpi=150)
    plt.close(fig)
    print('    docs/img/slam_map_sim.png', flush=True)

    occupied = int((a >= 65).sum())
    free = int((a == 0).sum())
    unknown = int((a < 0).sum())
    print(flush=True)
    print('================ END-TO-END RESULTS ================')
    print(f'  drive path: {len(DRIVE_PLAN)} segments, '
          f'{sum(d for _, _, d in DRIVE_PLAN):.0f} s, '
          f'{path_len:.2f} m driven (odom)')
    print(f'  odom samples: {len(odom)}')
    print(f'  SLAM map: {a.shape[1]}x{a.shape[0]} cells '
          f'@ {res:.3f} m/cell '
          f'({a.shape[1] * res:.1f} x {a.shape[0] * res:.1f} m)')
    print(f'  cells: {occupied} occupied, {free} free, {unknown} unknown')
    print(f'  IoU (occupied): {iou:.3f}')
    print(f'  Wall RMSE: {rmse:.3f} m')
    for t, r in measured_rates.items():
        print(f'  rate {t:<20} {r:6.2f} Hz')
    print('====================================================', flush=True)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

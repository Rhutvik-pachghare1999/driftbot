#!/usr/bin/env python3
"""End-to-end Gazebo simulation run for the Jackal-based stack:
drive, measure, capture results, compare against ground truth.

Run the sim first (separate terminal):
    ros2 launch driftbot_bringup sim.launch.py start_rviz:=false

Then:
    python3 scripts/sim_e2e_run.py

What it does:
  1. Verifies the stack is live (waits for /map + all three odom streams)
  2. Measures live publish rates for 5 s (sensor + odometry topics)
  3. Drives a coverage path through the corridor (straight + S-curves +
     a U-turn past the box obstacles) via /platform/cmd_vel TwistStamped
     at 10 Hz, recording /platform/odom, /odom (EKF) and /gt_odom
  4. Stops the robot and lets SLAM settle, then captures the final /map
  5. Computes trajectory error vs ground truth (ATE RMSE + final pose)
  6. Compares the SLAM map against the ground-truth world geometry
     (occupied-cell IoU + RMSE, walls + boxes rasterized from the SDF)
  7. Writes results:
       - docs/img/slam_map_sim.png            map + trajectory figure
       - docs/maps/sim_corridor_map.pgm/.yaml nav2-format map files
       - docs/maps/sim_e2e_results.json       machine-readable metrics
       - stdout summary
"""

import json
import math
import os
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import rclpy.parameter
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
IMG = os.path.join(REPO, 'docs', 'img')
MAPS = os.path.join(REPO, 'docs', 'maps')
os.makedirs(IMG, exist_ok=True)
os.makedirs(MAPS, exist_ok=True)

RATE_TOPICS = [
    ('/scan', LaserScan),
    ('/imu/data', Imu),
    ('/platform/odom', Odometry),
    ('/odom', Odometry),
    ('/gt_odom', Odometry),
    ('/platform/joint_states', JointState),
]

# Coverage path (linear m/s, angular rad/s, duration s).
# Stays on/near the corridor centerline; box obstacles are at
# (3.5, 0.8, yaw 0.4) and (4.8, -0.7, yaw -0.3) — the plan passes between
# them with >= 0.15 m clearance and U-turns before the far wall.
DRIVE_PLAN = [
    (0.20, 0.00, 8.0),    # straight out
    (0.20, 0.08, 4.0),    # S-curve up
    (0.20, -0.08, 4.0),   # S-curve
    (0.20, -0.08, 4.0),   # S-curve down
    (0.20, 0.08, 4.0),    # S-curve back to centerline (~x 4.75)
    (0.03, -0.50, 6.28),  # in-place U-turn (pi rad)
    (0.20, 0.00, 9.0),    # straight back through the box gap
    (0.20, 0.00, 9.0),    # straight back
    (0.10, 0.00, 4.0),    # slow final approach
]

# Ground-truth world (driftbot_corridor.sdf):
#   side walls   y = +/-1.25 center, 0.1 thick, x in [-2.2, 6.2], 0.4 tall
#   end walls    x = -2.3 / 6.3 center, 0.1 thick, y in [-1.3, 1.3]
#   box_left     0.3x0.3 at (3.5, 0.8), yaw 0.4,  0.5 tall
#   box_right    0.25x0.25 at (4.8, -0.7), yaw -0.3, 0.5 tall
#   lidar plane  z = 0.241 (sees walls at 0.4 and boxes at 0.5)
GT_WALLS = [  # (x0, x1, y0, y1) axis-aligned rectangles
    (-2.2, 6.2, 1.20, 1.30),
    (-2.2, 6.2, -1.30, -1.20),
    (-2.35, -2.25, -1.30, 1.30),
    (6.25, 6.35, -1.30, 1.30),
]
GT_BOXES = [  # (cx, cy, half, yaw) rotated squares
    (3.5, 0.8, 0.15, 0.4),
    (4.8, -0.7, 0.125, -0.3),
]
GT_INNER = (-2.25, 6.25, -1.20, 1.20)  # free-space interior bounds
GT_UNKNOWN_MARGIN = 2.0                 # outside corridor + this = unknown


def in_rect(x, y, rect):
    return rect[0] <= x <= rect[1] and rect[2] <= y <= rect[3]


def in_box(x, y, box):
    cx, cy, half, yaw = box
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = x - cx, y - cy
    rx, ry = c * dx - s * dy, s * dx + c * dy
    return abs(rx) <= half and abs(ry) <= half


def build_ground_truth_map(res, ox, oy, w, h):
    """Rasterize the known world into the SLAM map's frame.

    Returns int8 array: 100 = occupied (walls/boxes), 0 = free, -1 = unknown.
    """
    gt = np.full((h, w), -1, dtype=np.int8)
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    x0i, x1i, y0i, y1i = GT_INNER
    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            if x < x0i - GT_UNKNOWN_MARGIN or x > x1i + GT_UNKNOWN_MARGIN \
               or y < y0i - GT_UNKNOWN_MARGIN or y > y1i + GT_UNKNOWN_MARGIN:
                continue  # unknown
            if any(in_rect(x, y, r) for r in GT_WALLS) \
               or any(in_box(x, y, b) for b in GT_BOXES):
                gt[iy, ix] = 100
            else:
                gt[iy, ix] = 0
    return gt


def compute_iou(slam, gt):
    slam_occ = slam >= 65
    gt_occ = gt == 100
    union = np.logical_or(slam_occ, gt_occ).sum()
    return float(np.logical_and(slam_occ, gt_occ).sum() / union) if union else 0.0


def dist_to_surfaces(pts):
    """Distance from each point to the nearest GT wall/box surface (m)."""
    pts = np.atleast_2d(pts)
    d = np.full(len(pts), np.inf)
    for x0, x1, y0, y1 in GT_WALLS:  # axis-aligned rects
        dx = np.maximum(np.maximum(x0 - pts[:, 0], pts[:, 0] - x1), 0)
        dy = np.maximum(np.maximum(y0 - pts[:, 1], pts[:, 1] - y1), 0)
        d = np.minimum(d, np.hypot(dx, dy))
    for cx, cy, half, yaw in GT_BOXES:  # rotated squares
        c, s = math.cos(-yaw), math.sin(-yaw)
        rx = c * (pts[:, 0] - cx) - s * (pts[:, 1] - cy)
        ry = s * (pts[:, 0] - cx) + c * (pts[:, 1] - cy)
        dx = np.maximum(np.abs(rx) - half, 0)
        dy = np.maximum(np.abs(ry) - half, 0)
        d = np.minimum(d, np.hypot(dx, dy))
    return d


def observable_surface_samples(step=0.02):
    """Sample the surfaces the robot's lidar could actually see."""
    pts = []
    x0, x1, y0, y1 = GT_INNER
    for x in np.arange(x0, x1 + 1e-9, step):   # inner faces of side walls
        pts += [(x, 1.20), (x, -1.20)]
    for y in np.arange(y0, y1 + 1e-9, step):   # inner faces of end walls
        pts += [(x0, y), (x1, y)]
    for cx, cy, half, yaw in GT_BOXES:          # box perimeters
        c, s = math.cos(yaw), math.sin(yaw)
        for t in np.arange(-half, half + 1e-9, step):
            for ex, ey in ((t, -half), (t, half), (-half, t), (half, t)):
                pts.append((cx + c * ex - s * ey, cy + s * ex + c * ey))
    return np.array(pts)


def compute_map_metrics(slam, res, ox, oy):
    """Surface-based precision/recall (immune to raster-extent artifacts)."""
    from scipy.spatial import KDTree
    idx = np.argwhere(slam >= 65)
    pts = np.column_stack([ox + (idx[:, 1] + 0.5) * res,
                           oy + (idx[:, 0] + 0.5) * res])
    prec_d = dist_to_surfaces(pts) if len(pts) else np.array([np.inf])
    samples = observable_surface_samples()
    if len(pts):
        dists, _ = KDTree(pts).query(samples, k=1)
    else:
        dists = np.full(len(samples), np.inf)
    return {
        'precision_rmse_m': float(np.sqrt(np.mean(prec_d ** 2))),
        'precision_within_5cm': float((prec_d <= 0.05).mean()),
        'precision_within_10cm': float((prec_d <= 0.10).mean()),
        'spurious_cells_beyond_30cm': int((prec_d > 0.30).sum()),
        'recall_within_10cm': float((dists <= 0.10).mean()),
        'n_surface_samples': int(len(samples)),
    }


def sync_ate(est, gt, t_start=None, t_end=None):
    """ATE RMSE of est vs gt, synced by nearest header stamp.

    est/gt: lists of (stamp_sec, x, y, yaw). Returns (rmse, n, final_err).
    If t_start/t_end provided, only use est samples in [t_start, t_end].
    """
    if not est or not gt:
        return float('inf'), 0, {}
    gts = np.array([g[0] for g in gt])
    errs, last = [], None
    for t, x, y, yaw in est:
        if t_start is not None and t < t_start:
            continue
        if t_end is not None and t > t_end:
            continue
        i = int(np.argmin(np.abs(gts - t)))
        if abs(gts[i] - t) > 0.1:
            continue
        gx, gy, gyaw = gt[i][1], gt[i][2], gt[i][3]
        errs.append(math.hypot(x - gx, y - gy))
        last = (x - gx, y - gy, math.atan2(math.sin(yaw - gyaw),
                                           math.cos(yaw - gyaw)))
    rmse = float(np.sqrt(np.mean(np.square(errs)))) if errs else float('inf')
    final = {}
    if last:
        final = {'x': round(last[0], 4), 'y': round(last[1], 4),
                 'heading': round(last[2], 4),
                 'euclidean': round(math.hypot(last[0], last[1]), 4)}
    return rmse, len(errs), final


def main():
    rclpy.init()
    node = rclpy.create_node('sim_e2e_run', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    traj = {t: [] for t in ('/platform/odom', '/odom', '/gt_odom')}
    rates = {t: 0 for t, _ in RATE_TOPICS}
    got = {}

    def mk_odom_cb(key):
        def cb(msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y ** 2 + q.z ** 2))
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            traj[key].append((t, p.x, p.y, yaw))
        return cb

    be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    for key in traj:
        node.create_subscription(Odometry, key, mk_odom_cb(key), be)

    def mk_rate_cb(key):
        def cb(msg):
            rates[key] += 1
        return cb

    for t, mt in RATE_TOPICS:
        node.create_subscription(mt, t, mk_rate_cb(t), be)

    def map_cb(msg):
        got['/map'] = msg

    node.create_subscription(OccupancyGrid, '/map', map_cb, 20)

    pub = node.create_publisher(TwistStamped, '/platform/cmd_vel', 10)

    # 1. wait for the stack to be fully alive
    print('[1/7] waiting for /map + odom streams ...', flush=True)
    t0 = time.monotonic()
    while rclpy.ok() and ('/map' not in got or any(len(v) < 10 for v in traj.values())):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t0 > 120:
            raise SystemExit('TIMEOUT waiting for stack — is the sim running? '
                             '(ros2 launch driftbot_bringup sim.launch.py start_rviz:=false)')
    print(f'    ok: map {got["/map"].info.width}x{got["/map"].info.height}, '
          f'odom samples {[len(v) for v in traj.values()]}', flush=True)

    if pub.get_subscription_count() == 0:
        raise SystemExit('/platform/cmd_vel has no subscriber — '
                         'is platform_velocity_controller active?')

    # 2. measure rates for 5 s
    print('[2/7] measuring topic rates for 5 s ...', flush=True)
    for t in rates:
        rates[t] = 0
    t_end = time.monotonic() + 5.0
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    measured = {t: n / 5.0 for t, n in rates.items()}
    for t, r in measured.items():
        print(f'      {t:<22} {r:6.2f} Hz', flush=True)

    # 3. drive the coverage path (10 Hz wall-paced, sim-stamped TwistStamped)
    total = sum(d for _, _, d in DRIVE_PLAN)
    print(f'[3/7] driving coverage path ({len(DRIVE_PLAN)} segments, '
          f'{total:.0f} s) ...', flush=True)
    cmd = TwistStamped()
    n_cmds = 0
    drive_start_wall = time.monotonic()
    drive_start_sim = None
    last_cmd_sim = None
    for linear, angular, dur in DRIVE_PLAN:
        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular
        start, next_pub = time.monotonic(), time.monotonic()
        end = start + dur
        while time.monotonic() < end:
            if time.monotonic() >= next_pub:
                stamp = node.get_clock().now().to_msg()
                cmd.header.stamp = stamp
                if drive_start_sim is None:
                    drive_start_sim = stamp.sec + stamp.nanosec * 1e-9
                last_cmd_sim = stamp.sec + stamp.nanosec * 1e-9
                pub.publish(cmd)
                n_cmds += 1
                next_pub += 0.1
            rclpy.spin_once(node, timeout_sec=0.01)
    drive_end_wall = time.monotonic()
    # last_cmd_sim holds the sim timestamp of the last drive command

    # 4. stop + settle
    print('[4/7] stopping + letting SLAM settle 6 s ...', flush=True)
    stop = TwistStamped()
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        stop.header.stamp = node.get_clock().now().to_msg()
        pub.publish(stop)
        time.sleep(0.025)
    got.pop('/map', None)
    end = time.monotonic() + 6.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    deadline = time.monotonic() + 10.0
    while '/map' not in got and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if '/map' not in got:
        raise SystemExit('no /map captured — SLAM not publishing')
    grid = got['/map']

    # 5. trajectory error vs ground truth (only during the drive window)
    print('[5/7] computing trajectory error vs /gt_odom ...', flush=True)
    ate_ekf, n_ekf, fin_ekf = sync_ate(traj['/odom'], traj['/gt_odom'],
                                        drive_start_sim, last_cmd_sim)
    ate_ctrl, n_ctrl, fin_ctrl = sync_ate(traj['/platform/odom'], traj['/gt_odom'],
                                           drive_start_sim, last_cmd_sim)
    print(f'    EKF /odom     ATE RMSE {ate_ekf:.4f} m over {n_ekf} synced samples', flush=True)
    print(f'    ctrl /platform ATE RMSE {ate_ctrl:.4f} m over {n_ctrl} synced samples', flush=True)

    # path length from EKF odom (drive window only)
    o = [(t, x, y) for t, x, y, yaw in traj['/odom']
         if (drive_start_sim is None or t >= drive_start_sim) and
            (last_cmd_sim is None or t <= last_cmd_sim)]
    path_len = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(o, o[1:]))

    # 6. map comparison against ground-truth geometry
    print('[6/7] comparing SLAM map against ground truth ...', flush=True)
    a = np.array(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
    res, ox, oy = grid.info.resolution, grid.info.origin.position.x, \
        grid.info.origin.position.y
    gt = build_ground_truth_map(res, ox, oy, grid.info.width, grid.info.height)
    iou = compute_iou(a, gt)  # reference only — raster-thickness sensitive
    mm = compute_map_metrics(a, res, ox, oy)
    print(f"    precision: RMSE {mm['precision_rmse_m']:.3f} m, "
          f"{mm['precision_within_10cm'] * 100:.1f}% of occupied cells "
          f"within 10 cm of a GT surface", flush=True)
    print(f"    spurious cells (>30 cm from any GT surface): "
          f"{mm['spurious_cells_beyond_30cm']}", flush=True)
    print(f"    recall: {mm['recall_within_10cm'] * 100:.1f}% of observable "
          f"GT surface within 10 cm of an occupied cell", flush=True)
    print(f'    (raster IoU, reference: {iou:.3f})', flush=True)

    # 7. write results
    print('[7/7] writing results ...', flush=True)
    pgm = np.full(a.shape, 205, dtype=np.uint8)
    pgm[a >= 65] = 0
    pgm[(a >= 0) & (a < 65)] = 254
    with open(os.path.join(MAPS, 'sim_corridor_map.pgm'), 'wb') as f:
        f.write(f'P5\n{a.shape[1]} {a.shape[0]}\n255\n'.encode())
        f.write(pgm.tobytes())
    with open(os.path.join(MAPS, 'sim_corridor_map.yaml'), 'w') as f:
        f.write(f'image: sim_corridor_map.pgm\nresolution: {res}\n'
                f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.25\n')

    # figure: SLAM map + EKF trajectory + GT geometry
    fig, ax = plt.subplots(figsize=(10, 7))
    cmap = matplotlib.colors.ListedColormap(
        ['#e2e8f0', '#ffffff', '#1e293b'])
    norm = matplotlib.colors.BoundaryNorm([-1.5, -0.5, 65, 256], 3)
    ax.imshow(a, origin='lower', cmap=cmap, norm=norm,
              extent=[ox, ox + a.shape[1] * res, oy, oy + a.shape[0] * res])
    if o:
        xs, ys = zip(*[(p[1], p[2]) for p in o])
        ax.plot(xs, ys, '-', color='#dc2626', lw=2.0,
                label=f'EKF /odom trajectory ({path_len:.1f} m)')
        ax.plot(xs[0], ys[0], 'o', color='#16a34a', ms=10, label='start')
        ax.plot(xs[-1], ys[-1], 's', color='#7c3aed', ms=10, label='end')
    for i, r in enumerate(GT_WALLS):
        ax.plot([r[0], r[1]], [r[2], r[2]], 'k--', lw=1.5, alpha=0.6,
                label='GT walls' if i == 0 else None)
        ax.plot([r[0], r[1]], [r[3], r[3]], 'k--', lw=1.5, alpha=0.6)
    for i, b in enumerate(GT_BOXES):
        cx, cy, half, yaw = b
        corners = np.array([(-half, -half), (half, -half), (half, half),
                           (-half, half), (-half, -half)])
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.column_stack([c * corners[:, 0] - s * corners[:, 1],
                               s * corners[:, 0] + c * corners[:, 1]])
        ax.plot(rot[:, 0] + cx, rot[:, 1] + cy, color='#f59e0b', lw=1.5,
                label='GT boxes' if i == 0 else None)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('SLAM map — Jackal (j100) in Gazebo Harmonic corridor world\n'
                 f'slam_toolbox + SICK lidar /scan + single-stream EKF  |  '
                 f"precision RMSE {mm['precision_rmse_m']:.3f} m "
                 f"({mm['precision_within_10cm'] * 100:.0f}% ≤10 cm), "
                 f"spurious {mm['spurious_cells_beyond_30cm']}, "
                 f"recall {mm['recall_within_10cm'] * 100:.0f}%, "
                 f"EKF ATE={ate_ekf:.3f} m")
    ax.legend(loc='upper right', fontsize=9)
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'slam_map_sim.png'), dpi=150)
    plt.close(fig)

    results = {
        'drive': {'segments': len(DRIVE_PLAN), 'duration_s': total,
                  'cmds_published': n_cmds, 'path_len_odom_m': round(path_len, 2)},
        'rates_hz': {t: round(r, 2) for t, r in measured.items()},
        'trajectory': {
            'ate_rmse_ekf_m': round(ate_ekf, 4),
            'ate_rmse_controller_m': round(ate_ctrl, 4),
            'synced_samples_ekf': n_ekf,
            'final_pose_error_ekf': fin_ekf,
            'final_pose_error_controller': fin_ctrl,
        },
        'map': {'width': grid.info.width, 'height': grid.info.height,
                'resolution_m': res,
                'occupied_cells': int((a >= 65).sum()),
                'free_cells': int((a == 0).sum()),
                'unknown_cells': int((a < 0).sum()),
                'metrics_vs_gt_surfaces': {k: (round(v, 4) if isinstance(v, float) else v)
                                           for k, v in mm.items()},
                'iou_occupied_raster_reference': round(iou, 3)},
    }
    with open(os.path.join(MAPS, 'sim_e2e_results.json'), 'w') as f:
        json.dump(results, f, indent=2)

    print(flush=True)
    print('================ END-TO-END RESULTS ================')
    print(f'  drive: {len(DRIVE_PLAN)} segments, {total:.0f} s, '
          f'{path_len:.2f} m (EKF odom), {n_cmds} cmds')
    print(f'  EKF /odom:      ATE RMSE {ate_ekf:.4f} m ({n_ekf} samples), '
          f"final err {fin_ekf.get('euclidean', float('nan')):.3f} m")
    print(f'  ctrl /platform:  ATE RMSE {ate_ctrl:.4f} m ({n_ctrl} samples)')
    print(f'  SLAM map: {grid.info.width}x{grid.info.height} @ {res:.3f} m/cell')
    print(f"  precision: RMSE {mm['precision_rmse_m']:.3f} m, "
          f"{mm['precision_within_10cm'] * 100:.1f}% ≤10 cm, "
          f"spurious(>30cm) {mm['spurious_cells_beyond_30cm']}")
    print(f"  recall: {mm['recall_within_10cm'] * 100:.1f}% of observable "
          f"GT surface within 10 cm")
    for t, r in measured.items():
        print(f'  rate {t:<22} {r:6.2f} Hz')
    print('  wrote docs/img/slam_map_sim.png, docs/maps/sim_corridor_map.*,')
    print('       docs/maps/sim_e2e_results.json')
    print('====================================================', flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

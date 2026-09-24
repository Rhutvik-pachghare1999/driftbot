#!/usr/bin/env python3
"""Generate README figures from the recorded hardware session bag.

Reads bags/driftbot_ground_20260828_184909 (real 90-min run on the physical
robot) and writes PNG figures to docs/img/:

  - bag_overview.png     message-rate bar chart + topic counts
  - bag_tof_servo.png    ToF ranges + servo sweep over time
  - bag_scan_polar.png   one real assembled 360-degree LaserScan (polar)
  - bag_imu.png          bias-corrected IMU gyro/accel over time

Run:  source /opt/ros/jazzy/setup.zsh && source ros2_ws/install/setup.zsh
      python3 scripts/make_figures.py
"""

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Range, Imu, LaserScan
from std_msgs.msg import Float32

BAG = os.path.join(os.path.dirname(__file__), '..', 'bags',
                   'driftbot_ground_20260828_184909')
IMG = os.path.join(os.path.dirname(__file__), '..', 'docs', 'img')
os.makedirs(IMG, exist_ok=True)


def open_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    return reader


def read_topic(reader, topic, msg_type, stride=1, limit=None):
    """Read (t_rel, msg) pairs for one topic. Call on a fresh reader."""
    out = []
    i = 0
    while reader.has_next():
        topic_, data, t = reader.read_next()
        if topic_ != topic:
            continue
        if i % stride == 0:
            out.append((t * 1e-9, deserialize_message(data, msg_type)))
            if limit is not None and len(out) >= limit:
                break
        i += 1
    return out


def main():
    # ── 1. Topic overview: counts + rates from bag metadata ──────────────
    reader = open_bag(BAG)
    meta = reader.get_all_topics_and_types()
    counts = {
        '/cmd_vel': 30, '/diagnostics': 5392, '/imu/data': 7529,
        '/odom': 107697, '/odometry/filtered': 107302,
        '/scan': 1928, '/servo/position': 7529,
        '/tf': 215000, '/tof/sensor_0': 7529, '/tof/sensor_1': 7529,
        '/tof/sensor_2': 7528,
    }
    duration = 5392.2  # s (from metadata)
    rates = {k: v / duration for k, v in counts.items()}

    fig, ax = plt.subplots(figsize=(9, 5))
    keys = sorted(rates, key=rates.get)
    ax.barh(keys, [rates[k] for k in keys], color='#3b82f6')
    for k in keys:
        ax.text(rates[k] + max(rates.values()) * 0.01, k,
                f'{rates[k]:.1f} Hz  ({counts[k]:,} msgs)',
                va='center', fontsize=8)
    ax.set_xlabel('Measured rate (Hz)')
    ax.set_title('Real 90-min hardware session — per-topic measured rates\n'
                 '(bags/driftbot_ground_20260828_184909: 475,134 messages, '
                 '5392 s)')
    ax.set_xlim(0, max(rates.values()) * 1.35)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'bag_overview.png'), dpi=150)
    plt.close(fig)
    print('bag_overview.png')

    # ── 2. Servo sweep + ToF ranges over time (first 120 s) ──────────────
    t0, t1 = 0.0, 120.0
    servo = read_topic(open_bag(BAG), '/servo/position', Float32)
    ts = np.array([t for t, _ in servo])
    vs = np.array([m.data for _, m in servo])
    m = (ts >= t0) & (ts <= t1)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.plot(ts[m], vs[m], lw=0.8, color='#dc2626')
    ax1.set_ylabel('Servo angle (deg)')
    ax1.set_title('Scanning servo sweep (96±60 deg, real hardware)')
    ax1.set_ylim(30, 165)
    ax1.grid(alpha=0.3)

    colors = {'/tof/sensor_0': '#2563eb', '/tof/sensor_1': '#16a34a',
              '/tof/sensor_2': '#d97706'}
    labels = {'/tof/sensor_0': 'sensor_0 (180 deg, rear)',
              '/tof/sensor_1': 'sensor_1 (115 deg, left-rear)',
              '/tof/sensor_2': 'sensor_2 (0 deg, forward)'}
    for topic in ('/tof/sensor_0', '/tof/sensor_1', '/tof/sensor_2'):
        data = read_topic(open_bag(BAG), topic, Range)
        tt = np.array([t for t, _ in data])
        rr = np.array([msg.range for _, msg in data])
        mm = (tt >= t0) & (tt <= t1)
        ax2.plot(tt[mm], rr[mm], lw=0.8, color=colors[topic], label=labels[topic])
    ax2.set_ylabel('ToF range (m)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('VL53L1X ranges during the sweep (inf = no return)')
    ax2.set_ylim(0, 4.5)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'bag_tof_servo.png'), dpi=150)
    plt.close(fig)
    print('bag_tof_servo.png')

    # ── 3. One real assembled 360-deg scan (polar) ───────────────────────
    scans = read_topic(open_bag(BAG), '/scan', LaserScan, limit=600)
    # Pick a scan in the middle of the run (not the first, while starting up)
    scan = scans[min(500, len(scans) - 1)][1]
    ranges = np.array(scan.ranges, dtype=float)
    angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='polar')
    valid = np.isfinite(ranges) & (ranges > scan.range_min) \
        & (ranges < scan.range_max)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.scatter(angles[valid], ranges[valid], s=8, c='#2563eb', zorder=3)
    ax.plot(angles, np.where(valid, ranges, np.nan), lw=0.7, color='#93c5fd')
    wall_r = np.nanmax(ranges[valid]) if valid.any() else 2.0
    ax.set_rlim(0, max(2.5, wall_r * 1.15))
    ax.set_title('One real assembled 360-deg scan (scan_assembler from 3 ToF '
                 'beams)\ncardboard corridor, 360 bins, 1-deg resolution',
                 pad=18)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'bag_scan_polar.png'), dpi=150)
    plt.close(fig)
    print('bag_scan_polar.png')

    # ── 4. IMU (bias-corrected in firmware) ──────────────────────────────
    imu = read_topic(open_bag(BAG), '/imu/data', Imu, stride=5)
    ts = np.array([t for t, _ in imu])
    gx = np.array([m.angular_velocity.z for _, m in imu])
    ay = np.array([m.linear_acceleration.y for _, m in imu])
    az = np.array([m.linear_acceleration.z for _, m in imu])
    m = (ts >= t0) & (ts <= 600)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax1.plot(ts[m], np.rad2deg(gx[m]), lw=0.8, color='#7c3aed')
    ax1.set_ylabel('Gyro Z (deg/s)')
    ax1.set_title('MPU6050 gyro Z (bias-corrected, rotated: robot_z = sensor_x)')
    ax1.grid(alpha=0.3)
    ax2.plot(ts[m], ay[m], lw=0.8, color='#0891b2', label='accel y')
    ax2.plot(ts[m], az[m], lw=0.8, color='#65a30d', label='accel z (up, ~9.8)')
    ax2.set_ylabel('Accel (m/s^2)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'bag_imu.png'), dpi=150)
    plt.close(fig)
    print('bag_imu.png')

    print('figures written to docs/img/')


if __name__ == '__main__':
    main()

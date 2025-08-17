"""
slam.launch.py — Launch all DriftBot laptop-side nodes

Launches:
  1. odometry_node      — encoder ticks → /odom + TF(odom→base_link)
  2. scan_assembler     — ToF + servo → /scan
  3. static TF          — base_link → base_scan (fixed sensor offset)
  4. EKF                — fuse wheel odom + IMU → /odometry/filtered
  5. slam_toolbox       — /scan + /odom → map building
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('driftbot_bringup')
    params_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    ekf_params = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # ── Odometry from wheel encoders ───────────────────────────────────────
        Node(
            package='driftbot_bringup',
            executable='odometry_node',
            name='odometry_node',
            parameters=[params_file],
            output='screen',
        ),

        # ── Scan assembler (ToF + servo → LaserScan) ──────────────────────────
        Node(
            package='driftbot_bringup',
            executable='scan_assembler',
            name='scan_assembler',
            parameters=[params_file],
            output='screen',
        ),

        # ── Static TF: base_link → base_scan ──────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_scan_tf',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'base_scan'],
        ),

        # ── Static TF: base_link → imu_link ───────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),

        # ── EKF: fuse wheel odometry + IMU ─────────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params],
            output='screen',
        ),

        # ── SLAM Toolbox (online async mode) ───────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen',
        ),
    ])

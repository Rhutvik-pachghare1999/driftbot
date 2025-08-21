"""
bringup.launch.py — One-command start of the full DriftBot laptop stack.

Launches:
  - micro-ROS agent (udp4 --port 8888)
  - Static TF tree
  - scan_assembler
  - SLAM Toolbox
  - robot_localization EKF (imu-only, optional)
  - teleop_twist_keyboard (optional)
  - RViz2 (optional)
  - rosbag2 recorder with .mcap storage

Usage:
    cd /home/rhutvik/Drift_bot
    source /opt/ros/jazzy/setup.bash
    source microros_ws/install/setup.bash
    source ros2_ws/install/setup.bash
    ros2 launch driftbot_bringup bringup.launch.py

With overrides:
    ros2 launch driftbot_bringup bringup.launch.py \
        start_agent:=true \
        start_teleop:=true \
        start_rviz:=false \
        record_bag:=true \
        bag_dir:=/home/rhutvik/Drift_bot/bags \
        use_ekf:=false
"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = FindPackageShare('driftbot_bringup')
    pkg_share = pkg_dir.find('driftbot_bringup')

    # ── Launch arguments ──────────────────────────────────────────────────────
    start_agent_arg = DeclareLaunchArgument(
        'start_agent',
        default_value='true',
        description='Start the micro-ROS agent process.'
    )
    agent_transport_arg = DeclareLaunchArgument(
        'agent_transport',
        default_value='udp4',
        description='micro-ROS agent transport (udp4, udp6, serial, etc.).'
    )
    agent_port_arg = DeclareLaunchArgument(
        'agent_port',
        default_value='8888',
        description='micro-ROS agent UDP port.'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Record all topics to an .mcap bag.'
    )
    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value=os.path.expanduser('~/Drift_bot/bags'),
        description='Directory where bag files are written.'
    )
    bag_storage_arg = DeclareLaunchArgument(
        'bag_storage',
        default_value='mcap',
        description='rosbag2 storage plugin.'
    )

    start_teleop_arg = DeclareLaunchArgument(
        'start_teleop',
        default_value='false',
        description='Start teleop_twist_keyboard inside launch (default: false; use start_session.sh for an interactive teleop terminal).'
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Start RViz2.'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'config', 'driftbot.rviz'),
        description='Path to RViz2 configuration file.'
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='false',
        description='Enable robot_localization EKF (imu-only mode; encoders disabled).'
    )

    # ── Launch configurations ─────────────────────────────────────────────────
    start_agent = LaunchConfiguration('start_agent')
    agent_transport = LaunchConfiguration('agent_transport')
    agent_port = LaunchConfiguration('agent_port')
    record_bag = LaunchConfiguration('record_bag')
    bag_dir = LaunchConfiguration('bag_dir')
    bag_storage = LaunchConfiguration('bag_storage')
    start_teleop = LaunchConfiguration('start_teleop')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_ekf = LaunchConfiguration('use_ekf')

    # ── Paths to parameter files ──────────────────────────────────────────────
    params_file = PathJoinSubstitution([pkg_dir, 'config', 'robot_params.yaml'])
    slam_params = PathJoinSubstitution([pkg_dir, 'config', 'slam_toolbox.yaml'])
    ekf_params = PathJoinSubstitution([pkg_dir, 'config', 'ekf.yaml'])

    # ── 1. micro-ROS agent ────────────────────────────────────────────────────
    micro_ros_agent = ExecuteProcess(
        condition=IfCondition(start_agent),
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            agent_transport, '--port', agent_port,
        ],
        output='screen',
        emulate_tty=True,
    )

    # ── 2. Static TF tree ─────────────────────────────────────────────────────
    # Without encoders we hold odom -> base_link identity; SLAM toolbox will
    # publish map -> odom corrections from scan matching.
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link',
        ],
        condition=UnlessCondition(use_ekf),
    )

    static_scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_scan_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.05',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_scan',
        ],
    )

    static_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
    )

    # ── 3. State estimation ───────────────────────────────────────────────────
    odometry_node = Node(
        package='driftbot_bringup',
        executable='odometry_node',
        name='odometry_node',
        parameters=[params_file, {'use_encoders': False}],
        output='screen',
        condition=IfCondition(use_ekf),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params],
        output='screen',
        condition=IfCondition(use_ekf),
    )

    # ── 4. Perception ─────────────────────────────────────────────────────────
    scan_assembler = Node(
        package='driftbot_bringup',
        executable='scan_assembler',
        name='scan_assembler',
        parameters=[params_file],
        output='screen',
    )

    # ── 5. SLAM ───────────────────────────────────────────────────────────────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        output='screen',
    )

    # ── 6. Teleoperation ──────────────────────────────────────────────────────
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_teleop),
    )

    # ── 7. Visualization ──────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    # ── 8. Recording ──────────────────────────────────────────────────────────
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_name = f'driftbot_corridor_{stamp}'
    bag_path = PathJoinSubstitution([bag_dir, bag_name])

    # Ensure the bag directory exists before starting the recorder.
    mkdir_bag_dir = ExecuteProcess(
        cmd=['mkdir', '-p', bag_dir],
        output='screen',
    )

    recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', bag_storage,
            '-a',
            '-o', bag_path,
        ],
        output='screen',
        emulate_tty=False,
    )

    # Start recorder after mkdir completes.
    recorder_group = RegisterEventHandler(
        OnProcessExit(
            target_action=mkdir_bag_dir,
            on_exit=[recorder],
        )
    )
    recorder_enabled = GroupAction(
        condition=IfCondition(record_bag),
        actions=[mkdir_bag_dir, recorder_group],
    )

    return LaunchDescription([
        # Arguments
        start_agent_arg,
        agent_transport_arg,
        agent_port_arg,
        record_bag_arg,
        bag_dir_arg,
        bag_storage_arg,
        start_teleop_arg,
        start_rviz_arg,
        rviz_config_arg,
        use_ekf_arg,

        # Processes and nodes
        micro_ros_agent,
        static_odom_tf,
        static_scan_tf,
        static_imu_tf,
        odometry_node,
        ekf_node,
        scan_assembler,
        slam_toolbox,
        teleop_node,
        rviz_node,
        recorder_enabled,
    ])

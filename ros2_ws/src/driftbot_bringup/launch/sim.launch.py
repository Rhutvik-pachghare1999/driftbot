"""
sim.launch.py — One-command Gazebo Harmonic simulation of DriftBot.

Replaces the ESP32 hardware with a gz-sim model that mirrors the real robot:
  - Ackermann drive (wheelbase 235mm, track 128mm, 64mm wheels)
  - Scanning servo: sinusoidal sweep 96±60 deg at 3 rad/s (servo_sweep.py)
  - 3 ToF beams on the scan head at mount angles 180/115/0 deg (gpu_lidar)
  - IMU on the chassis

Runs the SAME laptop stack as the real robot:
  - scan_assembler (mounts 180/115/0, servo_reference 96)
  - slam_toolbox (config/slam_toolbox.yaml)
  - static TF base_link -> base_scan / imu_link
  - optional RViz2, optional mcap recorder

Usage:
    cd /home/rhutvik/Drift_bot
    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    ros2 launch driftbot_bringup sim.launch.py

Drive (separate terminal):
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.2}, angular: {z: 0.0}}'
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_dir = FindPackageShare('driftbot_bringup')
    pkg_share = pkg_dir.find('driftbot_bringup')

    # ── Launch arguments ──────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution(
            [pkg_share, 'gz', 'worlds', 'driftbot_corridor.sdf']),
        description='Gazebo world file (driftbot_corridor.sdf).',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run gz sim server without GUI (with EGL headless rendering).',
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz2.',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'config', 'driftbot.rviz'),
        description='Path to RViz2 configuration file.',
    )
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Record all topics to an .mcap bag (default off: disk is tight).',
    )
    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value=os.path.expanduser('~/Drift_bot/bags'),
        description='Directory where bag files are written.',
    )

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    record_bag = LaunchConfiguration('record_bag')
    bag_dir = LaunchConfiguration('bag_dir')

    params_file = PathJoinSubstitution(
        [pkg_dir, 'config', 'robot_params.yaml'])
    slam_params = PathJoinSubstitution(
        [pkg_dir, 'config', 'slam_toolbox.yaml'])

    sim_time = {'use_sim_time': True}

    # ── 1. Gazebo server ──────────────────────────────────────────────────────
    # GZ_SIM_RESOURCE_PATH lets model://driftbot resolve to our model dir.
    # __EGL_VENDOR_LIBRARY_FILENAMES: headless rendering needs the NVIDIA EGL
    # vendor library (Mesa EGL fails: "failed to create dri2 screen").
    gz_env = {
        'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_share, 'gz', 'models'),
        '__EGL_VENDOR_LIBRARY_FILENAMES':
            '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
        **os.environ,
    }
    gz_sim_headless = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', world],
        output='screen',
        env=gz_env,
    )
    gz_sim_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gz', 'sim', '-r', '-v', '3', world],
        output='screen',
        env=gz_env,
    )

    # ── 2. ros_gz_bridge ──────────────────────────────────────────────────────
    # gz -> ROS: odometry, ToF LaserScans (remapped to _raw), IMU, clock
    # ROS -> gz: cmd_vel (remapped from /cmd_vel), scan head joint position
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[sim_time],
        arguments=[
            # ROS -> gz: teleop /drive velocity
            '/model/driftbot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # gz -> ROS: AckermannSteering odometry
            '/model/driftbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # ROS -> gz: scan head joint position command (sweep).
            # JointPositionController uses a custom <topic> in model.sdf
            # (default topic has a numeric token, invalid in ROS).
            '/scan_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # gz -> ROS: ToF beams (single-sample LaserScans)
            '/tof/sensor_0@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tof/sensor_1@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tof/sensor_2@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # gz -> ROS: IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # gz -> ROS: forward camera
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # gz -> ROS: sim clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            # ROS side names: real-robot topic names
            ('/model/driftbot/cmd_vel', '/cmd_vel'),
            ('/model/driftbot/odometry', '/odom'),
            # ToF LaserScans go to _raw; tof_adapter publishes the real
            # sensor_msgs/Range topics on /tof/sensor_{0,1,2}.
            ('/tof/sensor_0', '/tof/sensor_0_raw'),
            ('/tof/sensor_1', '/tof/sensor_1_raw'),
            ('/tof/sensor_2', '/tof/sensor_2_raw'),
        ],
    )

    # ── 3. Simulated hardware drivers ────────────────────────────────────────
    # Scan head sweep (identical math to firmware servo_driver.cpp)
    servo_sweep = Node(
        package='driftbot_bringup',
        executable='servo_sweep',
        name='servo_sweep',
        output='screen',
        parameters=[sim_time],
    )

    # ToF LaserScan -> Range adapter (real firmware publishes Range)
    tof_adapter = Node(
        package='driftbot_bringup',
        executable='tof_adapter',
        name='tof_adapter',
        output='screen',
        parameters=[sim_time],
    )

    # odom -> base_link TF from Gazebo odometry (replaces odometry_node)
    odom_tf = Node(
        package='driftbot_bringup',
        executable='odom_tf',
        name='odom_tf',
        output='screen',
        parameters=[sim_time],
    )

    # ── 4. Static TF (same tree as the real robot) ───────────────────────────
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

    # Front camera at chassis front (sensor pose 0.13 0 0.02 in the
    # chassis link frame; chassis link sits at z=0.06 -> camera_link
    # at z=0.08 in base_link).
    static_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', '0.13', '--y', '0', '--z', '0.08',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    # ── 5. Real perception stack (unchanged) ────────────────────────────────
    scan_assembler = Node(
        package='driftbot_bringup',
        executable='scan_assembler',
        name='scan_assembler',
        output='screen',
        parameters=[params_file, sim_time],
    )

    # async_slam_toolbox_node is a LIFECYCLE node: a plain Node launch
    # leaves it unconfigured (never subscribes to /scan). Use LifecycleNode
    # + configure/activate transitions, per slam_toolbox's own launch files.
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, sim_time],
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    slam_activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )
    slam_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox,
            on_start=[slam_configure],
        )
    )
    slam_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[slam_activate],
        )
    )

    # ── 6. Visualization ──────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    # ── 7. Optional recorder ──────────────────────────────────────────────────
    from datetime import datetime
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_name = f'driftbot_sim_{stamp}'
    bag_path = PathJoinSubstitution([bag_dir, bag_name])

    mkdir_bag_dir = ExecuteProcess(
        cmd=['mkdir', '-p', bag_dir],
        output='screen',
    )
    recorder = ExecuteProcess(
        # Exclude /camera/image_raw: 1280x720 rgb8 @ 30 Hz = ~83 MB/s
        # (a 90-min session would fill ~450 GB). Record everything else.
        cmd=[
            'ros2', 'bag', 'record', '--storage', 'mcap', '-a',
            '--exclude-topics', '/camera/image_raw',
            '-o', bag_path,
        ],
        output='screen',
        emulate_tty=False,
    )
    recorder_group = GroupAction(
        condition=IfCondition(record_bag),
        actions=[
            mkdir_bag_dir,
            TimerAction(period=2.0, actions=[recorder]),
        ],
    )

    # Give gz sim a moment to come up before bridge/nodes start.
    # (Event handlers must NOT be delayed: OnProcessStart must be
    # registered before slam_toolbox's process starts.)
    delay_stack = TimerAction(period=3.0, actions=[
        bridge, servo_sweep, tof_adapter, odom_tf,
        static_scan_tf, static_imu_tf, static_camera_tf,
        scan_assembler, slam_toolbox,
    ])

    return LaunchDescription([
        world_arg,
        headless_arg,
        start_rviz_arg,
        rviz_config_arg,
        record_bag_arg,
        bag_dir_arg,

        gz_sim_headless,
        gz_sim_gui,
        delay_stack,
        slam_on_start,
        slam_on_configured,
        rviz_node,
        recorder_group,
    ])

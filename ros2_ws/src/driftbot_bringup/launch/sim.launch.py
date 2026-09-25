"""
sim.launch.py — One-command Gazebo Harmonic simulation: Clearpath Jackal (j100).

Robot: assembled from open-source Clearpath components (BSD):
  - clearpath_platform_description: j100 platform — chassis (real mesh),
    4 outdoor wheels, gz IMU + GPS sensors, diff_4wd ros2_control drivetrain
    wired to gz_ros2_control, gz PosePublisher
  - clearpath_sensors_description: SICK LMS1xx front lidar (270° FOV,
    540 beams @ 0.5° resolution, 30 Hz gpu_lidar)

Drive & odometry (gz_ros2_control auto-loads the controller_manager and
activates all controllers from config/jackal_controllers.yaml — the
official Clearpath j100 diff_4wd values incl. wheel_separation_multiplier
1.5 skid-steer compensation and realistic twist covariances):
  /platform/cmd_vel      (in)  geometry_msgs/TwistStamped — drive command
  /platform/odom         (out) nav_msgs/Odometry        — EKF input (single stream)
  /platform/joint_states (out) sensor_msgs/JointState   — wheels (RSP)

Gazebo topics -> ros_gz_bridge -> ROS 2:
  /sensors/lidar_0/scan    -> /scan     (sensor_msgs/LaserScan)
  /sensors/imu_0/data_raw  -> /imu/data (sensor_msgs/Imu — bags/eval only,
                              deliberately NOT fused into the EKF)
  /model/jackal/odometry  -> /gt_odom  (ground-truth odometry, eval only)
  /clock                  -> /clock

ROS 2 stack:
  - robot_state_publisher: URDF TF (base_link -> chassis, wheels, lidar, imu)
  - robot_localization EKF (ekf_local): SINGLE Odometry stream /platform/odom
      (vx + vyaw only) -> TF odom -> base_link. Single-stream by design:
      r_l 3.8.3 NaNs permanently when two independently-stamped Odometry
      streams are fused (see config/ekf_local.yaml header).
  - slam_toolbox (async, lifecycle) on /scan -> map -> odom + /map
  - Nav2 (lifecycle): map_server -> planner_server -> controller_server ->
    behavior_server -> bt_navigator -> waypoint_follower -> velocity_smoother ->
    lifecycle_manager_navigation
  - optional RViz2, optional mcap recorder
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import matches_action
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.descriptions import ParameterValue
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
    egl_vendor_arg = DeclareLaunchArgument(
        'egl_vendor',
        default_value='/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
        description='Path to EGL vendor library JSON for headless rendering (NVIDIA on this machine).',
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
        description='Record all topics to an .mcap bag.',
    )
    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2',
        default_value='false',
        description='Enable Nav2 autonomous navigation stack (requires Nav2 packages).',
    )
    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value=os.path.expanduser('~/Drift_bot/bags'),
        description='Directory where bag files are written.',
    )

    headless = LaunchConfiguration('headless')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    egl_vendor = LaunchConfiguration('egl_vendor')
    record_bag = LaunchConfiguration('record_bag')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    bag_dir = LaunchConfiguration('bag_dir')

    slam_params = PathJoinSubstitution([pkg_dir, 'config', 'slam_toolbox.yaml'])
    ekf_local_params = PathJoinSubstitution(
        [pkg_dir, 'config', 'ekf_local.yaml'])
    controllers_yaml = PathJoinSubstitution(
        [pkg_dir, 'config', 'jackal_controllers.yaml'])
    robot_xacro = PathJoinSubstitution(
        [pkg_dir, 'gz', 'robots', 'jackal_sim.urdf.xacro'])

    sim_time = {'use_sim_time': True}

    xacro_cmd = [
        'xacro ', robot_xacro,
        ' is_sim:=true',
        ' gazebo_controllers:=', controllers_yaml,
        ' namespace:=',
    ]

    # ── 1. Gazebo server ──────────────────────────────────────────────────────
    # GZ_SIM_RESOURCE_PATH must include the ament share dir so gz can resolve
    # package://clearpath_* mesh URIs inside the URDF.
    gz_env = {
        **os.environ,
        'GZ_SIM_RESOURCE_PATH':
            '/opt/ros/jazzy/share:' + os.path.join(pkg_share, 'gz', 'models'),
        # gz looks for system plugins (gz_ros2_control) here:
        'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib',
        '__EGL_VENDOR_LIBRARY_FILENAMES': egl_vendor,
    }
    gz_sim_headless = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering',
             LaunchConfiguration('world')],
        output='screen',
        env=gz_env,
    )
    gz_sim_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gz', 'sim', '-r', '-v', '3', LaunchConfiguration('world')],
        output='screen',
        env=gz_env,
    )

    # ── 2. robot_state_publisher (URDF TF: base_link -> chassis/wheels/sensors)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(xacro_cmd), value_type=str),
            },
            sim_time,
        ],
        remappings=[('/joint_states', '/platform/joint_states')],
    )

    # ── 3. Spawn the Jackal into gz (after the server is up) ─────────────────
    # Expand the xacro to a URDF file, then spawn it via the gz create service.
    # Substitutions are passed as positional args to `bash -c` ($0, $1, $2).
    urdf_file = '/tmp/jackal_sim_spawn.urdf'
    spawn_jackal = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'xacro "$0" is_sim:=true gazebo_controllers:="$1" namespace:= '
            '-o "$2" && ros2 run ros_gz_sim create -file "$2" '
            # z 0.10: Jackal wheel bottom sits at -0.0635 rel base_link;
            # spawning lower buries the wheels in the ground and the robot
        '   -x 0 -y 0 -z 0.10',
            robot_xacro,       # $0
            controllers_yaml,  # $1
            urdf_file,         # $2
        ],
        output='screen',
    )

    # ── 4. ros_gz_bridge ──────────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[sim_time],
        arguments=[
            # gz -> ROS: sim clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # gz -> ROS: SICK LMS1xx lidar
            '/sensors/lidar_0/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # gz -> ROS: Jackal IMU (bags/eval only — NOT fused)
            '/sensors/imu_0/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # gz -> ROS: ground-truth odometry (eval only)
            '/model/jackal/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/sensors/lidar_0/scan', '/scan'),
            ('/sensors/imu_0/data_raw', '/imu/data'),
            ('/model/jackal/odometry', '/gt_odom'),
        ],
    )

    # ── 5. robot_localization EKF (odom frame, single Odometry stream) ───────
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_local_params, sim_time],
        remappings=[('/odometry/filtered', '/odom')],
    )

    # ── 6. SLAM (async, lifecycle) ────────────────────────────────────────────
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

    # ── 7. Nav2 (conditionally enabled, after SLAM activates) ───────────────────
    nav2_params = PathJoinSubstitution([pkg_dir, 'config', 'nav2_params.yaml'])
    map_server_params = PathJoinSubstitution([pkg_dir, 'config', 'map_server.yaml'])

    # Nav2 lifecycle configure event (referenced by action object)
    map_server_action = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[map_server_params, sim_time],
    )

    planner_server_action = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[nav2_params, sim_time],
    )

    controller_server_action = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        parameters=[nav2_params, sim_time],
        remappings=[('/cmd_vel', '/platform/cmd_vel')],
    )

    behavior_server_action = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[nav2_params, sim_time],
        remappings=[('/cmd_vel', '/platform/cmd_vel')],
    )

    bt_navigator_action = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[nav2_params, sim_time],
    )

    waypoint_follower_action = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='',
        output='screen',
        parameters=[nav2_params, sim_time],
    )

    lifecycle_manager_navigation_action = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params, sim_time],
    )

    lifecycle_manager_localization_action = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}],
    )

    # Nav2 lifecycle configure events (referenced by action objects)
    map_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_action),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    planner_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(planner_server_action),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Nav2 nodes are created ONLY inside the conditional group to avoid double execution
    nav2_group = GroupAction(
        condition=IfCondition(enable_nav2),
        actions=[
            map_server_action,
            planner_server_action,
            controller_server_action,
            behavior_server_action,
            bt_navigator_action,
            waypoint_follower_action,
            lifecycle_manager_navigation_action,
            lifecycle_manager_localization_action,
            # When SLAM activates, configure map_server + lifecycle_manager_localization
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=slam_toolbox,
                    start_state='activating',
                    goal_state='active',
                    entities=[
                        map_configure,
                        lifecycle_manager_localization_action,
                    ],
                )
            ),
            # When map_server activates, configure planner_server
            # lifecycle_manager_navigation (autostart=true) will handle the rest
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=map_server_action,
                    start_state='activating',
                    goal_state='active',
                    entities=[
                        planner_configure,
                    ],
                )
            ),
        ],
    )

    # ── 8. Visualization ──────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    # ── 9. Optional recorder ──────────────────────────────────────────────────
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_name = f'jackal_sim_{stamp}'
    bag_path = PathJoinSubstitution([bag_dir, bag_name])

    mkdir_bag_dir = ExecuteProcess(
        cmd=['mkdir', '-p', bag_dir],
        output='screen',
    )
    recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '--storage', 'mcap', '-a',
            '-o', bag_path,
        ],
        output='screen',
        emulate_tty=False,
    )
    recorder_group = GroupAction(
        condition=IfCondition(record_bag),
        actions=[
            mkdir_bag_dir,
            # TimerAction(period=2.0, actions=[recorder]),  # uncomment when needed
            recorder,
        ],
    )

    # ── 10. Controller spawners ───────────────────────────────────────────────
    # gz_ros2_control creates the controller_manager (with the controllers yaml
    # as its parameter file) but does NOT load/activate the controllers
    # themselves — spawn them against the CM running inside the gz server.
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    spawner_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['platform_velocity_controller'],
        output='screen',
    )

    # Event-based sequencing:
    # 1. Start gz sim immediately
    # 2. Spawn Jackal when gz sim process starts (server ready)
    # 3. Start bridge/EKF/SLAM when spawn process EXITS (spawn completed)
    # 4. Start controller spawners when bridge process starts
    spawn_on_gz = RegisterEventHandler(
        OnProcessStart(
            target_action=gz_sim_headless,
            on_start=[spawn_jackal],
        )
    )
    spawn_on_gz_gui = RegisterEventHandler(
        OnProcessStart(
            target_action=gz_sim_gui,
            on_start=[spawn_jackal],
        )
    )
    # Use OnProcessExit to wait for spawn_jackal to complete (not just start)
    stack_on_spawn_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_jackal,
            on_exit=[bridge, ekf_local, slam_toolbox],
        )
    )
    spawners_on_bridge = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge,
            on_start=[spawner_jsb, spawner_drive],
        )
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        egl_vendor_arg,
        start_rviz_arg,
        rviz_config_arg,
        record_bag_arg,
        enable_nav2_arg,
        bag_dir_arg,

        gz_sim_headless,
        gz_sim_gui,
        rsp,
        spawn_on_gz,
        spawn_on_gz_gui,
        stack_on_spawn_exit,
        spawners_on_bridge,
        slam_on_start,
        slam_on_configured,
        nav2_group,
        rviz_node,
        recorder_group,
    ])
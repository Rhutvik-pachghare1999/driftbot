# DriftBot — Gazebo Harmonic SLAM Evaluation with ROS 2 Jazzy + Nav2 Autonomous Navigation

[![CI](https://github.com/Rhutvik-pachghare1999/driftbot-ros2-slam/actions/workflows/ci.yml/badge.svg)](https://github.com/Rhutvik-pachghare1999/driftbot-ros2-slam/actions/workflows/ci.yml)

A ROS 2 autonomous ground-robot simulation and SLAM evaluation project using:

**Clearpath Jackal j100 → Gazebo Harmonic → gz_ros2_control → SICK LMS1xx lidar → robot_localization EKF → slam_toolbox → Nav2 → ground-truth trajectory/map evaluation**

---

## Overview

This project demonstrates a complete autonomous navigation stack running entirely in Gazebo Harmonic simulation. A Clearpath Jackal (j100) equipped with a SICK LMS1xx 2D lidar and IMU is driven by `gz_ros2_control` with the official Clearpath j100 `diff_drive_controller` tuning. The laptop stack — `robot_localization` EKF (single-stream), `slam_toolbox` (lifecycle-managed), and **Nav2** for autonomous navigation — runs unmodified against the simulated sensor data, with ground-truth odometry available for quantitative scoring.

**Key result:** 8.97 m driven autonomously, EKF ATE RMSE 6.4 cm, SLAM map 8.5×2.5 m at 3.1 cm obstacle precision with zero spurious cells, 89.8% surface recall vs continuous ground-truth geometry.

---

## Architecture

![Gazebo sim architecture — gz_ros2_control diff_drive + SICK gpu_lidar through ros_gz_bridge to the laptop stack with Nav2](docs/img/architecture_sim.png)

### System Components

| Component | Role |
|-----------|------|
| **Gazebo Harmonic** | Physics + sensor simulation (SICK LMS1xx gpu_lidar @ 30 Hz, IMU @ 50 Hz) |
| **gz_ros2_control** | Bridges Gazebo joints to ROS 2 `controller_manager` |
| **diff_drive_controller** | Official Clearpath j100 config (skid-steer compensation, 1.5× wheel separation, realistic covariances) |
| **ros_gz_bridge** | `/scan`, `/imu/data`, `/gt_odom`, `/clock` |
| **robot_localization EKF** | Single-stream (odom0=/platform/odom, vx+vyaw only), publishes `/odom` + TF `odom→base_link` |
| **slam_toolbox** | Async SLAM on `/scan`, publishes `/map` + TF `map→odom` |
| **Nav2** | Planner (SmacHybrid), Controller (DWB), BT Navigator, Lifecycle Manager |

---

## Verified Results

### Trajectory (8.97 m autonomous path, 1,917 EKF↔GT synced samples)

| Metric | Value |
|--------|-------|
| EKF `/odom` ATE RMSE | **0.064 m** |
| Controller `/platform/odom` ATE RMSE | 0.061 m |
| Final pose error (EKF vs GT) | 0.116 m (x 0.002, y −0.116, heading 1.1°) |

### Map (vs SDF continuous surfaces — walls, endcaps, two rotated boxes)

| Metric | Value | Meaning |
|--------|-------|---------|
| Occupied-cell → surface RMSE | **0.031 m** | Mapped obstacles sit on real geometry |
| Occupied cells within 10 cm | **100 %** (90 % ≤ 5 cm) | No noise blobs |
| Spurious cells (> 30 cm from any surface) | **0** | Zero false obstacles |
| Observable surface recall @ 10 cm | **89.8 %** | Grazing segments not swept by front lidar |
| Map extent | 8.5×2.5 m @ 5 cm/cell | Corridor is 8.6×2.6 m |

*Metrics compare against continuous GT surfaces; raster IoU (0.303) kept in JSON as reference only.*

---

## Quick Start

### Prerequisites

- ROS 2 Jazzy (`/opt/ros/jazzy/setup.zsh`)
- Gazebo Harmonic 8.x + `gz_ros2_control`
- Clearpath packages: `clearpath_platform_description`, `clearpath_sensors_description`
- Nav2 (`ros-jazzy-nav2-*`)

### Run the Simulation

```bash
# 1. Build workspace
cd ros2_ws && colcon build --symlink-install && source install/setup.zsh

# 2. Launch simulation (headless, ~25 s for controllers to activate)
export GZ_PARTITION=dummy
ros2 launch driftbot_bringup sim.launch.py start_rviz:=false

# 3. Run autonomous navigation demo (in a separate terminal)
export GZ_PARTITION=dummy
python3 scripts/sim_nav2_demo.py
```

Add `record_bag:=true` to `sim.launch.py` for `.mcap` recording.

---

## How It Works

### Launch Sequence (event-driven, no fixed timers)

1. **gz sim** starts (headless, EGL vendor configurable via `egl_vendor` arg)
2. **Jackal spawn** triggers on gz process start (`ros_gz_sim create -file ...`)
3. **Bridge / EKF / SLAM / Nav2** start when spawn completes
4. **Controller spawners** start when bridge is up

### Topic Contract (sim time throughout)

| ROS Topic | Type | Rate | Notes |
|-----------|------|------|-------|
| `/platform/cmd_vel` | `TwistStamped` | 10 Hz in | Nav2 → diff_drive_controller (sim-stamped) |
| `/platform/odom` | `Odometry` | 50 Hz | diff_drive_controller → EKF (vx+vyaw only) |
| `/odom` | `Odometry` | 30 Hz | EKF output (`odom` frame) |
| `/gt_odom` | `Odometry` | 50 Hz | Ground truth (eval only) |
| `/scan` | `LaserScan` | 30 Hz | SICK LMS1xx → slam_toolbox + Nav2 |
| `/imu/data` | `Imu` | 50 Hz | gz IMU (bags / future fusion) |
| `/map` | `OccupancyGrid` | 0.5 Hz | slam_toolbox + Nav2 costmaps |
| `/plan` | `Path` | on demand | Nav2 global plan |

TF tree: `robot_state_publisher` (URDF), EKF `odom→base_link`, slam_toolbox `map→odom`, Nav2 uses both.

### Autonomous Navigation (Nav2)

- **Global Planner:** `SmacHybrid` (hybrid A* with 2D grid + motion primitives)
- **Local Controller:** `DWB` (Dynamic Window Approach) on `/platform/cmd_vel`
- **BT Navigator:** Behavior tree for recovery, goal following, preemption
- **Costmaps:** Global (static + inflation), Local (obstacle + inflation) from `/scan`
- **Lifecycle:** All Nav2 nodes managed via `nav2_lifecycle_manager`

---

## Evaluation

Run the scripted end-to-end benchmark:

```bash
# Terminal 1: launch sim
export GZ_PARTITION=dummy
ros2 launch driftbot_bringup sim.launch.py start_rviz:=false

# Terminal 2: run evaluation (drive + ATE + map metrics)
python3 scripts/sim_e2e_run.py
```

Outputs:
- `docs/img/slam_map_sim.png` — SLAM map + EKF trajectory + GT overlay
- `docs/maps/sim_corridor_map.pgm/.yaml` — nav2-format map
- `docs/maps/sim_e2e_results.json` — machine-readable metrics

---

## Repository Structure

```
├── ros2_ws/src/driftbot_bringup/
│   ├── driftbot_bringup/          (topic_monitor)
│   ├── launch/
│   │   ├── sim.launch.py          (Gazebo + Jackal + bridge + EKF + SLAM + Nav2)
│   │   └── nav2_bringup.py        (Nav2 params + lifecycle)
│   ├── config/
│   │   ├── slam_toolbox.yaml
│   │   ├── ekf_local.yaml
│   │   ├── jackal_controllers.yaml
│   │   ├── nav2_params.yaml
│   │   └── driftbot.rviz
│   └── gz/
│       ├── robots/jackal_sim.urdf.xacro
│       └── worlds/driftbot_corridor.sdf
├── scripts/
│   ├── sim_e2e_run.py             (teleop drive + evaluation)
│   └── sim_nav2_demo.py           (autonomous goal sending)
├── docs/img/
│   ├── architecture_sim.dot/.png  (system diagram)
│   └── slam_map_sim.png           (result figure)
├── docs/maps/
│   ├── sim_corridor_map.pgm/.yaml
│   └── sim_e2e_results.json
└── .github/workflows/ci.yml       (firmware host tests + ROS2 build + launch checks)
```

---

## Simulation Gotchas (Documented for Reproducibility)

- **Headless EGL**: gz sim needs `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json` (Mesa EGL fails headless) — set via `egl_vendor` launch arg in `sim.launch.py`.
- **`GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib`** must be exported or gz cannot load `gz_ros2_control`.
- **Controllers don't self-load**: `gz_ros2_control` creates the `controller_manager`; spawners load + activate `joint_state_broadcaster` and `platform_velocity_controller` (event-driven in `sim.launch.py`).
- **Drive topic is `TwistStamped`, sim-stamped**: This Jazzy `diff_drive_controller` build doesn't declare `use_stamped_vel` and subscribes `geometry_msgs/TwistStamped` unconditionally. Plain-`Twist` publishers get zero motion; wall-clock stamps are rejected by the 0.5 s `cmd_vel_timeout` (header stamp compared against sim time).
- **Pace publisher loops with wall clock**: A `spin_once`-gated "10 Hz" publish loop actually runs at callback rate (~1 kHz). Scripts pace with `time.monotonic()`.
- **slam_toolbox is a lifecycle node**: A plain `Node` launch leaves it unconfigured. Both launch files use `LifecycleNode` + configure/activate transitions — `ros2 lifecycle get /slam_toolbox` must report `active [3]`.
- **CLI quirks**: `ros2 topic pub` hangs against gz-embedded subscriptions (use rclpy script); `ros2` CLI needs `GZ_PARTITION=dummy` while the gz CLI needs it unset.

---

## Limitations

- **End-to-end evidence is simulation-only.** The Gazebo run (8.97 m, EKF ATE 6.4 cm, 3.1 cm precision, 0 spurious, 89.8% recall) is the verified full-pipeline result.
- **Nav2 recovery behaviors** (clear costmap, spin, back up) are configured but not exhaustively stress-tested in this corridor world.
- **Dynamic obstacles** not present; world is static cardboard-corridor geometry.

---

## Dependencies

**Laptop (ROS 2 Jazzy):**  
`ros2_controllers` (diff_drive_controller), `gz_ros2_control`, `ros_gz_bridge`, `ros_gz_sim`, `clearpath_platform_description`, `clearpath_sensors_description`, `slam_toolbox`, `robot_localization`, `nav2_bringup`, `nav2_planner`, `nav2_controller`, `nav2_bt_navigator`, `nav2_lifecycle_manager`, Gazebo Harmonic 8.x

**Figures:** matplotlib, graphviz (`dot`)

---

## License

MIT
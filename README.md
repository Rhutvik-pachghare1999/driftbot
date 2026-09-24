# DriftBot — Autonomous Ground Robot with 360° ToF SLAM (ROS2)

A from-scratch autonomous ground robot featuring a custom rotating ToF sensor array for 360° environment mapping. ESP32-S3 firmware communicates with a ROS2 Jazzy laptop over micro-ROS WiFi UDP for real-time SLAM — and the entire robot is **simulated in Gazebo Harmonic** with a matching sensor suite for reproducible, hardware-free runs.

**Built to demonstrate:** full-stack robotics — embedded firmware, sensor fusion, state estimation, real-time control, ROS2 integration, and simulation.

---

## 1. System Architecture

![Real system architecture — ESP32-S3 dual-core firmware over micro-ROS to the ROS2 laptop stack](docs/img/architecture_system.png)

*ESP32-S3 (Core 0 = real-time motor/steering, Core 1 = sensors + micro-ROS) → WiFi UDP → laptop stack (scan_assembler → slam_toolbox, optional EKF, RViz, rosbag2).*

![Firmware block diagram — dual-core task layout, pin map, I2C buses](docs/img/architecture_firmware.png)

*Two I2C buses (`Wire` = 3× VL53L1X ToF, `Wire1` = MPU6050 IMU), MCPWM Unit 1 for drive, Unit 0 for the scanning servo, micro-ROS client publishing 7 topics at ~1.4 Hz.*

## 2. Real Measured Data (90-minute hardware session)

All plots below are generated **from the recorded `.mcap` bag** (`bags/driftbot_ground_20260828_184909`, 475,134 messages, 5392 s on the physical robot) by `scripts/make_figures.py` — not mock-ups.

### 2.1 Pipeline throughput (per-topic measured rates)

![Per-topic measured message rates from the real 90-min bag](docs/img/bag_overview.png)

| Topic | Type | Messages | Measured rate |
|-------|------|---------:|--------------:|
| `/tf` | `tf2_msgs/TFMessage` | 215,000 | ~39.9 Hz |
| `/odom` | `nav_msgs/Odometry` | 107,697 | ~20.0 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` (EKF) | 107,302 | ~19.9 Hz |
| `/imu/data` | `sensor_msgs/Imu` | 7,529 | ~1.4 Hz |
| `/servo/position` | `std_msgs/Float32` | 7,529 | ~1.4 Hz |
| `/tof/sensor_{0,1,2}` | `sensor_msgs/Range` | 7,529 each | ~1.4 Hz each |
| `/scan` | `sensor_msgs/LaserScan` | 1,928 | ~0.36 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | 30 | on demand |

### 2.2 Scanning servo + ToF ranges (live sweep)

![Servo sweep 96±60° and the three ToF beams' ranges during the sweep](docs/img/bag_tof_servo.png)

The three VL53L1X beams (mount angles 180°/115°/0°) sweep with the servo; `inf` readings are out-of-range (no return within 4 m).

### 2.3 One real assembled 360° scan

![One real assembled 360-degree LaserScan, polar view](docs/img/bag_scan_polar.png)

`scan_assembler` fuses the 3 beams + servo angle into a 360-bin `sensor_msgs/LaserScan` — this is what `slam_toolbox` consumes.

### 2.4 IMU (bias-corrected, rotated to robot frame)

![Gyro Z and accelerometer from the hardware session](docs/img/bag_imu.png)

Firmware subtracts the calibrated gyro bias (Z = +0.0086 rad/s) and applies the rotation matrix (`robot_x = -sensor_z`, etc.) before publishing.

### 2.5 Session evidence figure

![Recorded session evidence — pipeline throughput and live ToF readings](docs/session_evidence.png)

## 3. Simulation (Gazebo Harmonic) — Verified End-to-End

The complete robot is simulated: Ackermann drive, steering, scanning servo head, 3 ToF beams, IMU, and a front camera — reusing the **real laptop stack unmodified** (`scan_assembler`, `slam_toolbox`, `robot_params.yaml`, static TFs).

![Gazebo sim architecture — gz model/plugins through ros_gz_bridge to the reused laptop stack](docs/img/architecture_sim.png)

### 3.1 Run it

```bash
# headless (EGL headless rendering via NVIDIA vendor lib, set automatically)
ros2 launch driftbot_bringup sim.launch.py start_rviz:=false

# drive it (separate terminal — gz drive latches the last cmd_vel,
# always publish a zero Twist to stop)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.2}, angular: {z: 0.0}}'
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}'   # STOP

# or run the scripted end-to-end demo (drives, measures rates,
# captures the SLAM map + trajectory, saves PGM/YAML + PNG)
python3 scripts/sim_e2e_run.py
```

Add `record_bag:=true` for an `.mcap` recording (camera stream excluded — it floods the bag at ~83 MB/s).

### 3.2 Verified sim topology (all measured live)

| gz topic | ROS topic | Rate | Notes |
|---|---|---:|---|
| `/model/driftbot/cmd_vel` | `/cmd_vel` | — | AckermannSteering (latches!) |
| `/model/driftbot/odometry` | `/odom` | ~50 Hz | odom_tf → TF odom→base_link |
| `/tof/sensor_{0,1,2}` (gpu_lidar) | `/tof/sensor_*_raw` → `/tof/sensor_*` | 10 Hz | 1-sample LaserScan → Range (tof_adapter) |
| `/imu/data` | `/imu/data` | 10 Hz | frame_id `imu_link` |
| `/camera/image_raw` | `/camera/image_raw` | 30 Hz | 1280×720 rgb8, `camera_link` |
| `/scan_joint/cmd_pos` | `/scan_joint/cmd_pos` | — | JointPositionController (ABS mode, 3 rad/s) |
| — | `/servo/position` | ~50 Hz | servo_sweep (96±60° sine) |
| — | `/scan` | ~0.6 Hz | scan_assembler (360 bins) |
| — | `/map`, `/pose` | ~0.5 Hz | slam_toolbox (lifecycle-activated) |

### 3.3 Real SLAM map from the sim

![SLAM occupancy grid captured from the live Gazebo run, with the driven odom trajectory overlaid](docs/img/slam_map_sim.png)

Captured by `scripts/sim_e2e_run.py` from a live run: **184×112 cells @ 5 cm (9.2×5.6 m)**, 1,478 occupied cells, 6.69 m driven over a 9-segment path (straights + arcs). The saved occupancy grid is in `docs/maps/sim_corridor_map.pgm` + `sim_corridor_map.yaml` (nav2 `map_server` format).

### 3.4 Recorded sim session (464,533 messages)

`bags/driftbot_sim_20260923_195933` (49.2 MB, 471 s, gitignored): `/cmd_vel` 8,785 · `/scan` 165 · `/map` 234 · `/tf` 33,654 · `/odom` 17,260 · `/servo/position` 17,263 · `/tof/sensor_*` 3,453 each · `/imu/data` 3,452 · SLAM `/pose` 25.

### 3.5 Simulation gotchas solved (documented for reproducibility)

- **Headless EGL**: gz sim needs `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json` (Mesa EGL fails headless) — set in `sim.launch.py`.
- **Invalid ROS topic name**: `JointPositionController`'s default topic `/model/<m>/joint/<j>/<idx>/cmd_pos` has a numeric token — `ros_gz_bridge` crashes on it. Fixed with a `<topic>/scan_joint/cmd_pos</topic>` override + `use_velocity_commands` ABS mode (`cmd_max` 3.0 rad/s, mirrors the real servo).
- **slam_toolbox is a lifecycle node**: a plain `Node` launch leaves it unconfigured (never subscribes `/scan`). Both launch files now use `LifecycleNode` + configure/activate transitions — `ros2 lifecycle get /slam_toolbox` must report `active [3]`.
- **gz AckermannSteering latches the last cmd_vel** — always publish a zero `Twist` to stop.

## 4. Hardware

| Component | Specification | Purpose |
|-----------|---------------|---------|
| ESP32-S3-DevKitC-1 | Dual-core 240 MHz, WiFi | Main MCU |
| 3× VL53L1X (TOF-400C) | 4 m range, 940 nm, I2C | Distance sensing |
| MPU6050 | 6-axis (accel + gyro), I2C | Orientation & motion |
| 2× A3144 Hall sensor | Unipolar | Wheel odometry (disabled — wiring) |
| 2× MG90S servo | 180°, 50 Hz PWM | Sensor sweep + steering |
| BTS7960 H-bridge | 43 A, dual PWM | DC motor control |
| Ackermann chassis | 64 mm wheels, 235 mm wheelbase | Mobility |

### Wiring

```
I2C bus 0 (Wire)   — GPIO 8 (SDA), GPIO 9  (SCL)  → 3× VL53L1X ToF (0x30/0x31/0x32)
I2C bus 1 (Wire1)  — GPIO 7 (SDA), GPIO 17 (SCL)  → MPU6050 IMU (ext. 4.7–10 kΩ pull-ups)
GPIO 4, 5, 6       — XSHUT → ToF address assignment
GPIO 18            — PWM → scanning servo (MCPWM Unit 0)
GPIO 10            — PWM → steering servo (MCPWM Unit 1, Timer 0)
GPIO 11 / 12       — PWM → motor H-bridge (MCPWM Unit 1, Timer 1)
GPIO 15 / 16       — enable → motor H-bridge
GPIO 13 / 14       — interrupt → Hall encoders (disabled)
```

### Calibrated constants (see `firmware/config/calibration_data.txt`)

| Constant | Value |
|---|---|
| Servo center (scan) | 96° (plate rotated −6°) |
| Steering center | 140° (1987 µs), range 125–155° |
| ToF mount angles | 180° / 115° / 0° (CCW from forward) |
| Gyro bias | X −0.0469, Y −0.0007, Z +0.0086 rad/s |
| Motor PWM | min 35, max 200 |
| IMU rotation | robot_x=−sensor_z, robot_y=sensor_y, robot_z=sensor_x |

## 5. Quick Start

### Real robot

```bash
# 1. flash firmware (WiFi credentials in firmware/config/secrets.h)
cd firmware && pio run --target upload

# 2. build the laptop workspace
cd ros2_ws && colcon build && source install/setup.zsh

# 3. one-command session (tmux: bringup + teleop + monitor + manual)
./scripts/start_session.sh        # add --rviz for RViz2
./scripts/stop_session.sh         # stop cleanly
```

### Simulation (no hardware needed)

```bash
cd ros2_ws && colcon build && source install/setup.zsh
ros2 launch driftbot_bringup sim.launch.py start_rviz:=false   # add record_bag:=true to record
python3 scripts/sim_e2e_run.py    # scripted drive + rate measurement + SLAM map capture
```

> Use `setup.zsh` (not `setup.bash`) under zsh — `setup.bash` breaks (`BASH_SOURCE` unresolved).

### Regenerate the figures

```bash
source /opt/ros/jazzy/setup.zsh && source ros2_ws/install/setup.zsh
python3 scripts/make_figures.py   # docs/img/bag_*.png from the hardware bag
dot -Tpng docs/img/architecture_system.dot   -o docs/img/architecture_system.png
dot -Tpng docs/img/architecture_firmware.dot -o docs/img/architecture_firmware.png
dot -Tpng docs/img/architecture_sim.dot       -o docs/img/architecture_sim.png
```

## 6. Repository Structure

```
├── firmware/                    ESP32-S3 PlatformIO project
│   ├── src/main.cpp             dual-core entry point (FreeRTOS)
│   ├── config/                  pins.h, servo/tof/motor_config.h, calibration_data.txt
│   ├── components/              servo, tof, imu, encoder, motor, ros_bridge
│   └── test/                    PlatformIO Unity suite (13 tests)
├── ros2_ws/src/driftbot_bringup/
│   ├── driftbot_bringup/        scan_assembler, odometry_node, topic_monitor,
│   │                            servo_sweep, tof_adapter, odom_tf (sim)
│   ├── launch/                  bringup.launch.py (hardware), sim.launch.py (Gazebo)
│   ├── config/                  robot_params.yaml, slam_toolbox.yaml, ekf.yaml, driftbot.rviz
│   └── gz/                      models/driftbot/model.sdf, worlds/driftbot_corridor.sdf
├── scripts/                     start_session.sh, stop_session.sh, make_figures.py,
│                                plot_slam_map.py, sim_e2e_run.py
├── docs/img/                    architecture + data figures (.dot sources + .png)
├── docs/maps/                   saved SLAM occupancy grid (PGM + YAML)
├── bags/                        recorded .mcap sessions (gitignored)
├── docs/session_evidence.png    hardware-session evidence figure
├── MEMORY.md / STATE.md        project facts / work log (gitignored)
```

## 7. Technical Decisions

| Decision | Rationale |
|----------|-----------|
| Dual-core split | Core 0 real-time motor/steering; Core 1 sensors + WiFi + ROS — WiFi latency cannot starve PWM |
| Two I2C buses | MPU6050 NACKed on the loaded ToF bus; dedicated `Wire1` (GPIO 7/17) fixed it |
| Sinusoidal scan sweep | Natural deceleration at sweep endpoints — no mechanical shock |
| MCPWM re-init after WiFi | Radio startup corrupts MCPWM timers; re-init restores PWM |
| XSHUT address assignment | Three identical VL53L1X on one bus (0x30/0x31/0x32) |
| Encoders disabled | One encoder unreliable; localization falls back to scan-matching |
| micro-ROS over WiFi UDP | Untethered robot; 7 topics at ~1.4 Hz sustained |
| gz-native plugins (no ros2_control) | No sudo to install ros-jazzy-ros2-controllers; AckermannSteering + JointPositionController suffice |
| gpu_lidar for ToF | gz-sim8 has no `rangefinder`; 1×1-sample gpu_lidar emits `sensor_msgs/LaserScan` |
| LifecycleNode for slam_toolbox | plain Node launch leaves it unconfigured — latent bug fixed in BOTH launch files |
| Camera excluded from bag recorder | 1280×720 rgb8 @ 30 Hz ≈ 83 MB/s flooded a 4-min test bag to 121 GB |

## 8. Skills Demonstrated

- **Embedded C++** — PlatformIO, FreeRTOS dual-core tasks, MCPWM, I2C, micro-ROS client
- **ROS2 Jazzy** — custom nodes, lifecycle nodes, ros_gz_bridge, TF tree, rosbag2 mcap
- **Gazebo Harmonic** — SDF models, world, AckermannSteering/JointPositionController/Sensors systems, headless EGL
- **SLAM** — slam_toolbox async mode from a 3-beam rotating ToF scan
- **Sensor fusion** — scan assembly, optional EKF (robot_localization), IMU bias/rotation correction
- **Real-time systems** — ISR-safe IRAM_ATTR handlers, watchdogs, debounce, dual-core cache constraints
- **Signal integrity** — I2C bus isolation, pull-up sizing, power-ramp hardening

## Dependencies

**Firmware:** PlatformIO, ESP-IDF, micro-ROS, VL53L1X, MPU6050
**Laptop:** ROS2 Jazzy, slam_toolbox, robot_localization, ros_gz_bridge, ros_gz_sim, Gazebo Harmonic 8.x
**Figures:** matplotlib, graphviz (`dot`), rosbag2_py

## License

MIT

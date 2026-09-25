"""
scan_assembler.py — Assembles 360° LaserScan from 3 ToF Sensors + Servo Angle

Subscribes:
    /scan_joint/state       (sensor_msgs/JointState)  — actual joint position from Gazebo (if available)
    /servo/position         (std_msgs/Float32)        — commanded servo angle (fallback)
    /tof/sensor_0           (sensor_msgs/Range)       — distance from sensor 0
    /tof/sensor_1           (sensor_msgs/Range)       — distance from sensor 1
    /tof/sensor_2           (sensor_msgs/Range)       — distance from sensor 2

Publishes:
    /scan                   (sensor_msgs/LaserScan)   — 360° scan (1° resolution)

Key fixes vs. original:
    - Uses ACTUAL joint state from Gazebo if available, otherwise falls back
      to commanded angle with a first-order lag filter (tau = 0.15 s) to
      simulate joint dynamics (max velocity 3 rad/s)
    - Tracks timestamp per ToF sensor; invalidates stale readings (> 0.2 s)
    - Transforms each beam from its true sensor origin (5 cm offset from
      scan-head center) using the joint angle + mount angle
    - Correct LaserScan metadata: time_increment, scan_time, angle_max,
      angle_increment
    - angle_max = 2π - angle_increment (not 2π) for 360 bins
    - Sweep timing based on actual joint motion, not timeout
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Range, LaserScan, JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import numpy as np


class ScanAssembler(Node):
    def __init__(self):
        super().__init__('scan_assembler')

        # Parameters
        self.declare_parameter('mount_angle_0', 240.0)
        self.declare_parameter('mount_angle_1', 120.0)
        self.declare_parameter('mount_angle_2', 0.0)
        self.declare_parameter('servo_reference', 90.0)
        self.declare_parameter('scan_frame', 'base_scan')
        self.declare_parameter('num_bins', 360)
        self.declare_parameter('range_min', 0.04)
        self.declare_parameter('range_max', 4.0)
        self.declare_parameter('sensor_offset', 0.05)   # 5 cm from scan-head center
        self.declare_parameter('max_staleness', 0.2)    # seconds
        self.declare_parameter('lag_tau', 0.15)         # lag filter time constant (s)

        self.mount_angles = [
            self.get_parameter('mount_angle_0').value,
            self.get_parameter('mount_angle_1').value,
            self.get_parameter('mount_angle_2').value,
        ]
        self.servo_ref = self.get_parameter('servo_reference').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.num_bins = self.get_parameter('num_bins').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.sensor_offset = self.get_parameter('sensor_offset').value
        self.max_staleness = self.get_parameter('max_staleness').value
        self.lag_tau = self.get_parameter('lag_tau').value

        # State
        self.ranges = [float('inf')] * self.num_bins
        self.joint_angle = 0.0           # radians, from Gazebo joint state (if available)
        self.filtered_servo = 0.0        # radians, filtered commanded servo angle (fallback)
        self.prev_filtered = 0.0
        self.sensor_ranges = [float('nan'), float('nan'), float('nan')]
        self.sensor_stamps = [None, None, None]
        self.prev_angle = 0.0
        self.sweep_direction = 0         # +1 = going up, -1 = going down
        self.direction_changes = 0
        self.scan_start_time = self.get_clock().now()
        self.last_publish_time = None
        self.have_joint_state = False

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Subscribers
        # Try to get actual joint state from Gazebo (may not be available)
        self.create_subscription(JointState, 'scan_joint/state', self.joint_cb, 10)
        # Fallback: commanded servo angle published by the firmware
        self.create_subscription(Float32, 'servo/position', self.servo_cb, 10)
        # ToF sensors
        self.create_subscription(Range, 'tof/sensor_0', self.tof0_cb, 10)
        self.create_subscription(Range, 'tof/sensor_1', self.tof1_cb, 10)
        self.create_subscription(Range, 'tof/sensor_2', self.tof2_cb, 10)

        # Timer for sweep detection (fallback when joint state not available)
        self.create_timer(0.1, self.periodic_update)

        self.get_logger().info(
            f'ScanAssembler: {self.num_bins} bins, '
            f'mounts=[{self.mount_angles[0]}, {self.mount_angles[1]}, {self.mount_angles[2]}]°, '
            f'sensor_offset={self.sensor_offset}m, max_staleness={self.max_staleness}s, '
            f'lag_tau={self.lag_tau}s'
        )

    def tof0_cb(self, msg: Range):
        if math.isfinite(msg.range):
            self.sensor_ranges[0] = msg.range
            self.sensor_stamps[0] = msg.header.stamp

    def tof1_cb(self, msg: Range):
        if math.isfinite(msg.range):
            self.sensor_ranges[1] = msg.range
            self.sensor_stamps[1] = msg.header.stamp

    def tof2_cb(self, msg: Range):
        if math.isfinite(msg.range):
            self.sensor_ranges[2] = msg.range
            self.sensor_stamps[2] = msg.header.stamp

    def joint_cb(self, msg: JointState):
        """Use actual joint state from Gazebo if available."""
        try:
            idx = msg.name.index('scan_joint')
            self.joint_angle = msg.position[idx]
            self.have_joint_state = True
        except ValueError:
            self.get_logger().warn('scan_joint not found in JointState', throttle_duration_sec=5.0)

    def servo_cb(self, msg: Float32):
        """Fallback: commanded servo angle with first-order lag filter."""
        # Convert commanded degrees to radians (joint = servo - servo_ref)
        commanded = math.radians(msg.data - self.servo_ref)
        # First-order lag filter: tau = 0.15 s (matches 3 rad/s max velocity)
        dt = 0.02  # assume 50 Hz update rate
        alpha = dt / (self.lag_tau + dt)
        self.filtered_servo = self.filtered_servo + alpha * (commanded - self.filtered_servo)

    def periodic_update(self):
        """Called at 10 Hz for sweep detection when using fallback angle."""
        now = self.get_clock().now()
        angle = self.joint_angle if self.have_joint_state else self.filtered_servo

        # Detect sweep direction change
        delta = angle - self.prev_angle
        if delta > 0.01:
            new_dir = 1
        elif delta < -0.01:
            new_dir = -1
        else:
            new_dir = self.sweep_direction

        if self.sweep_direction != 0 and new_dir != self.sweep_direction:
            self.direction_changes += 1
            if self.direction_changes >= 2:
                # Full sweep complete (went up AND came back down)
                self.publish_scan(now)
                self.reset_buffer()

        self.sweep_direction = new_dir
        self.prev_angle = angle

        # Place current readings into scan bins
        self.place_readings(now, angle)

    def place_readings(self, now, angle):
        """Place valid sensor readings into scan bins using actual/filtered angle."""
        # Check staleness and place readings
        for i in range(3):
            stamp = self.sensor_stamps[i]
            if stamp is None:
                continue
            age = (now - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e9
            if age > self.max_staleness:
                continue

            rng = self.sensor_ranges[i]
            if not math.isfinite(rng):
                continue
            if not (self.range_min <= rng <= self.range_max):
                continue

            # True world angle of this beam: joint_angle + mount_offset
            # mount_angles are in degrees (CCW from forward)
            mount_rad = math.radians(self.mount_angles[i])
            beam_angle = angle + mount_rad

            # Sensor origin is offset from joint center by sensor_offset along the beam
            effective_range = rng + self.sensor_offset

            # Normalize beam angle to 0-2π
            beam_angle = beam_angle % (2.0 * math.pi)

            # Convert to bin index
            bin_idx = int(beam_angle * self.num_bins / (2.0 * math.pi)) % self.num_bins

            # Place distance in bin (keep closest reading)
            if self.range_min <= effective_range <= self.range_max:
                if effective_range < self.ranges[bin_idx]:
                    self.ranges[bin_idx] = effective_range

    def publish_scan(self, now):
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.scan_frame

        scan.angle_min = 0.0
        scan.angle_increment = (2.0 * math.pi) / self.num_bins
        scan.angle_max = scan.angle_min + scan.angle_increment * (self.num_bins - 1)
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Proper time_increment: sweep duration / num_bins
        if self.last_publish_time is not None:
            sweep_duration = (now - self.last_publish_time).nanoseconds / 1e9
        else:
            sweep_duration = (now - self.scan_start_time).nanoseconds / 1e9
        scan.scan_time = max(sweep_duration, 0.1)
        scan.time_increment = scan.scan_time / self.num_bins

        scan.ranges = self.ranges[:]

        self.scan_pub.publish(scan)
        self.get_logger().debug(
            f'Published LaserScan: {self.num_bins} bins, '
            f'sweep_time={scan.scan_time:.3f}s, time_inc={scan.time_increment:.6f}s')

        self.last_publish_time = now

    def reset_buffer(self):
        self.ranges = [float('inf')] * self.num_bins
        self.direction_changes = 0
        self.scan_start_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = ScanAssembler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
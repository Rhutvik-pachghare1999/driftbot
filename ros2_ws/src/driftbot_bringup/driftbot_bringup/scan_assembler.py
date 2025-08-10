"""
scan_assembler.py — Assembles 360° LaserScan from 3 ToF Sensors + Servo Angle

Subscribes:
    /servo/position  (std_msgs/Float32)   — current servo angle in degrees
    /tof/sensor_0    (sensor_msgs/Range)  — distance from sensor 0
    /tof/sensor_1    (sensor_msgs/Range)  — distance from sensor 1
    /tof/sensor_2    (sensor_msgs/Range)  — distance from sensor 2

Publishes:
    /scan            (sensor_msgs/LaserScan) — 360° scan (1° resolution)

How it works:
    1. Each time servo position updates, calculate true_angle for each sensor:
         true_angle = (servo_angle - servo_reference) + mount_offset
    2. Place each sensor's distance into the correct bin (0°–359°)
    3. When the servo completes a full sweep (crosses zero twice), publish
       the accumulated scan buffer as a LaserScan message
    4. Clear buffer, start collecting next sweep

The LaserScan has 360 bins (1° each). During one servo sweep, ~3 points
are placed per servo position (one per sensor), filling ~360 total.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range, LaserScan


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
        self.declare_parameter('scan_timeout', 2.5)

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
        self.scan_timeout = self.get_parameter('scan_timeout').value

        # State
        self.ranges = [float('inf')] * self.num_bins  # scan buffer
        self.servo_angle = 90.0
        self.sensor_ranges = [0.0, 0.0, 0.0]  # latest range from each sensor
        self.prev_servo = 90.0
        self.sweep_direction = 0  # +1 = going up, -1 = going down
        self.direction_changes = 0  # 2 changes = 1 full sweep
        self.scan_start_time = self.get_clock().now()

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Subscribers
        self.create_subscription(Float32, 'servo/position', self.servo_cb, 10)
        self.create_subscription(Range, 'tof/sensor_0', self.tof0_cb, 10)
        self.create_subscription(Range, 'tof/sensor_1', self.tof1_cb, 10)
        self.create_subscription(Range, 'tof/sensor_2', self.tof2_cb, 10)

        # Timeout timer — publish whatever we have if sweep takes too long
        self.create_timer(1.0, self.check_timeout)

        self.get_logger().info(
            f'ScanAssembler: {self.num_bins} bins, '
            f'mounts=[{self.mount_angles[0]}, {self.mount_angles[1]}, {self.mount_angles[2]}]°'
        )

    def tof0_cb(self, msg):
        if msg.range > 0:
            self.sensor_ranges[0] = msg.range

    def tof1_cb(self, msg):
        if msg.range > 0:
            self.sensor_ranges[1] = msg.range

    def tof2_cb(self, msg):
        if msg.range > 0:
            self.sensor_ranges[2] = msg.range

    def servo_cb(self, msg):
        self.servo_angle = msg.data

        # Detect sweep direction change (servo reverses at endpoints)
        delta = self.servo_angle - self.prev_servo
        if delta > 0.5:
            new_dir = 1
        elif delta < -0.5:
            new_dir = -1
        else:
            new_dir = self.sweep_direction

        if self.sweep_direction != 0 and new_dir != self.sweep_direction:
            self.direction_changes += 1
            if self.direction_changes >= 2:
                # Full sweep complete (went up AND came back down)
                self.publish_scan()
                self.reset_buffer()

        self.sweep_direction = new_dir
        self.prev_servo = self.servo_angle

        # Place current readings into scan bins
        for i in range(3):
            if self.sensor_ranges[i] <= 0:
                continue

            # Calculate true world angle
            true_angle = (self.servo_angle - self.servo_ref) + self.mount_angles[i]
            # Normalize to 0-360
            true_angle = true_angle % 360.0
            if true_angle < 0:
                true_angle += 360.0

            # Convert to bin index
            bin_idx = int(true_angle * self.num_bins / 360.0) % self.num_bins

            # Place distance in bin (keep closest reading)
            dist = self.sensor_ranges[i]
            if self.range_min <= dist <= self.range_max:
                if dist < self.ranges[bin_idx]:
                    self.ranges[bin_idx] = dist

    def check_timeout(self):
        elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
        if elapsed > self.scan_timeout:
            # Publish whatever we have and reset
            self.publish_scan()
            self.reset_buffer()

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.scan_frame

        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / self.num_bins
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Time between measurements (approximate: sweep_time / num_bins)
        scan.time_increment = 0.0
        scan.scan_time = self.scan_timeout

        scan.ranges = self.ranges[:]

        self.scan_pub.publish(scan)
        self.get_logger().debug('Published LaserScan')

    def reset_buffer(self):
        self.ranges = [float('inf')] * self.num_bins
        self.direction_changes = 0
        self.scan_start_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = ScanAssembler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

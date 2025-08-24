#!/usr/bin/env python3
"""
DriftBot Live Monitor — watches all sensor data and commands in real-time.
Run: python3 monitor.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Range, Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math, time, os

class DriftBotMonitor(Node):
    def __init__(self):
        super().__init__('driftbot_monitor')

        # State
        self.servo_pos = 0.0
        self.enc_l = 0
        self.enc_r = 0
        self.tof = [0.0, 0.0, 0.0]
        self.imu_gyro_z = 0.0
        self.imu_accel_x = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.cmd_lin = 0.0
        self.cmd_ang = 0.0
        self.scan_count = 0
        self.last_scan_time = 0.0
        self.last_cmd_time = 0.0

        # Subscribers
        self.create_subscription(Float32, 'servo/position', self.servo_cb, 10)
        self.create_subscription(Int32, 'encoder/left', self.enc_l_cb, 10)
        self.create_subscription(Int32, 'encoder/right', self.enc_r_cb, 10)
        self.create_subscription(Range, 'tof/sensor_0', lambda m: self.tof_cb(0, m), 10)
        self.create_subscription(Range, 'tof/sensor_1', lambda m: self.tof_cb(1, m), 10)
        self.create_subscription(Range, 'tof/sensor_2', lambda m: self.tof_cb(2, m), 10)
        self.create_subscription(Imu, 'imu/data', self.imu_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)

        # Print timer (2Hz)
        self.create_timer(0.5, self.print_status)
        self.get_logger().info('DriftBot Monitor started — watching all topics')

    def servo_cb(self, msg): self.servo_pos = msg.data
    def enc_l_cb(self, msg): self.enc_l = msg.data
    def enc_r_cb(self, msg): self.enc_r = msg.data
    def tof_cb(self, idx, msg): self.tof[idx] = msg.range
    def imu_cb(self, msg):
        self.imu_gyro_z = msg.angular_velocity.z
        self.imu_accel_x = msg.linear_acceleration.x
    def scan_cb(self, msg):
        self.scan_count += 1
        self.last_scan_time = time.time()
    def cmd_cb(self, msg):
        self.cmd_lin = msg.linear.x
        self.cmd_ang = msg.angular.z
        self.last_cmd_time = time.time()

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny, cosy)

    def print_status(self):
        os.system('clear' if os.name == 'posix' else 'cls')

        now = time.time()
        cmd_age = now - self.last_cmd_time if self.last_cmd_time > 0 else 99
        scan_age = now - self.last_scan_time if self.last_scan_time > 0 else 99

        sweep_ok = "✅ SWEEPING" if self.servo_pos != 90.0 else "❌ STUCK at 90"
        scan_ok = f"✅ {self.scan_count} scans ({scan_age:.1f}s ago)" if self.scan_count > 0 else "❌ NO SCANS"
        cmd_active = f"✅ lin={self.cmd_lin:+.2f} ang={self.cmd_ang:+.2f}" if cmd_age < 2 else "⏸  idle"

        print("╔══════════════════════════════════════════════════════════╗")
        print("║           DriftBot Live Monitor                         ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print(f"║  SERVO:    {self.servo_pos:6.1f}°  {sweep_ok:<20}       ║")
        print(f"║  ENCODERS: L={self.enc_l:<6} R={self.enc_r:<6}                   ║")
        print(f"║  TOF:      [{self.tof[0]:.2f}m, {self.tof[1]:.2f}m, {self.tof[2]:.2f}m]         ║")
        print(f"║  IMU:      accel_x={self.imu_accel_x:+.2f}  gyro_z={self.imu_gyro_z:+.3f}    ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print(f"║  ODOM:     x={self.odom_x:+.3f}  y={self.odom_y:+.3f}  yaw={math.degrees(self.odom_yaw):+.1f}°  ║")
        print(f"║  SCAN:     {scan_ok:<40}  ║")
        print(f"║  CMD_VEL:  {cmd_active:<40}  ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print("║  DIAGNOSTICS:                                           ║")

        # Checks
        if self.servo_pos == 90.0:
            print("║  ⚠️  Servo not sweeping — scan won't build            ║")
        if self.enc_l == 0 and self.enc_r == 0:
            print("║  ⚠️  No encoder ticks — check magnet alignment        ║")
        if all(t == 0.0 for t in self.tof):
            print("║  ⚠️  All ToF reading 0 — I2C issue                    ║")
        if scan_age > 5 and self.scan_count == 0:
            print("║  ⚠️  No scans published — servo must sweep first      ║")
        if cmd_age < 2 and self.enc_l == 0:
            print("║  ⚠️  Driving but no ticks — wheels not engaging?       ║")
        if abs(self.imu_accel_x) > 2.0 and cmd_age > 2:
            print("║  ⚠️  High accel but no cmd — robot sliding/bumped?     ║")

        print("║  ✓  All systems nominal" if (self.servo_pos != 90.0 and self.scan_count > 0) else "")
        print("╚══════════════════════════════════════════════════════════╝")


def main():
    rclpy.init()
    node = DriftBotMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

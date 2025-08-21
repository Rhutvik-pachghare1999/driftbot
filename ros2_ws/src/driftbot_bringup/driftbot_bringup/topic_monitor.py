"""
topic_monitor.py — Live overview of key DriftBot ROS2 topics.

Prints message rate (Hz) and a one-line summary for each topic every 2 seconds.
Useful inside a tmux window or terminal during a test run.
"""

import math
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Float32


class TopicMonitor(Node):
    def __init__(self):
        super().__init__('driftbot_topic_monitor')

        self.period = self.declare_parameter('period', 2.0).value
        self.topic_specs = {
            '/cmd_vel': (Twist, self._fmt_twist),
            '/servo/position': (Float32, self._fmt_float),
            '/tof/sensor_0': (Range, self._fmt_range),
            '/tof/sensor_1': (Range, self._fmt_range),
            '/tof/sensor_2': (Range, self._fmt_range),
            '/imu/data': (Imu, self._fmt_imu),
            '/scan': (LaserScan, self._fmt_scan),
            '/odometry/filtered': (Odometry, self._fmt_odom),
            '/map': (OccupancyGrid, self._fmt_map),
        }

        self.state = defaultdict(lambda: {
            'count': 0,
            'first_stamp': None,
            'last_stamp': None,
            'last_msg': None,
        })

        for topic, (msg_type, fmt_fn) in self.topic_specs.items():
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, topic=topic, fmt_fn=fmt_fn: self._callback(topic, msg, fmt_fn),
                10,
            )

        self.create_timer(self.period, self._print)
        self.start_time = time.time()

    def _callback(self, topic, msg, fmt_fn):
        now = time.time()
        s = self.state[topic]
        s['count'] += 1
        s['last_msg'] = msg
        if s['first_stamp'] is None:
            s['first_stamp'] = now
        s['last_stamp'] = now

    def _fmt_twist(self, msg):
        return f"lin={msg.linear.x: .2f} ang={msg.angular.z: .2f}"

    def _fmt_float(self, msg):
        return f"val={msg.data:.1f}"

    def _fmt_range(self, msg):
        if msg.range <= 0.0 or math.isinf(msg.range) or math.isnan(msg.range):
            return "range=invalid"
        return f"range={msg.range:.3f}m"

    def _fmt_imu(self, msg):
        return (
            f"acc=({msg.linear_acceleration.x:6.2f},{msg.linear_acceleration.y:6.2f},"
            f"{msg.linear_acceleration.z:6.2f}) gyr=({msg.angular_velocity.x:6.3f},"
            f"{msg.angular_velocity.y:6.3f},{msg.angular_velocity.z:6.3f})"
        )

    def _fmt_scan(self, msg):
        vals = [r for r in msg.ranges if 0.0 < r < msg.range_max]
        if not vals:
            return f"bins={len(msg.ranges)} no_valid"
        return f"bins={len(msg.ranges)} min={min(vals):.2f} max={max(vals):.2f}m"

    def _fmt_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return f"x={msg.pose.pose.position.x:.3f} y={msg.pose.pose.position.y:.3f} yaw={math.degrees(yaw):5.1f}°"

    def _fmt_map(self, msg):
        return f"size={msg.info.width}x{msg.info.height} res={msg.info.resolution:.3f}m"

    def _print(self):
        now = time.time()
        self.get_logger().info("─── DriftBot topic monitor ───")
        for topic in self.topic_specs:
            s = self.state[topic]
            if s['count'] == 0:
                self.get_logger().info(f"{topic:30s}  no data")
                continue
            elapsed = (s['last_stamp'] - s['first_stamp']) if s['last_stamp'] else 0.0
            hz = s['count'] / elapsed if elapsed > 0.0 else 0.0
            summary = self.topic_specs[topic][1](s['last_msg'])
            age = now - s['last_stamp']
            self.get_logger().info(
                f"{topic:30s}  {hz:5.1f}Hz  age={age:.2f}s  {summary}"
            )
            # Reset counters to get a fresh rate every window
            s['count'] = 0
            s['first_stamp'] = None


def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

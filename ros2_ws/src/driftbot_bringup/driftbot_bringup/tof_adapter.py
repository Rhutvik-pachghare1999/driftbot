"""
tof_adapter.py — Converts simulated ToF LaserScans to sensor_msgs/Range

The Gazebo model publishes each ToF beam as a single-sample
sensor_msgs/LaserScan (via ros_gz_bridge, remapped to /tof/sensor_N_raw).
The real firmware publishes sensor_msgs/Range per ToF sensor, and the real
scan_assembler subscribes to /tof/sensor_{0,1,2} (sensor_msgs/Range).

This adapter converts each single-beam LaserScan to a Range message so the
real ROS stack (scan_assembler, topic_monitor, bag recorder, RViz) works
unmodified against the simulator.

Subscribes:
    /tof/sensor_0_raw  sensor_msgs/LaserScan  (from Gazebo, bridged)
    /tof/sensor_1_raw  sensor_msgs/LaserScan
    /tof/sensor_2_raw  sensor_msgs/LaserScan

Publishes:
    /tof/sensor_0  sensor_msgs/Range
    /tof/sensor_1  sensor_msgs/Range
    /tof/sensor_2  sensor_msgs/Range
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan


class TofAdapter(Node):
    def __init__(self):
        super().__init__('tof_adapter')

        self.declare_parameter('range_min', 0.04)   # VL53L1X minimum
        self.declare_parameter('range_max', 4.0)    # VL53L1X maximum
        self.declare_parameter('field_of_view', 0.047)  # ~2.7 deg VL53L1X FoV
        self.declare_parameter('frame_id', 'base_scan')

        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.fov = self.get_parameter('field_of_view').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pubs = {}
        self.subs = {}
        for i in range(3):
            topic = f'/tof/sensor_{i}'
            self.pubs[i] = self.create_publisher(Range, topic, 10)
            self.subs[i] = self.create_subscription(
                LaserScan, f'{topic}_raw',
                lambda msg, idx=i: self.scan_cb(msg, idx), 10)

        self.get_logger().info(
            f'TofAdapter: /tof/sensor_{{0,1,2}}_raw -> /tof/sensor_{{0,1,2}} '
            f'(Range, min={self.range_min}, max={self.range_max})')

    def scan_cb(self, msg: LaserScan, idx: int):
        if not msg.ranges:
            return

        rng = msg.ranges[0]
        if not math.isfinite(rng):
            # No return within sensor range — publish 0.0 (no reading),
            # matching the real firmware's out-of-range behavior.
            rng = 0.0
        rng = min(max(rng, 0.0), self.range_max)

        out = Range()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.radiation_type = Range.INFRARED
        out.field_of_view = self.fov
        out.min_range = self.range_min
        out.max_range = self.range_max
        out.range = rng
        self.pubs[idx].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TofAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
servo_sweep.py — Simulated scanning servo sweep (mirrors firmware servo_driver.cpp)

Replaces the ESP32's MCPWM sweep servo in simulation:
  - Computes the sinusoidal sweep  angle = center + amplitude * sin(speed * t)
    (center=96 deg, amplitude=60 deg, speed=3.0 rad/s — identical to
    firmware/config/servo_config.h)
  - Publishes std_msgs/Float64 on the gz JointPositionController topic so
    Gazebo physically rotates scan_joint = (angle - 96) deg in radians
  - Publishes std_msgs/Float32 on /servo/position (servo degrees, 36..156)
    so the real scan_assembler works unmodified:
    true_angle = (servo - 96) + mount_offset

Topics:
    Publishes:
      /servo/position   std_msgs/Float32   (servo degrees, 36..156)
      /scan_joint/cmd_pos  std_msgs/Float64 (rad, bridged to gz.msgs.Double)
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64


class ServoSweep(Node):
    def __init__(self):
        super().__init__('servo_sweep')

        # Same parameters as firmware/config/servo_config.h
        self.declare_parameter('servo_center', 96.0)      # SERVO_CENTER_DEG
        self.declare_parameter('servo_amplitude', 60.0)   # SERVO_AMPLITUDE_DEG
        self.declare_parameter('servo_speed', 3.0)        # SERVO_SPEED_RAD_S
        self.declare_parameter('publish_rate', 50.0)      # ~ SERVO_UPDATE_MS 10ms
        self.declare_parameter('servo_topic', '/servo/position')
        self.declare_parameter(
            'joint_cmd_topic', '/scan_joint/cmd_pos')

        self.center = self.get_parameter('servo_center').value
        self.amplitude = self.get_parameter('servo_amplitude').value
        self.speed = self.get_parameter('servo_speed').value
        self.rate = self.get_parameter('publish_rate').value
        servo_topic = self.get_parameter('servo_topic').value
        joint_topic = self.get_parameter('joint_cmd_topic').value

        self.phase = 0.0

        self.servo_pub = self.create_publisher(Float32, servo_topic, 10)
        self.joint_pub = self.create_publisher(Float64, joint_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate, self.update)

        self.get_logger().info(
            f'ServoSweep: center={self.center} amp={self.amplitude} '
            f'speed={self.speed} rad/s joint={joint_topic}')

    def update(self):
        dt = 1.0 / self.rate
        self.phase += self.speed * dt
        if self.phase > 2.0 * math.pi:
            self.phase -= 2.0 * math.pi

        # Servo angle in degrees (identical to firmware write_servo())
        servo_deg = self.center + self.amplitude * math.sin(self.phase)

        # Joint angle = (servo - center) deg -> radians
        joint_rad = math.radians(servo_deg - self.center)

        pos = Float32()
        pos.data = servo_deg
        self.servo_pub.publish(pos)

        cmd = Float64()
        cmd.data = joint_rad
        self.joint_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
odometry_node.py — Ackermann Odometry from Wheel Encoders + Steering Angle

Robot layout:
    Front axle (steering servo)
    ──────────────────────────
           │ wheelbase = 235mm
    ──────────────────────────
    Rear axle (driven by DC motor + differential gearbox)
    Left encoder    Right encoder

Subscribes:
    /encoder/left   (std_msgs/Int32)  — cumulative left rear wheel ticks
    /encoder/right  (std_msgs/Int32)  — cumulative right rear wheel ticks
    /servo/position (std_msgs/Float32) — scanning servo (NOT steering)

Publishes:
    /odom           (nav_msgs/Odometry) — robot pose and velocity
    TF: odom → base_link

Ackermann Bicycle Model:
    The rear axle center is the reference point.
    rear_speed = average of left and right wheel distances
    steering_angle = derived from encoder difference (differential gearbox)
    
    Since we have a differential gearbox:
      - When going straight: left_ticks ≈ right_ticks
      - When turning: outer wheel goes faster than inner wheel
      - Turn radius = (wheel_separation/2) × (left+right) / (right-left)
    
    This is equivalent to measuring the actual turn from the rear wheels
    without needing the steering servo angle (which we use for scanning,
    not for steering feedback).

    d_center = (d_left + d_right) / 2
    d_theta  = (d_right - d_left) / wheel_separation
    x += d_center × cos(theta)
    y += d_center × sin(theta)
    theta += d_theta

Note: Even though the robot is Ackermann, the rear differential gearbox
means the two rear encoders give us the SAME information as a differential
drive robot. The math is identical — the differential gearbox converts
Ackermann turning into differential wheel speeds at the rear.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(yaw):
    """Convert yaw angle (radians) to quaternion (only rotation around Z)."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Parameters
        self.declare_parameter('wheel_diameter', 0.064)
        self.declare_parameter('wheel_separation', 0.128)
        self.declare_parameter('wheelbase', 0.235)
        self.declare_parameter('ticks_per_revolution', 10)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_encoders', True)

        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Derived: meters per tick
        # wheel_circumference / ticks_per_rev = distance per tick
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left = None
        self.prev_right = None
        self.last_time = self.get_clock().now()

        # Latest tick values
        self.left_ticks = 0
        self.right_ticks = 0

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Int32, 'encoder/left', self.left_cb, 10)
        self.create_subscription(Int32, 'encoder/right', self.right_cb, 10)

        # Timer for odom publishing (20Hz)
        self.create_timer(0.05, self.publish_odom)

        self.get_logger().info(
            f'Odometry (Ackermann): d={self.wheel_diameter}m, '
            f'sep={self.wheel_separation}m, wheelbase={self.wheelbase}m, '
            f'{self.ticks_per_rev} ticks/rev, {self.meters_per_tick:.4f} m/tick'
        )

    def left_cb(self, msg):
        self.left_ticks = msg.data

    def right_cb(self, msg):
        self.right_ticks = msg.data

    def publish_odom(self):
        now = self.get_clock().now()

        if not self.get_parameter('use_encoders').value:
            # Encoders are disabled: publish a static identity odometry so the
            # TF tree stays intact. SLAM toolbox will localize from scan matching.
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            # High covariance so sensor-fusion nodes know this is not real motion.
            odom.pose.covariance[0] = 1e6
            odom.pose.covariance[7] = 1e6
            odom.pose.covariance[35] = 1e6
            self.odom_pub.publish(odom)

            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = self.odom_frame
                t.child_frame_id = self.base_frame
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)
            return

        # Skip first iteration
        if self.prev_left is None:
            self.prev_left = self.left_ticks
            self.prev_right = self.right_ticks
            self.last_time = now
            return

        # Delta ticks
        dl = self.left_ticks - self.prev_left
        dr = self.right_ticks - self.prev_right
        self.prev_left = self.left_ticks
        self.prev_right = self.right_ticks

        # Delta time
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt == 0:
            return

        # Convert ticks to meters
        d_left = dl * self.meters_per_tick
        d_right = dr * self.meters_per_tick

        # Ackermann with rear differential:
        # The differential gearbox makes the rear wheels behave exactly like
        # a differential drive — outer wheel goes faster in turns.
        # So we use the same math as differential drive:
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # Update pose (mid-point integration for better accuracy)
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Normalize theta to [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Velocities
        vx = d_center / dt
        vth = d_theta / dt

        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(self.theta)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        # Broadcast TF: odom → base_link
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = euler_to_quaternion(self.theta)
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

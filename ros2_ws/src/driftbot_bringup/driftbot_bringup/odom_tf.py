"""
odom_tf.py — Broadcasts odom->base_link TF from the Gazebo odometry message

The real stack publishes /odom + TF from the wheel odometry node (static
identity while encoders are offline). In simulation the gz-sim
AckermannSteering plugin publishes ground-truth odometry on
/model/driftbot/odometry, which ros_gz_bridge remaps to /odom (nav_msgs/
Odometry). This node converts that message into a dynamic TF transform
odom -> base_link so SLAM Toolbox and RViz get the same tree as the real
robot.

Subscribes:
    /odom   nav_msgs/msg/Odometry   (bridged from Gazebo AckermannSteering)

Publishes (TF):
    odom -> base_link   (from odometry pose, 30 Hz timer)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTf(Node):
    def __init__(self):
        super().__init__('odom_tf')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # TF requires best-effort compatible QoS (SLAM subscribes default).
        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_pose = None

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_cb, 10)

        self.tf_timer = self.create_timer(1.0 / 30.0, self.publish_tf)

        self.get_logger().info(
            f'OdomTf: /odom -> TF {self.odom_frame} -> {self.base_frame}')

    def odom_cb(self, msg: Odometry):
        self.latest_pose = msg

    def publish_tf(self):
        if self.latest_pose is None:
            return

        t = TransformStamped()
        t.header.stamp = self.latest_pose.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.latest_pose.pose.pose.position.x
        t.transform.translation.y = self.latest_pose.pose.pose.position.y
        t.transform.translation.z = self.latest_pose.pose.pose.position.z
        t.transform.rotation = self.latest_pose.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

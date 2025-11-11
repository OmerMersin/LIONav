#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class LioSamToMavros(Node):
    def __init__(self):
        super().__init__('lio_sam_to_mavros')

        # QoS compatible with LIO-SAM (BEST_EFFORT)
        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # MAVROS vision expects RELIABLE publishers
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            Odometry, '/lio_sam/mapping/odometry', self.odom_cb, qos_in)

        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', qos_out)

        self.speed_pub = self.create_publisher(
            TwistStamped, '/mavros/vision_speed/speed_twist', qos_out)

        self.get_logger().info("✅ LIO-SAM → MAVROS bridge active (pose + speed)")

    def odom_cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "map"
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)

        twist = TwistStamped()
        twist.header = msg.header
        twist.header.frame_id = "map"
        twist.twist = msg.twist.twist
        self.speed_pub.publish(twist)

def main():
    rclpy.init()
    node = LioSamToMavros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

class RelayOdometry(Node):
    def __init__(self):
        super().__init__('relay_odometry')

        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Odometry, '/lio_sam/mapping/odometry', self.callback, qos_in)
        self.pub = self.create_publisher(
            Odometry, '/mavros/odometry/out', qos_out)

        self.get_logger().info("✅ Bridging /lio_sam/mapping/odometry → /mavros/odometry/out (BEST_EFFORT → RELIABLE)")

    def callback(self, msg):
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = RelayOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

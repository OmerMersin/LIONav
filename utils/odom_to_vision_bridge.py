#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

class OdomToVisionBridge(Node):
    def __init__(self):
        super().__init__('odom_to_vision_bridge')

        self.sub = self.create_subscription(
            Odometry, '/lio_sam/mapping/odometry', self.odom_cb, 10)

        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10)

        self.twist_pub = self.create_publisher(
            TwistStamped, '/mavros/vision_speed/speed_twist', 10)

        self.get_logger().info('✅ Relaying /lio_sam/mapping/odometry → /mavros/vision_pose/pose')

    def odom_cb(self, msg):
        # --- Pose ---
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = msg.pose.pose
        self.pose_pub.publish(pose_msg)

        # --- Velocity ---
        twist_msg = TwistStamped()
        twist_msg.header = msg.header
        twist_msg.header.frame_id = 'map'
        twist_msg.twist = msg.twist.twist
        self.twist_pub.publish(twist_msg)

def main():
    rclpy.init()
    node = OdomToVisionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

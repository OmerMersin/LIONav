#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
# Temporary patch for old tf_transformations
if not hasattr(np, 'float'):
    np.float = float

from tf_transformations import quaternion_from_euler


class FakeVisionPublisher(Node):
    def __init__(self):
        super().__init__('fake_vision_publisher')

        # 10 Hz publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.vel_pub  = self.create_publisher(TwistStamped, '/mavros/vision_speed/speed_twist', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_data)  # 10 Hz
        self.start_time = time.time()

        self.get_logger().info('🟢 Fake VISION_POSITION_ESTIMATE + SPEED_ESTIMATE @10 Hz started')

    def publish_fake_data(self):
        t = time.time() - self.start_time

        # simple circle motion for realism
        r, w = 2.0, 0.4
        x = r * math.cos(w * t)
        y = r * math.sin(w * t)
        z = 1.5 + 0.2 * math.sin(0.3 * t)
        yaw = w * t
        q = quaternion_from_euler(0, 0, yaw)

        # Pose message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.pose_pub.publish(pose)

        # Velocity message (approx derivative)
        vel = TwistStamped()
        vel.header = pose.header
        vel.twist.linear.x = -r * w * math.sin(w * t)
        vel.twist.linear.y =  r * w * math.cos(w * t)
        vel.twist.linear.z =  0.06 * math.cos(0.3 * t)
        self.vel_pub.publish(vel)

def main():
    rclpy.init()
    node = FakeVisionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import csv
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class ReverseRTLExecutor(Node):
    def __init__(self):
        super().__init__('reverse_rtl_executor')

        # Load saved trajectory
        filename = self.declare_parameter('trajectory_file', '/home/orin/rtl_traj_latest.csv').value
        self.get_logger().info(f"📂 Loading trajectory from: {filename}")

        self.trajectory = []
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.trajectory.append((
                    float(row['x']),
                    float(row['y']),
                    float(row['z']),
                    float(row['qx']),
                    float(row['qy']),
                    float(row['qz']),
                    float(row['qw'])
                ))

        self.get_logger().info(f"Loaded {len(self.trajectory)} poses.")
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz output
        self.index = len(self.trajectory) - 1  # Start from last

    def timer_callback(self):
        if self.index < 0:
            self.get_logger().info("✅ Return to launch completed.")
            rclpy.shutdown()
            return

        x, y, z, qx, qy, qz, qw = self.trajectory[self.index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.publisher.publish(msg)

        self.index -= 1

def main():
    rclpy.init()
    node = ReverseRTLExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

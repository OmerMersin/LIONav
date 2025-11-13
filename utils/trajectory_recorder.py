#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import csv
from datetime import datetime

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # QoS matching LIO-SAM (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to LIO-SAM odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/lio_sam/mapping/odometry',
            self.odom_callback,
            qos_profile
        )

        # Prepare CSV file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # self.filename = f"/home/orin/rtl_traj_{timestamp}.csv"
        self.filename = f"/home/orin/rtl_traj_latest.csv"
        self.csvfile = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow([
            'time', 'x', 'y', 'z',
            'qx', 'qy', 'qz', 'qw',
            'vx', 'vy', 'vz'
        ])

        self.count = 0
        self.start_time = self.get_clock().now()
        self.get_logger().info(f"🛰️  Recording trajectory to {self.filename}")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist.linear

        # Relative timestamp in seconds
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        self.writer.writerow([
            f"{t:.3f}",
            pos.x, pos.y, pos.z,
            ori.x, ori.y, ori.z, ori.w,
            vel.x, vel.y, vel.z
        ])

        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(f"Recorded {self.count} poses...")

    def destroy_node(self):
        try:
            self.csvfile.close()
            self.get_logger().info("💾 Trajectory recording stopped.")
        except:
            pass  # Suppress any errors during cleanup
        try:
            super().destroy_node()
        except:
            pass  # Node may already be destroyed


def main():
    from rclpy.executors import ExternalShutdownException
    
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Suppress ROS2 shutdown exceptions (expected when terminated externally)
        pass
    except Exception as e:
        node.get_logger().error(f'Error during recording: {e}')
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except:
            pass  # Suppress cleanup errors
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass  # Context may already be shut down


if __name__ == '__main__':
    main()

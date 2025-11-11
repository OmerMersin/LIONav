#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class TimeOffsetMonitor(Node):
    def __init__(self):
        super().__init__('time_offset_monitor')

        # QoS that can receive both RELIABLE and BEST_EFFORT
        qos_any = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.last_imu = None
        self.last_cloud = None

        self.create_subscription(Imu, '/mavros/imu/data', self.imu_cb, qos_any)
        self.create_subscription(PointCloud2, '/ouster/points', self.cloud_cb, qos_any)

        self.timer = self.create_timer(1.0, self.print_offset)
        self.get_logger().info("⏱️ Monitoring time offset between IMU and LiDAR...")

    def imu_cb(self, msg):
        self.last_imu = msg.header.stamp

    def cloud_cb(self, msg):
        self.last_cloud = msg.header.stamp

    def print_offset(self):
        if self.last_imu is None or self.last_cloud is None:
            return
        imu_t = self.last_imu.sec + self.last_imu.nanosec * 1e-9
        cloud_t = self.last_cloud.sec + self.last_cloud.nanosec * 1e-9
        dt = imu_t - cloud_t
        self.get_logger().info(f"Δt (IMU - LiDAR) = {dt:+.6f} s")

def main():
    rclpy.init()
    node = TimeOffsetMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

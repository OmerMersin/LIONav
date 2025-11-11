import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, PointCloud2
from builtin_interfaces.msg import Time
from collections import deque
import numpy as np

class TimeMatcher(Node):
    def __init__(self):
        super().__init__('time_matcher_dynamic')

        self.qos_sensor = QoSPresetProfiles.SENSOR_DATA.value
        self.offset_window = deque(maxlen=50)  # store last 50 deltas
        self.avg_offset = 0.0
        self.last_lidar_stamp = None

        # Subscribers
        self.sub_imu = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_cb, self.qos_sensor)
        self.sub_lidar = self.create_subscription(
            PointCloud2, '/ouster/points', self.lidar_cb, self.qos_sensor)

        # Publisher
        self.pub_imu = self.create_publisher(Imu, '/imu_lio', self.qos_sensor)

        self.get_logger().info("Dynamic time matcher running...")

    def lidar_cb(self, msg: PointCloud2):
        # Keep most recent LiDAR timestamp
        self.last_lidar_stamp = msg.header.stamp

    def imu_cb(self, msg: Imu):
        if not self.last_lidar_stamp:
            return

        # Compute delta t in seconds
        t_imu = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_lidar = self.last_lidar_stamp.sec + self.last_lidar_stamp.nanosec * 1e-9
        dt = t_imu - t_lidar

        # Update rolling average
        self.offset_window.append(dt)
        self.avg_offset = float(np.mean(self.offset_window))

        # Apply correction (subtract average offset)
        corrected_time = t_imu - self.avg_offset
        if corrected_time < 0:
            corrected_time = 0

        new_msg = msg
        new_msg.header.stamp = Time()
        new_msg.header.stamp.sec = int(corrected_time)
        new_msg.header.stamp.nanosec = int((corrected_time % 1) * 1e9)
        self.pub_imu.publish(new_msg)

        # Log occasionally
        if len(self.offset_window) % 20 == 0:
            self.get_logger().info(f"Δt avg: {self.avg_offset*1000:.1f} ms")

def main(args=None):
    rclpy.init(args=args)
    node = TimeMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

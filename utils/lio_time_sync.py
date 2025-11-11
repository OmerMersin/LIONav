import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, PointCloud2
from builtin_interfaces.msg import Time
from collections import deque

class LioSync(Node):
    def __init__(self):
        super().__init__('lio_time_sync')

        self.qos_sensor = QoSPresetProfiles.SENSOR_DATA.value

        # Buffers
        self.imu_buf = deque(maxlen=500)
        self.lidar_buf = deque(maxlen=10)

        # Subscribers
        self.sub_imu = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_cb, self.qos_sensor)
        self.sub_lidar = self.create_subscription(
            PointCloud2, '/ouster/points', self.lidar_cb, self.qos_sensor)

        # Publishers (time-aligned topics)
        self.pub_imu = self.create_publisher(Imu, '/lio_imu', self.qos_sensor)
        self.pub_lidar = self.create_publisher(PointCloud2, '/lio_points', self.qos_sensor)

        self.get_logger().info('LIO sync node started (matching timestamps)')

    def imu_cb(self, msg: Imu):
        self.imu_buf.append(msg)

    def lidar_cb(self, msg: PointCloud2):
        if not self.imu_buf:
            return

        t_lidar = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Find closest IMU by timestamp
        closest = min(self.imu_buf,
                      key=lambda m: abs((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9) - t_lidar))

        t_imu = closest.header.stamp.sec + closest.header.stamp.nanosec * 1e-9
        delta = t_imu - t_lidar

        # Use the LiDAR timestamp as the unified time
        unified_time = msg.header.stamp

        # Republish both with identical timestamps
        imu_new = closest
        imu_new.header.stamp = unified_time
        lidar_new = msg
        lidar_new.header.stamp = unified_time

        self.pub_imu.publish(imu_new)
        self.pub_lidar.publish(lidar_new)

        self.get_logger().debug(f"Δt = {delta*1000:.1f} ms, unified stamp set to LiDAR time")

def main(args=None):
    rclpy.init(args=args)
    node = LioSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

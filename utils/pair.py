import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber

class Checker(Node):
    def __init__(self):
        super().__init__('sync_checker')
        self.imu_sub = Subscriber(self, Imu, '/lio_imu')
        self.lidar_sub = Subscriber(self, PointCloud2, '/lio_points')
        ats = ApproximateTimeSynchronizer([self.imu_sub, self.lidar_sub],
                                          queue_size=10, slop=0.001)
        ats.registerCallback(self.cb)

    def cb(self, imu, lidar):
        t_imu = imu.header.stamp.sec + imu.header.stamp.nanosec*1e-9
        t_lidar = lidar.header.stamp.sec + lidar.header.stamp.nanosec*1e-9
        self.get_logger().info(f"Δt={ (t_imu - t_lidar)*1000:.3f} ms")

def main():
    rclpy.init()
    rclpy.spin(Checker())
    rclpy.shutdown()

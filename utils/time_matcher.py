import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time

OFFSET = 0.065  # seconds, adjust to your measured mean offset

class ImuTimeShift(Node):
    def __init__(self):
        super().__init__('imu_time_shift')

        # Use SENSOR_DATA QoS to match MAVROS IMU publisher
        qos_profile = QoSPresetProfiles.SENSOR_DATA.value

        self.sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )

        self.pub = self.create_publisher(
            Imu,
            '/imu_lio',
            qos_profile
        )

        self.get_logger().info(f"IMU time shift node running with offset {OFFSET*1000:.1f} ms")

    def imu_callback(self, msg: Imu):
        # Shift timestamp backwards by OFFSET seconds
        total_nsec = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        total_nsec -= int(OFFSET * 1e9)
        if total_nsec < 0:
            total_nsec = 0

        new_msg = msg
        new_msg.header.stamp = Time()
        new_msg.header.stamp.sec = int(total_nsec // 1e9)
        new_msg.header.stamp.nanosec = int(total_nsec % 1e9)

        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTimeShift()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R


class ImuVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')

        # Match MAVROS IMU QoS (BEST_EFFORT)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber
        self.sub = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos)
        self.get_logger().info("Listening to /mavros/imu/data with BEST_EFFORT QoS...")

        # Initialize data
        self.orientation = np.eye(3)
        self.angular_vel = np.zeros(3)
        self.linear_acc = np.zeros(3)

        # Setup Matplotlib 3D
        self.fig = plt.figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self._setup_axes()

        # Animation (disable caching warning)
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, cache_frame_data=False)
        plt.show(block=False)

    def _setup_axes(self):
        self.ax.clear()
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('IMU Orientation / Angular Velocity / Acceleration')

    def imu_callback(self, msg: Imu):
        # Orientation quaternion to rotation matrix
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.orientation = R.from_quat(q).as_matrix()
        self.angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.linear_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

    def update_plot(self, _):
        self._setup_axes()
        origin = np.zeros(3)

        # Draw orientation axes
        for axis, color in zip(self.orientation.T, ['r', 'g', 'b']):
            self.ax.quiver(*origin, *axis, color=color)

        # Display numeric data
        text = (
            f"Angular Vel [rad/s]: {self.angular_vel[0]:+.2f}, {self.angular_vel[1]:+.2f}, {self.angular_vel[2]:+.2f}\n"
            f"Linear Acc [m/s²]: {self.linear_acc[0]:+.2f}, {self.linear_acc[1]:+.2f}, {self.linear_acc[2]:+.2f}"
        )
        self.ax.text2D(0.05, 0.95, text, transform=self.ax.transAxes)

    def spin(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                plt.pause(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            plt.close('all')


def main():
    rclpy.init()
    vis = ImuVisualizer()
    vis.spin()
    vis.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

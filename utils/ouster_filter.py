import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import open3d as o3d
import numpy as np

class OusterFilter(Node):
    def __init__(self):
        super().__init__('ouster_filter')
        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self.sub = self.create_subscription(PointCloud2, '/ouster/points', self.cb, qos)
        self.pub = self.create_publisher(PointCloud2, '/ouster/points_filtered', qos)
        self.get_logger().info('Runtime pointcloud filter active (Open3D SOR)')

    def cb(self, msg):
        # convert ROS → numpy
        pts = np.array(list(point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True)))
        if pts.size == 0:
            return
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        pc, _ = pc.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
        arr = np.asarray(pc.points)
        if arr.size == 0:
            return
        # back to PointCloud2
        ros_msg = point_cloud2.create_cloud_xyz32(msg.header, arr)
        self.pub.publish(ros_msg)

def main():
    rclpy.init()
    rclpy.spin(OusterFilter())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

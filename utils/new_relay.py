import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomToVision(Node):
    def __init__(self):
        super().__init__('odom_to_vision')
        self.sub = self.create_subscription(Odometry, '/lio_sam/mapping/odometry', self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
    def cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pub.publish(pose)

rclpy.init()
node = OdomToVision()
rclpy.spin(node)

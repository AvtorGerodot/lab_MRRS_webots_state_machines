#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import math

def quaternion_to_yaw(q):
    """
    Преобразует кватернион (geometry_msgs/Quaternion) в угол рыскания (yaw) в радианах.
    """
    # Нормализуем кватернион (на всякий случай)
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm == 0:
        return 0.0
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    
    # Вычисляем yaw: atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        
        # Подписчики на топики GPS и IMU (относительные)
        self.gps_sub = self.create_subscription(PointStamped, 'gps', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        
        # Издатель для Pose2D
        self.pose_pub = self.create_publisher(Pose2D, 'pose', 10)
        
        self.gps_data = None
        self.imu_data = None
        
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info('RobotStatePublisher started')

    def gps_callback(self, msg):
        # self.gps_data = msg.pose.pose.positione
        self.gps_data = msg.point   # msg.point – это Point (x, y, z)
        self.get_logger().debug(f'GPS: x={self.gps_data.x:.2f}, y={self.gps_data.y:.2f}')

    def imu_callback(self, msg):
        self.imu_data = msg.orientation

    def publish_pose(self):
        if self.gps_data is None or self.imu_data is None:
            return
        
        yaw = quaternion_to_yaw(self.imu_data)
        
        pose_msg = Pose2D()
        pose_msg.x = self.gps_data.x
        pose_msg.y = self.gps_data.y
        pose_msg.theta = yaw
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist, Vector3
import math

class PointToPointController(Node):
    def __init__(self):
        super().__init__('point_to_point_controller')
        
        # Параметры (можно менять через launch или командную строку)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('distance_tolerance', 0.1)    # метры
        self.declare_parameter('angle_tolerance', 0.05)     # радианы (~3°)
        self.declare_parameter('linear_speed', 0.3)         # м/с
        self.declare_parameter('angular_speed', 0.8)        # рад/с
        self.declare_parameter('angle_gain', 0.5)           # коэффициент для коррекции во время движения
        
        # Подписчик на текущую позу робота
        self.pose_sub = self.create_subscription(Pose2D, 'pose', self.pose_callback, 10)
        
        # Издатель команд скорости
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Таймер управления (20 Гц)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Состояния
        self.STATE_ROTATING = 0
        self.STATE_MOVING = 1
        self.STATE_FINAL_ROTATION = 2
        self.STATE_DONE = 3
        self.state = self.STATE_ROTATING
        
        # Текущее положение
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Цель (будет получена из параметров)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        self.dist_tol = self.get_parameter('distance_tolerance').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.angle_gain = self.get_parameter('angle_gain').value
        
        self.get_logger().info(
            f'PointToPointController started. Goal: ({self.goal_x}, {self.goal_y}, {self.goal_yaw:.2f} rad)'
        )
    
    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.theta
    
    def normalize_angle(self, angle):
        """Приводит угол в диапазон [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def control_loop(self):
        if self.state == self.STATE_DONE:
            self.cmd_vel_pub.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))
            return
        
        # Вычисляем расстояние и угол до цели
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)
        
        twist = Twist()
        
        if self.state == self.STATE_ROTATING:
            # Поворачиваемся к цели
            if abs(angle_error) > self.angle_tol:
                twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
                self.get_logger().debug(f'Rotating: error={angle_error:.3f}')
            else:
                self.state = self.STATE_MOVING
                self.get_logger().info('Target direction reached, start moving')
                # Возможно, потребуется небольшая задержка, но таймер сам вызовет следующий цикл
        
        elif self.state == self.STATE_MOVING:
            # Движение вперёд с коррекцией направления
            if distance > self.dist_tol:
                twist.linear.x = self.linear_speed
                # Простая П-регуляция угла
                twist.angular.z = self.angle_gain * angle_error
                # Ограничим угловую скорость максимумом
                max_angular = self.angular_speed
                if twist.angular.z > max_angular:
                    twist.angular.z = max_angular
                elif twist.angular.z < -max_angular:
                    twist.angular.z = -max_angular
                self.get_logger().debug(f'Moving: dist={distance:.3f}, ang_corr={twist.angular.z:.3f}')
            else:
                self.state = self.STATE_FINAL_ROTATION
                self.get_logger().info('Target position reached, final rotation')
        
        elif self.state == self.STATE_FINAL_ROTATION:
            # Поворот в заданную конечную ориентацию
            final_angle_error = self.normalize_angle(self.goal_yaw - self.current_yaw)
            if abs(final_angle_error) > self.angle_tol:
                twist.angular.z = self.angular_speed if final_angle_error > 0 else -self.angular_speed
                self.get_logger().debug(f'Final rotation: error={final_angle_error:.3f}')
            else:
                self.state = self.STATE_DONE
                self.get_logger().info('Goal reached, stopping')
                # Останавливаем таймер (или оставляем, но ничего не делаем)
                # self.timer.cancel()
        
        # Публикуем команду (если не DONE)
        if self.state != self.STATE_DONE:
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PointToPointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
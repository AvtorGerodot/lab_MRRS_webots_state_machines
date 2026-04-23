#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose2D, Twist
import math
import time
from rclpy.executors import MultiThreadedExecutor

from my_youbot_interfaces.action import MoveToPose

class PointToPointController(Node):
    def __init__(self, linear_speed=0.3, angular_speed=0.8, angle_gain=0.5):
        super().__init__('point_to_point_controller')
        
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.angle_gain = angle_gain
        self.distance_tolerance = 0.1
        self.angle_tolerance = 0.05
        
        self.pose_sub = self.create_subscription(Pose2D, 'pose', self.pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self._action_server = ActionServer(
            self,
            MoveToPose,
            'navigate_to_pose',
            self.execute_callback    # синхронный callback (не async)
        )
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_pose = False
        self.get_logger().info("PointToPointController Action Server ready")
    
    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.theta
        if not self.has_pose:
            self.has_pose = True
            self.get_logger().info(f"Got first pose: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f})")
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def execute_callback(self, goal_handle):
        goal = goal_handle.request.target
        self.get_logger().info(f"Received goal: ({goal.x:.2f}, {goal.y:.2f}, {goal.theta:.2f})")
        
        # Ожидание первой позы (таймаут 10 секунд)
        start_time = time.time()
        timeout = 10.0
        while not self.has_pose and (time.time() - start_time) < timeout:
            self.get_logger().warn("Waiting for initial pose...")
            time.sleep(0.5)
        
        if not self.has_pose:
            self.get_logger().error("No pose received, aborting goal")
            goal_handle.abort()
            result = MoveToPose.Result()
            result.success = False
            return result
        
        # Состояния
        STATE_ROTATING = 0
        STATE_MOVING = 1
        STATE_FINAL_ROTATION = 2
        state = STATE_ROTATING
        
        goal_x = goal.x
        goal_y = goal.y
        goal_yaw = goal.theta
        
        # Частота управления (20 Гц)
        rate = self.create_rate(20)
        
        while rclpy.ok():
            dx = goal_x - self.current_x
            dy = goal_y - self.current_y
            distance = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)
            
            # Feedback (синхронно публикуем)
            feedback_msg = MoveToPose.Feedback()
            feedback_msg.current_pose.x = self.current_x
            feedback_msg.current_pose.y = self.current_y
            feedback_msg.current_pose.theta = self.current_yaw
            goal_handle.publish_feedback(feedback_msg)
            
            twist = Twist()
            if state == STATE_ROTATING:
                if abs(angle_error) > self.angle_tolerance:
                    twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
                else:
                    state = STATE_MOVING
                    self.get_logger().info("Direction reached, moving forward")
            elif state == STATE_MOVING:
                if distance > self.distance_tolerance:
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angle_gain * angle_error
                    if twist.angular.z > self.angular_speed:
                        twist.angular.z = self.angular_speed
                    elif twist.angular.z < -self.angular_speed:
                        twist.angular.z = -self.angular_speed
                else:
                    state = STATE_FINAL_ROTATION
                    self.get_logger().info("Position reached, final rotation")
            elif state == STATE_FINAL_ROTATION:
                final_angle_error = self.normalize_angle(goal_yaw - self.current_yaw)
                if abs(final_angle_error) > self.angle_tolerance:
                    twist.angular.z = self.angular_speed if final_angle_error > 0 else -self.angular_speed
                else:
                    self.cmd_vel_pub.publish(Twist())
                    goal_handle.succeed()
                    result = MoveToPose.Result()
                    result.success = True
                    self.get_logger().info("Goal reached successfully")
                    return result
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()   # блокирующий сон, но в многопоточном исполнении это нормально
        
        # Если вышли из цикла (shutdown)
        result = MoveToPose.Result()
        result.success = False
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PointToPointController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
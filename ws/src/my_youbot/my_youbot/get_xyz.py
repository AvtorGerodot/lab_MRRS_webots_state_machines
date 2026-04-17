#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import Point
MAX_RANGE = 0.1

import sys
sys.path.insert(0, '/home/vadim/ros2_ws/src/executive_smach/smach')
import smach

sys.path.insert(0, '/home/vadim/ros2_ws/src/executive_smach/smach_ros')
import smach_ros

Q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



class GetXYZ(Node):
    def __init__(self):
        super().__init__('obstacle_smach')

        self.publisher_xyz = self.create_publisher(Point, 'xyz', 1)
        self.publisher_ee = self.create_publisher(String, 'end_effector', 1)

        self.create_subscription(Range, 'left_sensor', self.left_sensor_callback, 10)

        self.create_subscription(String, 'status_manipulator', self.status_manipulator_callback, 1)
        self.create_subscription(Point, 'xz_cube', self.xz_cube_callback, 1)

        self.left_sensor_value_ = None
        self.right_sensor_value_ = None
        self.flag_camera_ = None
        self.colour_ = None
        self.status_manipulator_ = ""

        self.xz_cube_ = Point()
        self.xyz_Point = Point()

    def left_sensor_callback(self, message):
        self.left_sensor_value_ = message.range

    def status_manipulator_callback(self, string):
        self.status_manipulator_ = string.data

    def xz_cube_callback(self, point):
        self.xz_cube_ = point

    def home_position(self):
        self.xyz_Point.x = 0.01
        self.xyz_Point.y = 0.2
        self.xyz_Point.z = 0.2
        end_effector = String()
        end_effector.data = "open"
        self.publisher_xyz.publish(self.xyz_Point)
        self.publisher_ee.publish(end_effector)

class HomePosition(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['outcome4', 'wait', 'take_cube_from_world', 'take_cube_from_youbot'])
        self.node = node

    def execute(self, userdata):
        rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.node.status_manipulator_ == "START":
            self.node.home_position()
            return 'outcome4'
        else:         
            return 'wait'

def main(args=None):
    rclpy.init(args=args)
    node = GetXYZ()
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])
    with sm:
        smach.StateMachine.add('HomePosition', HomePosition(node), 
                            transitions={'wait':'HomePosition',
                                         'take_cube_from_world':'HomePosition',
                                         'take_cube_from_youbot':'HomePosition'})

    outcome = sm.execute()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

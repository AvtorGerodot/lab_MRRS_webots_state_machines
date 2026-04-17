#!/usr/bin/env python
import rclpy
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point
MAX_RANGE = 0.1

import sys
sys.path.insert(0, '/home/vadim/ros2_ws/src/executive_smach/smach')
import smach

sys.path.insert(0, '/home/vadim/ros2_ws/src/executive_smach/smach_ros')
import smach_ros

class Smach(Node):
    def __init__(self):
        super().__init__('smach')

        self.publisher_ = self.create_publisher(String, 'status', 10)
        self.publisher_xyz = self.create_publisher(Point, 'xyz', 1)
        self.publisher_ee = self.create_publisher(String, 'end_effector', 1)
        self.publisher_need_colour = self.create_publisher(String, 'need_colour', 1)

        self.create_subscription(Range, 'left_sensor', self.left_sensor_callback, 10)

        self.create_subscription(Int32, 'wait_camera', self.wait_camera_callback, 1)
        self.create_subscription(String, 'green_detected', self.green_colour_callback, 1)
        self.create_subscription(String, 'red_detected', self.red_colour_callback, 1)
        self.create_subscription(Point, 'xz_cube', self.xz_cube_callback, 1)

        self.create_subscription(Int32, 'feedback', self.feedback_callback, 1)

        self.left_sensor_value_ = None
        self.right_sensor_value_ = None
        self.flag_camera_ = None
        self.colour_red = None
        self.colour_green = None
        self.feedback_ = None
        self.need_colour = String()

        self.xz_cube_ = Point()
        self.xyz_Point = Point()

    def left_sensor_callback(self, message):
        self.left_sensor_value_ = message.range

    def wait_camera_callback(self, int32):
        self.flag_camera_ = int32.data

    def red_colour_callback(self, string):
        self.colour_red = string.data

    def green_colour_callback(self, string):
        self.colour_green = string.data

    def xz_cube_callback(self, point):
        self.xz_cube_ = point

    def feedback_callback(self, int32):
        self.feedback_ = int32.data

    def home_position(self):
        self.xyz_Point.x = -0.01
        self.xyz_Point.y = 0.2
        self.xyz_Point.z = 0.2
        self.publisher_xyz.publish(self.xyz_Point)

    def end_effector(self, status):
        end_effector = String()
        if status == 0:
            end_effector.data = "open"
        elif status == 1:
            end_effector.data = "close"
        elif status == 0.5:
            end_effector.data = "cube"
        self.publisher_ee.publish(end_effector)

    def take(self):
        self.xyz_Point.x = self.xz_cube_.x

        while self.left_sensor_value_ is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.need_colour.data == "GREEN":
            a = float(self.left_sensor_value_) - 0.25 + 0.1 + 0.03
        elif self.need_colour.data == "RED":
            a = float(self.left_sensor_value_) + 0.1 + 0.03
        b = float(self.xz_cube_.x)
        self.xyz_Point.y = math.sqrt(abs(a)**2 - abs(b)**2)

        self.xyz_Point.z = self.xz_cube_.z

        self.publisher_xyz.publish(self.xyz_Point)
        

class START(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['START', 'WAIT'])
        self.node = node

    def execute(self, userdata):
        while self.node.flag_camera_ is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.node.flag_camera_ == 1:
            self.node.home_position()
            self.node.end_effector(0)
            self.node.flag_camera_ = None
            return 'START'
        else:
            self.node.flag_camera_ = None           
            return 'WAIT'
        
class Find_Cube(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['NEXT', 'WAIT'])
        self.node = node

    def execute(self, userdata):
        while self.node.colour_green is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.node.colour_green == "GREEN":
            self.node.need_colour.data = "GREEN"
            self.node.publisher_need_colour.publish(self.node.need_colour)
            return 'NEXT'
        else:
            return 'WAIT'
        
class GO(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['GO', 'STOP_TAKE', 'STOP_PUT'])
        self.node = node

    def execute(self, userdata):
        msg = String()

        while self.node.left_sensor_value_ is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        x = self.node.left_sensor_value_ - 0.25
        self.node.get_logger().info(f"{self.node.left_sensor_value_}, {x}")

        if self.node.need_colour.data == "GREEN":
            if self.node.left_sensor_value_ - 0.25 > MAX_RANGE:
                self.node.left_sensor_value_ = None
                msg.data = "GO"
                self.node.publisher_.publish(msg)
                return 'GO'
            else:
                self.node.left_sensor_value_ = None
                msg.data = "STOP"
                self.node.publisher_.publish(msg)
                if self.node.need_colour.data == "GREEN":
                    return 'STOP_TAKE'
                elif self.node.need_colour.data == "RED":
                    return 'STOP_PUT'
        elif self.node.need_colour.data == "RED":
            if self.node.left_sensor_value_ > MAX_RANGE:
                self.node.left_sensor_value_ = None
                msg.data = "GO"
                self.node.publisher_.publish(msg)
                return 'GO'
            else:
                self.node.left_sensor_value_ = None
                msg.data = "STOP"
                self.node.publisher_.publish(msg)
                if self.node.need_colour.data == "GREEN":
                    return 'STOP_TAKE'
                elif self.node.need_colour.data == "RED":
                    return 'STOP_PUT'
        
class TAKE(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['NEXT', 'WAIT'])
        self.node = node

    def execute(self, userdata):
        self.node.take()
        time.sleep(1)
        return 'NEXT'

class FIXED(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['NEXT', 'WAIT'])
        self.node = node

    def execute(self, userdata):
        self.node.end_effector(0.5)
        time.sleep(1)
        return 'NEXT'
    
class UNFIXED(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['NEXT', 'WAIT'])
        self.node = node

    def execute(self, userdata):
        self.node.end_effector(0)
        time.sleep(1)
        return 'NEXT'
    
class HOME_POSITION(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['NEXT', 'WAIT', "END"])
        self.node = node

    def execute(self, userdata):
        self.node.home_position()
        time.sleep(1)
        if self.node.need_colour.data == "RED":
            return 'END'
        else:
            return 'NEXT'
    
class FIND_TABLE(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['NEXT', 'WAIT'])
        self.node = node

    def execute(self, userdata):
        msg = String()
        msg.data = "ROTATE"
        self.node.publisher_.publish(msg)
        self.node.colour_red = None
        #while self.node.colour_red is None:
        rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.node.colour_red == "RED":
            self.node.need_colour.data = "RED"
            self.node.publisher_need_colour.publish(self.node.need_colour)
            return 'NEXT'
        else:
            return 'WAIT'
        
class PUT(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['NEXT', 'WAIT'])
        self.node = node

    def execute(self, userdata):
        self.node.take()
        time.sleep(1)
        return 'NEXT'

def main(args=None):
    rclpy.init(args=args)
    node = Smach()
    sm = smach.StateMachine(outcomes=['END', 'outcome5'])
    with sm:
        smach.StateMachine.add('START', START(node), 
                            transitions={'START':'Find_Cube',
                                         'WAIT':'START'})
        smach.StateMachine.add('Find_Cube', Find_Cube(node), 
                            transitions={'NEXT':'GO',
                                         'WAIT':'Find_Cube'})
        smach.StateMachine.add('GO', GO(node), 
                            transitions={'GO':'GO',
                                         'STOP_TAKE':'TAKE',
                                         'STOP_PUT':'PUT'})
        smach.StateMachine.add('TAKE', TAKE(node), 
                            transitions={'NEXT':'FIXED',
                                         'WAIT':'TAKE'})
        smach.StateMachine.add('FIXED', FIXED(node), 
                            transitions={'NEXT':'HOME_POSITION',
                                         'WAIT':'FIXED'})
        smach.StateMachine.add('HOME_POSITION', HOME_POSITION(node), 
                            transitions={'NEXT':'FIND_TABLE',
                                         'WAIT':'HOME_POSITION'})
        smach.StateMachine.add('FIND_TABLE', FIND_TABLE(node), 
                            transitions={'NEXT':'GO',
                                         'WAIT':'FIND_TABLE'})
        smach.StateMachine.add('PUT', PUT(node), 
                            transitions={'NEXT':'UNFIXED',
                                         'WAIT':'PUT'})
        smach.StateMachine.add('UNFIXED', UNFIXED(node), 
                            transitions={'NEXT':'HOME_POSITION',
                                         'WAIT':'FIXED'})

    outcome = sm.execute()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

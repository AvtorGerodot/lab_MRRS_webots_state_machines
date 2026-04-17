import rclpy
from geometry_msgs.msg import Twist, Point
import math
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Range
import numpy as np

HALF_DISTANCE_BETWEEN_WHEELS = 0.16
WHEEL_RADIUS = 0.05

KP = -0.003

Q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

LEFT = 0
RIGHT = 1

MIN_POS = 0.0
MAX_POS = 0.025
OFFSET_WHEN_LOCKED = 0.021


WHEEL_RADIUS = 0.05
LX = 0.228  # longitudinal distance from robot's COM to wheel [m].
LY = 0.158  # lateral distance from robot's COM to wheel [m].

threshold = 0.1

def bound(v, a, b):
  return max(a, min(v, b))

# Enum-like mapping in Python (using a dictionary)
arm_lengths = {
    'ARM1': 0.253,
    'ARM2': 0.155,
    'ARM3': 0.135,
    'ARM4': 0.081,
    'ARM5': 0.105,
}

def arm_get_sub_arm_length(arm):
    """Returns the length of a sub-arm.  Handles invalid arm names."""
    return arm_lengths.get(arm, 0.0) # Returns 0.0 if arm is not found


def arm_ik(x, y, z, Q):
    """Calculates and sets the joint angles for inverse kinematics."""
    y1 = math.sqrt(x**2 + y**2)
    z1 = z + arm_get_sub_arm_length('ARM4') + arm_get_sub_arm_length('ARM5') - arm_get_sub_arm_length('ARM1')

    a = arm_get_sub_arm_length('ARM2')
    b = arm_get_sub_arm_length('ARM3')
    c = math.sqrt(y1**2 + z1**2)

    if abs(x / y1) > 1:
        raise ValueError("Значение x/y1 выходит за пределы допустимого диапазона для asin")

    # Проверка на допустимые значения для acos
    acos_arg = (a**2 + c**2 - b**2) / (2.0 * a * c)
    if not -1 <= acos_arg <= 1:
        raise ValueError(f"{acos_arg} Значение аргумента acos выходит за пределы допустимого диапазона")

    acos_arg_gamma = (a**2 + b**2 - c**2) / (2.0 * a * b)
    if not -1 <= acos_arg_gamma <= 1:
        raise ValueError(f"{acos_arg_gamma}Значение аргумента acos для gamma выходит за пределы допустимого диапазона")

    Q[1] = -math.asin(x / y1)
    Q[2] = -(math.pi / 2 - math.acos((a**2 + c**2 - b**2) / (2.0 * a * c)) - math.atan(z1 / y1))
    Q[3] = -(math.pi - math.acos((a**2 + b**2 - c**2) / (2.0 * a * b)))
    Q[4] = -(math.pi + (Q[2] + Q[3]))
    Q[5] = math.pi / 2 + Q[1]

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot


        self.__right_forward_motor = self.__robot.getDevice('wheel1')
        self.__left_forward_motor = self.__robot.getDevice('wheel2')
        self.__right_backward_motor = self.__robot.getDevice('wheel3')
        self.__left_backward_motor = self.__robot.getDevice('wheel4')

        self.__left_forward_motor.setPosition(float('inf'))
        self.__left_forward_motor.setVelocity(0)

        self.__left_backward_motor.setPosition(float('inf'))
        self.__left_backward_motor.setVelocity(0)

        self.__right_forward_motor.setPosition(float('inf'))
        self.__right_forward_motor.setVelocity(0)

        self.__right_backward_motor.setPosition(float('inf'))
        self.__right_backward_motor.setVelocity(0)

        self.__arm1 = self.__robot.getDevice('arm1')
        self.__arm2 = self.__robot.getDevice('arm2')
        self.__arm3 = self.__robot.getDevice('arm3')
        self.__arm4 = self.__robot.getDevice('arm4')
        self.__arm5 = self.__robot.getDevice('arm5')

        self.__gripper = self.__robot.getDevice('finger::left')

        self.__gripper.setVelocity(0.03)

        self.__target_twist = Twist()
        self.__end_effector = ""
        self.__point = Point()
        self.__left_sensor_value_ = None
        self.__right_sensor_value_ = None


        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.__node.create_subscription(String, 'end_effector', self.__end_effector_callback, 1)
        self.__node.create_subscription(Point, 'xyz', self.__xyz_callback, 1)

        self.__node.create_subscription(String, 'status', self.__status_callback, 1)
        self.__status_value = ""

        self.__node.publisher_ = self.__node.create_publisher(Int32, 'feedback', 1)
        
        self.current_speed = None

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __end_effector_callback(self, string):
        self.__end_effector = string.data

    def __xyz_callback(self, point):
        self.__point = point
        if self.__point.x != 0.0 and self.__point.y != 0.0 and self.__point.z != 0.0:
            arm_ik(self.__point.x, self.__point.y, self.__point.z, Q)

    def __status_callback(self, string):
        self.__status_value = string.data


    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        speeds = [0.0] * 4  # Создаем список из 4 элементов со значением 0.0
        speeds[0] = 1 / WHEEL_RADIUS * (forward_speed + (LX + LY) * angular_speed)
        speeds[1] = 1 / WHEEL_RADIUS * (forward_speed - (LX + LY) * angular_speed)
        speeds[2] = 1 / WHEEL_RADIUS * (forward_speed + (LX + LY) * angular_speed)
        speeds[3] = 1 / WHEEL_RADIUS * (forward_speed - (LX + LY) * angular_speed)

        if self.__status_value == "ROTATE":
            speeds = [-speeds[0], -speeds[1], -speeds[2], -speeds[3]]
        else:
            speeds = [speeds[0], speeds[1], speeds[2], speeds[3]]

        self.__right_forward_motor.setVelocity(speeds[0])
        self.__left_forward_motor.setVelocity(speeds[1])
        self.__right_backward_motor.setVelocity(speeds[2])
        self.__left_backward_motor.setVelocity(speeds[3])

        #self.__left_forward_motor.setVelocity(command_motor_left)
        #self.__left_backward_motor.setVelocity(command_motor_left)
        #self.__right_forward_motor.setVelocity(command_motor_right)
        #self.__right_backward_motor.setVelocity(command_motor_right)

        self.__arm1.setPosition(Q[1])
        self.__arm2.setPosition(Q[2])
        self.__arm3.setPosition(Q[3])
        self.__arm4.setPosition(Q[4])
        self.__arm5.setPosition(0)

        if self.__end_effector == "open":
            self.__gripper.setPosition(MAX_POS)

        if self.__end_effector == "close":
            self.__gripper.setPosition(MIN_POS)

        if self.__end_effector == "cube":
            self.__gripper.setPosition((0.05-OFFSET_WHEN_LOCKED-0.001)/2)

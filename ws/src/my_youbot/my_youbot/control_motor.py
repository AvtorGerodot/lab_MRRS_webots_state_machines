import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
MAX_RANGE = 0.15
KP = -0.003

class ControlMotor(Node):
    def __init__(self):
        super().__init__('control_motor')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        
        self.create_subscription(Float32, 'rot_err', self.__rot_err__callback, 1)

        #self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        #self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)
        
        self.create_subscription(String, 'status', self.__status_callback, 1)

        self.__rot_err = Float32()
        self.__rot_err.data = 0.0

        self.__status_value = ""

    #def __left_sensor_callback(self, message):
    #    self.__left_sensor_value = message.range

    #def __right_sensor_callback(self, message):
    #    self.__right_sensor_value = message.range

    def __status_callback(self, string):
        self.__status_value = string.data

    def __rot_err__callback(self, float32):
        self.__rot_err = float32

        command_message = Twist()

        err = self.__rot_err.data

        if self.__status_value == "GO":
            command_message.linear.x = 0.08
            command_message.angular.z = err * KP
        elif self.__status_value == "STOP":
            command_message.linear.x = 0.0
            command_message.angular.z = 0.0
        elif self.__status_value == "ROTATE":
            command_message.linear.x = 0.0
            command_message.angular.z = 0.5

        #command_message.linear.x = 0.05
        
        self.__publisher.publish(command_message)

def main(args=None):
    rclpy.init(args=args)
    avoider = ControlMotor()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

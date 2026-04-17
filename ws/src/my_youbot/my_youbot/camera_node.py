import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Float32, Int32, String
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
import os

# Укажите путь к вашей модели YOLOv5
package_share = get_package_share_directory('my_youbot')
YOLO_MODEL_PATH = os.path.join(package_share, 'models', 'best3.pt')

CUBE_SIZE = 0.05
CUBE_SIZE_RED = 0.08

class Camera_node(Node):
  def __init__(self):
    super().__init__('camera_node')
    self.bridge = CvBridge()
    self.subscription = self.create_subscription(
      Image,
      '/camera/image_color', # Замените на вашу тему из Webots
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning
    self.create_subscription(String, 'need_colour', self.need_colour_callback, 1)

    self.__publisher_float = self.create_publisher(Float32, 'rot_err', 1)
    self.__publisher_int = self.create_publisher(Int32, 'wait_camera', 1)  # Издатель для топика 'wait'
    self.__publisher_green = self.create_publisher(String, 'green_detected', 1)
    self.__publisher_red = self.create_publisher(String, 'red_detected', 1)
    self.__publisher_xz = self.create_publisher(Point, 'xz_cube', 1)

    self.window_name = "Image from Webots with YOLOv8 detections" #Имя окна для удобства
    cv2.namedWindow(self.window_name) #Создаём окно ДО обработки изображений
    self.cv2_window_created = True # Указываем, что окно создано

    self.need_colour_ = None
    
    try:
      self.model = YOLO(YOLO_MODEL_PATH)
      self.model.conf = 0.6 # Установите порог уверенности
      self.model.iou = 0.7 # Установите порог IoU
      self.get_logger().info("YOLOv11 model loaded successfully.")
    except Exception as e:
      self.get_logger().error(f"Error loading YOLOv11 model: {e}")
      self.model = None

  def need_colour_callback(self, string):
        self.need_colour_ = string.data

  def listener_callback(self, msg):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
      image_height, image_width = cv_image.shape[:2]
      image_center_x = image_width // 2
      image_center_y = image_height // 2

      err = Float32()
      err.data = 1.0

      if self.model:
        results = self.model(cv_image)
        annotated_image = cv_image.copy() # Создаём копию изображения для отрисовки

        #command_message = Twist()

        # Ручная фильтрация результатов
        for *xyxy, conf, cls in results[0].boxes.data.tolist():
          x1, y1, x2, y2 = [round(float(x), 2) for x in xyxy]
          if conf >= 0.6: # Проверка порога уверенности
            cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2) # int() для cv2.rectangle
            label = f'{results[0].names[int(cls)]} {conf:.2f}'
        
            if results[0].names[int(cls)]=="green-cube":
              colour_wait = String()
              colour_wait.data = "GREEN"
              self.__publisher_green.publish(colour_wait)

              #while self.need_colour_ is None:
              #  rclpy.spin_once(self, timeout_sec=0.1)
              
              if self.need_colour_ == "GREEN":
                ## Расчет координат центра объекта (также как float)
                center_x = round(float((x1 + x2) / 2), 2)
                center_y = round(float((y1 + y2) / 2), 2)

                offset_x = round(float(center_x - image_center_x), 2)
                offset_y = round(float(center_y - image_center_y), 2)

                err.data = offset_x

                # Вывод информации о смещении
                self.get_logger().info(f"Object '{label}': Offset from center: ({offset_x}, {offset_y})")
                self.__publisher_float.publish(err)
              
                xz = Point()
                xz.x = (offset_x / abs(x2-x1)) * CUBE_SIZE
                xz.y = 0.0
                xz.z = CUBE_SIZE + 0.015
                self.__publisher_xz.publish(xz)

            if results[0].names[int(cls)]=="red-cube":
              colour_wait = String()
              colour_wait.data = "RED"
              self.__publisher_red.publish(colour_wait)
              
              if self.need_colour_ == "RED":
                ## Расчет координат центра объекта (также как float)
                center_x = round(float((x1 + x2) / 2), 2)
                center_y = round(float((y1 + y2) / 2), 2)#

                offset_x = round(float(center_x - image_center_x), 2)
                offset_y = round(float(center_y - image_center_y), 2)

                err.data = offset_x#

                # Вывод информации о смещении
                self.get_logger().info(f"Object '{label}': Offset from center: ({offset_x}, {offset_y})")
                self.__publisher_float.publish(err)
              
                xz = Point()
                xz.x = (offset_x / abs(x2-x1)) * CUBE_SIZE
                xz.y = 0.0
                xz.z = CUBE_SIZE_RED + CUBE_SIZE + 0.015
                self.__publisher_xz.publish(xz)

        if self.cv2_window_created and cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) >= 0:  #Проверка, что окно открыто
            # Отправляем 1 на топик 'wait' если окно открыто
            msg_wait = Int32()
            msg_wait.data = 1
            self.__publisher_int.publish(msg_wait)

        cv2.imshow(self.window_name, annotated_image)

      else:
        cv2.imshow("Image from Webots", cv_image)
      
      self.__publisher_float.publish(err)

      cv2.waitKey(1)

    except Exception as e:
      self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
  rclpy.init(args=args)
  webots_node = Camera_node()
  rclpy.spin(node=webots_node)
  webots_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import numpy as np
import math
from threading import Lock


class LaserScanMerger(Node):
    def __init__(self):
        super().__init__(node_name='laser_scan_merger')
        
        # Параметры
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('front_lidar_frame', 'lidar_front')
        self.declare_parameter('rear_lidar_frame', 'lidar_rear')
        self.declare_parameter('front_topic', '/lidar_front/scan')
        self.declare_parameter('rear_topic', '/lidar_rear/scan')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.005)  # ~0.3 градуса
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('duplicate_filter_distance', 0.05)  # 5 см для фильтрации дубликатов
        self.declare_parameter('enable_duplicate_filter', True)
        self.declare_parameter('transform_timeout', 0.1)
        self.declare_parameter('scan_timeout', 2.0)  # Время ожидания скана перед считанием его неактивным
        
        self.target_frame = self.get_parameter('target_frame').value
        self.front_lidar_frame = self.get_parameter('front_lidar_frame').value
        self.rear_lidar_frame = self.get_parameter('rear_lidar_frame').value
        self.front_topic = self.get_parameter('front_topic').value
        self.rear_topic = self.get_parameter('rear_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.duplicate_filter_distance = self.get_parameter('duplicate_filter_distance').value
        self.enable_duplicate_filter = self.get_parameter('enable_duplicate_filter').value
        self.transform_timeout = self.get_parameter('transform_timeout').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS профиль для лидаров
        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        qos_profile_subscriber = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Подписчики
        self.front_sub = self.create_subscription(
            LaserScan,
            self.front_topic,
            self.front_callback,
            qos_profile_subscriber
        )
        
        self.rear_sub = self.create_subscription(
            LaserScan,
            self.rear_topic,
            self.rear_callback,
            qos_profile_subscriber
        )
        
        # Издатель
        self.merged_pub = self.create_publisher(LaserScan, self.output_topic, qos_profile_publisher)
        
        # Буферы для хранения последних сканов
        self.front_scan = None
        self.rear_scan = None
        self.scan_lock = Lock()
        
        # ✅ НОВОЕ: Отслеживание активности лидаров
        self.front_active = False
        self.rear_active = False
        self.front_last_time = None
        self.rear_last_time = None
        
        # Количество лучей в выходном скане
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        # ✅ НОВОЕ: Таймер для проверки активности лидаров
        self.activity_timer = self.create_timer(1.0, self.check_lidar_activity)
        
        self.get_logger().info(f'Laser Scan Merger started')
        self.get_logger().info(f'Target frame: {self.target_frame}')
        self.get_logger().info(f'Front lidar: {self.front_topic} ({self.front_lidar_frame})')
        self.get_logger().info(f'Rear lidar: {self.rear_topic} ({self.rear_lidar_frame})')
        self.get_logger().info(f'Output: {self.output_topic}')
        self.get_logger().info(f'Will publish scans even with only one active lidar')
        
    def check_lidar_activity(self):
        """✅ НОВОЕ: Проверка активности лидаров по таймауту"""
        current_time = self.get_clock().now()
        
        # Проверяем передний лидар
        if self.front_last_time is not None:
            time_since_front = (current_time - self.front_last_time).nanoseconds / 1e9
            front_should_be_active = time_since_front < self.scan_timeout
            
            if self.front_active != front_should_be_active:
                self.front_active = front_should_be_active
                if front_should_be_active:
                    self.get_logger().info('✅ Front lidar came online!')
                else:
                    self.get_logger().warn('❌ Front lidar went offline (timeout)')
        
        # Проверяем задний лидар
        if self.rear_last_time is not None:
            time_since_rear = (current_time - self.rear_last_time).nanoseconds / 1e9
            rear_should_be_active = time_since_rear < self.scan_timeout
            
            if self.rear_active != rear_should_be_active:
                self.rear_active = rear_should_be_active
                if rear_should_be_active:
                    self.get_logger().info('✅ Rear lidar came online!')
                else:
                    self.get_logger().warn('❌ Rear lidar went offline (timeout)')
    
    def front_callback(self, msg):
        with self.scan_lock:
            self.front_scan = msg
            self.front_last_time = self.get_clock().now()
            
            # ✅ НОВОЕ: Отмечаем лидар как активный при первом сообщении
            if not self.front_active:
                self.front_active = True
                self.get_logger().info('✅ Front lidar came online!')
            
            self.merge_and_publish()
    
    def rear_callback(self, msg):
        with self.scan_lock:
            self.rear_scan = msg
            self.rear_last_time = self.get_clock().now()
            
            # ✅ НОВОЕ: Отмечаем лидар как активный при первом сообщении
            if not self.rear_active:
                self.rear_active = True
                self.get_logger().info('✅ Rear lidar came online!')
            
            self.merge_and_publish()
    
    def get_transform(self, from_frame, to_frame, time):
        """Получить трансформацию между фреймами"""
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                time,
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )
            return transform
        except TransformException as ex:
            self.get_logger().debug(f'Could not transform {from_frame} to {to_frame}: {ex}')
            return None
    
    def transform_scan_to_points(self, scan, transform):
        """Преобразовать LaserScan в точки в целевом фрейме"""
        points = []
        
        # Позиция и поворот лидара в целевом фрейме
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        
        # Преобразование кватерниона в угол поворота
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        
        # Угол поворота вокруг оси Z
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        for i, distance in enumerate(scan.ranges):
            if math.isnan(distance) or math.isinf(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
                
            # Угол луча в фрейме лидара
            angle = scan.angle_min + i * scan.angle_increment
            
            # Позиция точки в фрейме лидара
            x_local = distance * math.cos(angle)
            y_local = distance * math.sin(angle)
            
            # Преобразование в целевой фрейм
            x_global = tx + x_local * math.cos(yaw) - y_local * math.sin(yaw)
            y_global = ty + x_local * math.sin(yaw) + y_local * math.cos(yaw)
            
            points.append((x_global, y_global, distance))
        
        return points
    
    def filter_duplicate_points(self, points1, points2):
        """Фильтрация дублирующихся точек - оставляем ближайшие к лидару"""
        if not points1 or not points2:
            return points1, points2
        
        # Преобразуем в numpy массивы для ускорения вычислений
        p1_array = np.array([(p[0], p[1]) for p in points1])
        p2_array = np.array([(p[0], p[1]) for p in points2])
        
        filtered_points1 = []
        filtered_points2 = []
        used_from_p2 = set()  # Индексы уже использованных точек из points2
        
        # Сначала обрабатываем точки из первого лидара
        for i, point1 in enumerate(points1):
            distances = np.sqrt(np.sum((p2_array - np.array([point1[0], point1[1]]))**2, axis=1))
            closest_idx = np.argmin(distances)
            
            if distances[closest_idx] > self.duplicate_filter_distance:
                # Нет близких точек во втором скане - добавляем точку
                filtered_points1.append(point1)
            else:
                # Есть близкая точка во втором скане - выбираем ближайшую к источнику
                point1_original_dist = point1[2]
                point2_original_dist = points2[closest_idx][2]
                
                if point1_original_dist <= point2_original_dist:
                    # Точка из первого лидара ближе - берем её
                    filtered_points1.append(point1)
                    used_from_p2.add(closest_idx)
                else:
                    # Точка из второго лидара ближе - помечаем что её нужно взять
                    used_from_p2.add(closest_idx)
        
        # Теперь добавляем точки из второго лидара, которые не конфликтуют
        for i, point2 in enumerate(points2):
            if i not in used_from_p2:
                # Проверяем, не конфликтует ли с уже добавленными точками из первого лидара
                if len(filtered_points1) > 0:
                    p1_filtered_array = np.array([(p[0], p[1]) for p in filtered_points1])
                    distances = np.sqrt(np.sum((p1_filtered_array - np.array([point2[0], point2[1]]))**2, axis=1))
                    if np.min(distances) > self.duplicate_filter_distance:
                        filtered_points2.append(point2)
                else:
                    filtered_points2.append(point2)
            else:
                # Эта точка была выбрана как лучшая в конфликте
                filtered_points2.append(points2[i])
        
        return filtered_points1, filtered_points2
    
    def points_to_laser_scan(self, points, timestamp):
        """Преобразовать точки обратно в LaserScan"""
        scan = LaserScan()
        scan.header.stamp = timestamp
        scan.header.frame_id = self.target_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1  # Примерное время сканирования
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Инициализируем массив дистанций
        ranges = [float('inf')] * self.num_ranges
        
        for x, y, original_distance in points:
            # Вычисляем угол и дистанцию для этой точки
            angle = math.atan2(y, x)
            distance = math.sqrt(x*x + y*y)
            
            # Проверяем валидность дистанции
            if distance < self.range_min or distance > self.range_max:
                continue
            
            # Нормализуем угол к диапазону [angle_min, angle_max]
            while angle < self.angle_min:
                angle += 2 * math.pi
            while angle > self.angle_max:
                angle -= 2 * math.pi
            
            # Проверяем, попадает ли угол в наш диапазон
            if self.angle_min <= angle <= self.angle_max:
                # Находим индекс в массиве
                index = int(round((angle - self.angle_min) / self.angle_increment))
                if 0 <= index < self.num_ranges:
                    # Берем минимальную дистанцию (ближайшее препятствие)
                    if distance < ranges[index]:
                        ranges[index] = distance
        
        # Заменяем inf на nan для точек без измерений
        scan.ranges = [float('nan') if r == float('inf') else r for r in ranges]
        
        return scan
    
    def merge_and_publish(self):
        """✅ ИЗМЕНЕНО: Объединить сканы и опубликовать результат (работает с одним лидаром)"""
        # ✅ НОВОЕ: Проверяем, есть ли хотя бы один активный лидар
        if not self.front_active and not self.rear_active:
            self.get_logger().debug('No active lidars - skipping merge')
            return
        
        # ✅ ИЗМЕНЕНО: Работаем с тем что есть
        scans_to_process = []
        transforms_to_get = []
        
        if self.front_active and self.front_scan is not None:
            scans_to_process.append(('front', self.front_scan, self.front_lidar_frame))
            
        if self.rear_active and self.rear_scan is not None:
            scans_to_process.append(('rear', self.rear_scan, self.rear_lidar_frame))
        
        if not scans_to_process:
            self.get_logger().debug('No scans available for processing')
            return
        
        # Определяем время для трансформаций
        timestamps = []
        for name, scan, frame in scans_to_process:
            timestamps.append(rclpy.time.Time.from_msg(scan.header.stamp))
        
        current_time = max(timestamps)
        
        # Получаем трансформации и преобразуем сканы в точки
        all_points = []
        processed_lidars = []
        
        for name, scan, frame in scans_to_process:
            transform = self.get_transform(frame, self.target_frame, current_time)
            
            if transform is None:
                self.get_logger().warn(f'Could not get transform for {name} lidar')
                continue
            
            points = self.transform_scan_to_points(scan, transform)
            if points:
                all_points.extend(points)
                processed_lidars.append(name)
        
        if not all_points:
            self.get_logger().warn('No valid points after transformation')
            return
        
        # ✅ НОВОЕ: Информативное логирование
        status_parts = []
        if 'front' in processed_lidars:
            front_points = len([p for p in all_points if 'front' in processed_lidars])
            status_parts.append(f"front: ✅")
        else:
            status_parts.append(f"front: ❌")
            
        if 'rear' in processed_lidars:
            status_parts.append(f"rear: ✅")
        else:
            status_parts.append(f"rear: ❌")
        
        # Преобразуем точки обратно в LaserScan
        merged_scan = self.points_to_laser_scan(all_points, current_time.to_msg())
        
        # Подсчитываем финальные точки
        final_valid_points = sum(1 for r in merged_scan.ranges if not math.isnan(r))
        
        # Публикуем
        self.merged_pub.publish(merged_scan)
        
        # ✅ УЛУЧШЕННОЕ: Более информативное логирование
        lidar_status = ", ".join(status_parts)
        self.get_logger().debug(
            f'Scan merged ({lidar_status}): {len(processed_lidars)} lidar(s) -> '
            f'{len(all_points)} points -> {final_valid_points} final points'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = LaserScanMerger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
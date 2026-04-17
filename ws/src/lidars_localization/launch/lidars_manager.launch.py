from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
import os
import subprocess
import yaml


def generate_launch_description():

    package_dir = get_package_share_directory('lidars_localization')
    

    # Загрузка конфигурации лидаров из YAML файла
    config_path = os.path.join(package_dir, 'config', 'lidars', 'lidars_params.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    
    lidars_config = config['lidars']
    
    ## Определение портов лидаров с помощью скрипта
    lidar_ports_script = os.path.join(package_dir, 'lidars_localization', 'get_lidars_ports.py')

    result = subprocess.run(['python3', lidar_ports_script], 
                            capture_output=True, 
                            text=True)
    
    # опредление лаунч файла для взаимодействия с лидаров
    urg_launch_file = os.path.join(
        get_package_share_directory('urg_node2'),
        'launch',
        'urg_node2.launch.py'
    )

    # print(result)  # для дебага

    # Парсинг результата выполнения скрипта для получения портов
    ports = result.stdout.strip().split()
    lidar_front_serial_port = ports[0]
    lidar_rear_serial_port = ports[1]


    # небольшая логика для определения портов неподключенного лидара
    # (в случае, если )
    # print('OLD PORTS', lidar_front_serial_port, lidar_rear_serial_port) # для дебага
    
    if lidar_front_serial_port == "NONE" and lidar_rear_serial_port == "/dev/ttyACM0":
        lidar_front_serial_port = "/dev/ttyACM1"
    elif lidar_front_serial_port == "NONE" and lidar_rear_serial_port == "/dev/ttyACM1":
        lidar_front_serial_port = '/dev/ttyACM0'


    if lidar_rear_serial_port == "NONE" and lidar_front_serial_port == "/dev/ttyACM0":
        lidar_rear_serial_port = "/dev/ttyACM1"
    elif lidar_rear_serial_port == "NONE" and lidar_front_serial_port == "/dev/ttyACM1":
        lidar_rear_serial_port = "/dev/ttyACM0"

    # print('NEW PORTS', lidar_front_serial_port, lidar_rear_serial_port) # для дебага


    # Объявление аргументов с использованием конфига
    # Неймспейс переднего лидара. По дефолту импортируется из конфига
    lidar_front_namespace_arg = DeclareLaunchArgument(
        'lidar_front_namespace',
        default_value=lidars_config['front']['namespace'],
        description='Namespace for front lidar' 
    )

    # Фрейм переднего лидара. По дефолту импортируется из конфига
    lidar_front_frame_id_arg = DeclareLaunchArgument(
        'lidar_front_frame_id',
        default_value=lidars_config['front']['frame_id'],
        description='Frame_id for front lidar'
    )

    # Неймспейс заднего лидара. По дефолту импортируется из конфига
    lidar_rear_namespace_arg = DeclareLaunchArgument(
        'lidar_rear_namespace',
        default_value=lidars_config['rear']['namespace'],
        description='Namespace for rear lidar' 
    )

    # Фрейм заднего лидара. По дефолту импортируется из конфига
    lidar_rear_frame_id_arg = DeclareLaunchArgument(
        'lidar_rear_frame_id',
        default_value=lidars_config['rear']['frame_id'],
        description='Frame_id for rear lidar'
    )

    # Получение значений аргументов
    lidar_front_namespace = LaunchConfiguration('lidar_front_namespace')
    lidar_front_frame_id = LaunchConfiguration('lidar_front_frame_id')
    lidar_rear_namespace = LaunchConfiguration('lidar_rear_namespace')
    lidar_rear_frame_id = LaunchConfiguration('lidar_rear_frame_id')

    # Группа для переднего лидара (только если доступен)
    lidar_front_group = GroupAction([
        PushRosNamespace(lidar_front_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urg_launch_file),
            launch_arguments={
                'serial_port': lidar_front_serial_port,
                'namespace': lidar_front_namespace,
                'frame_id': lidar_front_frame_id,
                'angle_max': str(lidars_config['front']['angle_max']),
                'angle_min': str(lidars_config['front']['angle_min']),
            }.items()
        )
    ])

    # Группа для заднего лидара (только если доступен)
    lidar_rear_group = GroupAction([
        PushRosNamespace(lidar_rear_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urg_launch_file),
            launch_arguments={
                'serial_port': lidar_rear_serial_port,
                'namespace': lidar_rear_namespace,
                'frame_id': lidar_rear_frame_id,
                'angle_max': str(lidars_config['rear']['angle_max']),
                'angle_min': str(lidars_config['rear']['angle_min']),
            }.items()
        )
    ])

    return LaunchDescription([
        # Аргументы
        lidar_front_namespace_arg,
        lidar_front_frame_id_arg,
        lidar_rear_namespace_arg,
        lidar_rear_frame_id_arg,

        # Ноды           
        lidar_front_group,
        lidar_rear_group
    ])
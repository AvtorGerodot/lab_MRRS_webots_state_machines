from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import ExecuteProcess
from launch.conditions import UnlessCondition
import os
import yaml

def generate_launch_description():

    package_dir = get_package_share_directory('lidars_localization')
    laser_odometry_dir = get_package_share_directory('rf2o_laser_odometry')

    # Загрузка конфигурации лидаров из YAML файла
    config_path = os.path.join(package_dir, 'config', 'lidars', 'lidars_params.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    
    lidars_config = config['lidars']
    merger_config = config['merger_params']

    ## Конфигурационный файл RViz2
    rviz_config_path = os.path.join(package_dir, 'config', 'rviz2', 'rviz2_config.rviz')


    ### Объявлние аргументов
    ### Фрейм базы робота
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value= 'base_link',
        description='Base frame for the robot'
    )

    ### Фрей одометри
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value= 'odom',
        description='odom frame for the robot'
    )

    ### Глобальный фрейм (карта)
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value= 'map',
        description='map frame for the robot'
    )
    
    ### Фрейм переднего лидара
    lidar_front_frame_arg = DeclareLaunchArgument(
        'lidar_front_frame',
        default_value= lidars_config['front']['frame_id'],
        description='Frame of front lidar'
    )

    ### Топик переднего лидара
    lidar_front_topic_arg = DeclareLaunchArgument(
        'lidar_front_topic',
        default_value= merger_config['front_topic'],
        description='Topic of front lidar'
    )

    ### Фрейм заднего лидара
    lidar_rear_frame_arg = DeclareLaunchArgument(
        'lidar_rear_frame',
        default_value= lidars_config['rear']['frame_id'],
        description='Frame of rear lidar'
    )

    ### Топик заднего лидара
    lidar_rear_topic_arg = DeclareLaunchArgument(
        'lidar_rear_topic',
        default_value= merger_config['rear_topic'],
        description='Topic of rear lidar'
    )

    ### Использование симуляции
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value= 'True',
        choices=['True', 'False'],
        description='Use simulation time'
    )

    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    
    lidar_front_frame = LaunchConfiguration('lidar_front_frame')
    lidar_front_topic = LaunchConfiguration('lidar_front_topic')

    lidar_rear_frame = LaunchConfiguration('lidar_rear_frame')
    lidar_rear_topic = LaunchConfiguration('lidar_rear_topic')
    
    use_sim_time = LaunchConfiguration('use_sim_time')


    ### Вызов лаунч файла для коммуницирования с физическими лидарами
    ### НЕ запускается при работе с симуляцией ('use_sim_time' = 'True')
    lidars_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_dir, 'launch', 'lidars_manager.launch.py')
        ]),
        launch_arguments={
            'lidar_front_namespace': lidars_config['front']['namespace'],
            'lidar_front_frame_id': lidar_front_frame,
            'lidar_rear_namespace': lidars_config['rear']['namespace'],
            'lidar_rear_frame_id': lidar_rear_frame,
        }.items()
    )

    ### Запуск рвиза
    ### НЕ запускается при работе с симуляцией
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_path],
        parameters = [{
            'use_sim_time': True
        }],
    )

    ### Нода слияния сканов лидаров
    ### Работает как с ральными сенсорами, так и с симуляцией (то есть всегда запускается)
    laser_scan_merger = Node(
        package = 'lidars_localization',
        executable = 'laser_scan_merger',
        name = 'laser_scan_merger',
        output = 'screen',
        parameters = [{
            'target_frame': lidar_front_frame,               # то есть объединенный скан публикуется отноительно переднего лидара
            'front_lidar_frame': lidar_front_frame,          # 'lidar_front_frame',
            'rear_lidar_frame': lidar_rear_frame,            # 'lidar_rear_frame',
            'front_topic': lidar_front_topic,        # lidar_front_scan_topic
            'rear_topic': lidar_rear_topic,          # lidar_rear_scan_topic,
            'output_topic': merger_config['output_topic'],
            'angle_max': merger_config['angle_max'],
            'angle_min': merger_config['angle_min'],
            'angle_increment': merger_config['angle_increment'], 
            'range_min': merger_config['range_min'],
            'range_max': merger_config['range_max'],
            'transform_timeout': merger_config['transform_timeout'],
            'enable_duplicate_filter': merger_config['enable_duplicate_filter'],
            'duplicate_filter_distance': merger_config['duplicate_filter_distance'],
            'use_sim_time': use_sim_time
        }],
        respawn = True,
        respawn_delay = 1.0
    )

    ### Статичные трансофрмы для дебага
    ### НЕ запускаются при работе с симуляцией
    static_tf_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_dir, 'launch', 'tf_launch.py')
        ]),
        launch_arguments={
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'map_frame': map_frame,
            'lidar_front_frame': lidar_front_frame,
            'lidar_rear_frame': lidar_rear_frame
        }.items()
    )

    ### Нода лазерной одометрии
    ### Всегда запускается
    laser_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(laser_odometry_dir, 'launch', 'rf2o_laser_odometry.launch.py')
        ]),
        launch_arguments={
            'scan_topic': merger_config['output_topic'],
            'odom_topic': '/odom',
            'publish_tf': 'True',
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    ### Узел решения задачи SLAM
    ### На данный момент используется в режиме КАРТОГРАФИРОВАНИЯ
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_dir, 'launch', 'slam_launch.py')
        ]),
        launch_arguments={
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'map_frame': map_frame,
            'laser_frame': lidar_front_frame,
            'scan_topic': merger_config['output_topic'],
            'mode': 'mapping',
            'use_sim_time': use_sim_time
        }.items()
    )

    real_lidars_group = GroupAction(
        actions = [
            lidars_manager,
            rviz_node,
            static_tf_nodes
        ],
        condition = UnlessCondition(use_sim_time)
    )

    tf_base_to_lidar = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_tf_odom_to_base',
        arguments = ['0', '0', '0.0', '0', '0', '0', 'base_link', 'hokuyo_lidar'],
        output = 'screen',
        respawn = True,
        respawn_delay = 1.0
    )

    return  LaunchDescription([
        base_frame_arg,
        odom_frame_arg,
        map_frame_arg,

        lidar_front_frame_arg,
        lidar_front_topic_arg,

        lidar_rear_frame_arg,
        lidar_rear_topic_arg,

        use_sim_time_arg,

        # static_tf_nodes,
        
        # lidars_manager,
        tf_base_to_lidar,

        laser_scan_merger,

        laser_odometry,

        rviz_node,
        
        slam,

        # rviz_node,
    ])
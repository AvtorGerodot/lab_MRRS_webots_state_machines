import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('lidars_localization')
    slam_params_file = os.path.join(package_dir, 'config', 'slam', 'slam_params.yaml')

    # Загрузка конфигурации лидаров из YAML файла
    with open(slam_params_file, 'r') as file:
        config = yaml.safe_load(file)
    
    slam_config = config['slam_toolbox']['ros__parameters']

    # Объявление launch аргументов
    declare_base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value= slam_config ['base_frame'],
        description='Base frame for the robot'
    )
    
    declare_odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value= slam_config ['odom_frame'],
        description='Odometry frame'
    )
    
    declare_map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value= slam_config ['map_frame'],
        description='Map frame'
    )
    
    declare_laser_frame_arg = DeclareLaunchArgument(
        'laser_frame',
        default_value= slam_config ['laser_frame'],
        description='Laser frame'
    )
    
    declare_scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value= slam_config ['scan_topic'],
        description='Laser scan topic'
    )
    
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value= slam_config ['mode'],
        choices=['mapping', 'localization'],
        description='SLAM mode: mapping or localization'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value= 'False',
        choices=['True', 'False'],
        description='Use simulation time'
    )
    

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {   
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
                'scan_frame': LaunchConfiguration('laser_frame'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'mode': LaunchConfiguration('mode'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        respawn=True,
        respawn_delay=1.0
    )
    
    # Lifecycle manager для SLAM (опционально)
    # slam_lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_slam',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'autostart': True,
    #         'node_names': ['slam_toolbox'],
    #         'bond_timeout': 60.0
    #     }]
    # )
    
    return LaunchDescription([
        # Объявления аргументов
        declare_base_frame_arg,
        declare_odom_frame_arg,
        declare_map_frame_arg,
        declare_laser_frame_arg,
        declare_scan_topic_arg,
        declare_mode_arg,
        declare_use_sim_time_arg,
        
        # Узлы
        slam_toolbox_node,
        # slam_lifecycle_manager
    ])
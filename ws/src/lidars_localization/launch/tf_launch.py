from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value= 'base_link',
        description='Base frame for the robot'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value= 'odom',
        description='odom frame for the robot'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value= 'map',
        description='map frame for the robot'
    )

    lidar_front_frame_arg = DeclareLaunchArgument(
        'lidar_front_frame',
        default_value= 'lidar_front_frame',
        description='Frame for front lidar'
    )

    lidar_rear_frame_arg = DeclareLaunchArgument(
        'lidar_rear_frame',
        default_value= 'lidar_rear_frame',
        description='Frame for rear lidar'
    )

    lidar_front_frame = LaunchConfiguration('lidar_front_frame')
    lidar_rear_frame = LaunchConfiguration('lidar_rear_frame')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')


    tf_map_to_odom = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_tf_map_to_odom',
        arguments = ['0', '0', '0.2', '0', '0', '0', map_frame, odom_frame],
        output = 'screen',
        respawn = True,
        respawn_delay = 1.0
    )

    tf_odom_to_base = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_tf_odom_to_base',
        arguments = ['0', '0', '0.2', '0', '0', '0', odom_frame, base_frame],
        output = 'screen',
        respawn = True,
        respawn_delay = 1.0
    )


    tf_base_to_lidar_front = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_tf_base_to_front_lidar',
        arguments = ['0', '0', '0.2', '0', '0', '0', base_frame, lidar_front_frame],
        output = 'screen',
        respawn = True,
        respawn_delay = 1.0
    )
    
    tf_base_to_lidar_rear = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_tf_base_to_rear_lidar',
        arguments = ['-0.2', '0', '0.2', '3.14', '0', '0', base_frame, lidar_rear_frame],
        output = 'screen',
        respawn = True,
        respawn_delay = 1.0
    )

    return  LaunchDescription([
        base_frame_arg,
        odom_frame_arg,
        map_frame_arg,
        lidar_front_frame_arg,
        lidar_rear_frame_arg,

        # tf_map_to_odom,
        # tf_odom_to_base,
        tf_base_to_lidar_front,
        tf_base_to_lidar_rear,
    ])
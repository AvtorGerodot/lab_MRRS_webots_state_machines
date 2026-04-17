import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('my_youbot')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world_old.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    obstacle_avoider = Node(
        package='my_youbot',
        executable='obstacle_avoider',
    )

    obstacle_smach = Node(
        package='my_youbot',
        executable='obstacle_smach',
    )

    camera_node= Node(
        package='my_youbot',
        executable='camera_node',
    )

#    get_xyz= Node(
#        package='my_youbot',
#        executable='get_xyz',
#    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        obstacle_smach,
        obstacle_avoider,
        camera_node,
#        get_xyz,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])

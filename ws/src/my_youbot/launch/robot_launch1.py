import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('my_youbot')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot_gps_imu.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'three_robots_world_GPS_IMU_wider_walls.wbt'), # my_world_new.wbt
        mode='realtime',
        ros2_supervisor=True
    )


    ROBOT_NAMES = ['my_robot_0', 'my_robot_1', 'my_robot_2']
    ROBOT_NAMES_PARAM = ','.join(ROBOT_NAMES)
    
    # linear_speed, angular_speed
    robot_velocities = {
        'my_robot_0': [0.3, 0.8],
        'my_robot_1': [0.2, 0.6],
        'my_robot_2': [0.4, 1.0],
    }

    def create_robot(webots_robot_name_and_namespace):
        robot_driver = WebotsController(
            robot_name = webots_robot_name_and_namespace,
            namespace = webots_robot_name_and_namespace,
            parameters=[
                {'robot_description': robot_description_path,
                'use_sim_time' : True
                },
            ],
        )

        control_motor = Node(
            package='my_youbot',
            namespace = webots_robot_name_and_namespace,
            executable='control_motor',
            parameters=[{
                        'use_sim_time' : True,
            }],
        )

        state_publisher = Node(
            package = 'my_youbot',
            namespace = webots_robot_name_and_namespace,
            executable = 'robot_state_publisher',
            parameters = [{
                'use_sim_time': True,
            }],
        )

        point_to_point_controller = Node( 
            package='my_youbot',
            namespace=webots_robot_name_and_namespace,
            executable='point_to_point_controller',
            parameters=[{
                'use_sim_time': True,
                'robot_velocities': robot_velocities[webots_robot_name_and_namespace],
                }],
        )

        return [robot_driver, control_motor, state_publisher, point_to_point_controller]



    multi_robot_smach = Node(
        package='my_youbot',
        executable='multi_robot_smach',
        parameters=[{
            'use_sim_time': True,
            'robot_names': ROBOT_NAMES,
            }],
    )

    # smach = Node(
    #     package='my_youbot',
    #     executable='smach',
    #     output='log',
    #     parameters=[{
    #                 'use_sim_time' : True,
    #     }]
    # )

    # camera_node= Node(
    #     package='my_youbot',
    #     executable='camera_node',
    #     parameters=[{
    #                 'use_sim_time' : True,
    #     }]
    # )

#    get_xyz= Node(
#        package='my_youbot',
#        executable='get_xyz',
#    )

    return LaunchDescription([
        webots,
        webots._supervisor,

        # *[create_robot(robot_name) for robot_name in ROBOT_NAMES],
        *create_robot("my_robot_0"),    # распаковка списка
        *create_robot("my_robot_1"),
        *create_robot("my_robot_2"),
        
        multi_robot_smach,
        # smach,
        # camera_node,
        # get_xyz,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])

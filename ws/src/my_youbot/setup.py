from setuptools import find_packages, setup
import glob

package_name = 'my_youbot'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch1.py']))

data_files.append(('share/' + package_name + '/worlds', glob.glob('worlds/*.wbt')))

# data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))e
data_files.append(('share/' + package_name + '/resource', glob.glob('resource/*.urdf')))

data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/models', ['models/best3.pt']))
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = my_youbot.my_robot_driver:main',
            'control_motor = my_youbot.control_motor:main',
            'smach = my_youbot.smach:main',
            'camera_node= my_youbot.camera_node:main',
            'robot_state_publisher = my_youbot.robot_state_publisher:main',
            # 'get_xyz= my_youbot.get_xyz:main',
        ],
    },
)

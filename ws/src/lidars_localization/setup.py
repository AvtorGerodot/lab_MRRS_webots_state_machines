from setuptools import find_packages, setup
from glob import glob

package_name = 'lidars_localization'

data_files=[]
data_files.append(('share/ament_index/resource_index/packages',['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))


### launch files
data_files.append(('share/' + package_name + '/launch', glob('launch/*')))
### config files
data_files.append(('share/' + package_name + '/config/rviz2', ['config/rviz2/rviz2_config.rviz']))
data_files.append(('share/' + package_name + '/config/slam', ['config/slam/slam_params.yaml']))
data_files.append(('share/' + package_name + '/config/lidars', ['config/lidars/lidars_params.yaml']))

data_files.append(('share/' + package_name + '/lidars_localization', ['lidars_localization/get_lidars_ports.py']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rat',
    maintainer_email='rat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_scan_merger = lidars_localization.laser_scan_merger:main'
        ],
    },
)

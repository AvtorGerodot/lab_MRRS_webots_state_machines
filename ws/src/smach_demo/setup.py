from setuptools import find_packages, setup

package_name = 'smach_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dedbanzay',
    maintainer_email='dedbanzay@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_smach = smach_demo.simple_smach:main',
            'my_petri_net_demo = smach_demo.my_petri_net_demo:main',
        ],
    },
)

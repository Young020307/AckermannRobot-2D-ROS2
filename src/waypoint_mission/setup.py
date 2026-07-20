from setuptools import find_packages, setup


package_name = 'waypoint_mission'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='young',
    maintainer_email='young@example.com',
    description='Manual-release multi-waypoint mission manager for ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_mission_node = '
            'waypoint_mission.waypoint_mission_node:main',
        ],
    },
)

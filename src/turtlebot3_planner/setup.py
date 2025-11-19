from setuptools import setup, find_packages
import os

package_name = 'turtlebot3_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 indexing
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/planner_launch.py',
            'launch/rviz_launch.py',
        ]),

        # RViz config
        ('share/' + package_name + '/rviz', [
            'rviz/default.rviz',
        ]),

        # Maps
        ('share/' + package_name + '/maps', [
            'maps/map.yaml',
            'maps/map.pgm',
        ]),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'PyYAML',
        'Pillow',
    ],
    zip_safe=True,
    maintainer='rahulpanchal7',
    maintainer_email='rahulpanchal7.de@gmail.com',
    description='TurtleBot3 planner package with RViz integration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_node = turtlebot3_planner.planner_node:main',
            'path_generator = turtlebot3_planner.path_generator:main',
        ],
    },
)

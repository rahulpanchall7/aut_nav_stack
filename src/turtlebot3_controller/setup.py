from setuptools import find_packages, setup

package_name = 'turtlebot3_controller'

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
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='rahulpanchal7',
    maintainer_email='rahulpanchal7.de@gmail.com',
    description='Pure Pursuit trajectory tracking controller for TurtleBot3',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Pure Pursuit controller node
            'pure_pursuit_controller = turtlebot3_controller.pure_pursuit_controller:main',
        ],
    },
)

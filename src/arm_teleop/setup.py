from setuptools import find_packages, setup

package_name = 'arm_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'poses.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UCUSpaceRobotics',
    maintainer_email='arm@ucu.edu.ua',
    description='Direct operator control (keyboard teleop) for the robot arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_servo_node = arm_teleop.keyboard_servo_node:main',
        ],
    },
)

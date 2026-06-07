from setuptools import find_packages, setup

package_name = 'robot'

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
    maintainer='Isaac Ho, UCLA Mechanical Engineering B.S.',
    maintainer_email='isaacscho@g.ucla.edu',
    description='Project-NUEVO Master FSM and Hardware Controls',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = robot.robot_node:main',
            'main = robot.robot_node:main',
            'face_tracker = robot.face_tracker:main',
            'test_stepper_motor = robot.test_stepper_motor:main',
            'test_gripper_close = robot.test_gripper_close:main',
            'test_gripper_open = robot.test_gripper_open:main',
            'test_inward_belt = robot.test_inward_belt:main',
            'test_outward_belt = robot.test_outward_belt:main',
        ],
    },
)
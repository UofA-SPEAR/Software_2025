from setuptools import setup

package_name = 'teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name, ['launch/teleop.launch.py']),
        ('share/' + package_name, ['launch/keyboard.launch.py']),
        

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Induwara Kandapahala',
    maintainer_email='kandapah@ualberta.ca',
    description='Uses Joy Package (ROS Built-in) to get joy inputs and converts to MoveitServo compatiable msgs',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Rover_Control_Node = teleop.Rover_control:main',
            'SPEAR_Arm_Node = teleop.Joystick_Arm:main',
            'Joystick_Input = teleop.Joy_Input:main',
            'Rover_New_Control = teleop.Rover_control:main',
            'Gripper_Control = teleop.Gripper:main',
            'k_input = teleop.keyboard_input:main',
            'k_arm = teleop.keyboard_arm:main',
        ],
    },
)

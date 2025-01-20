from setuptools import find_packages, setup

package_name = 'teleop_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name, ['launch/teleop_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayden',
    maintainer_email='aydenbravender@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kipps_arm_node = teleop_control.joy_arm:main',
            'joy_input = teleop_control.joy_input:main',
        ],
    },
)

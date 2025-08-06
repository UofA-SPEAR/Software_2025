from setuptools import find_packages, setup
from glob import glob
from setup import setup
import os
package_name = 'kipp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spear1',
    maintainer_email='kandpah@ualberta.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_control_xbox = kipp.drive_control_xbox:main',
            'arm_control_xbox = kipp.arm_control_xbox:main',
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
from setup import setup
import os

package_name = 'kipp_arm_encoder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Induwara Kandpahala',
    maintainer_email='kandapah@ualberta.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kipp_arm_can = kipp_arm_encoder.kipp_arm_can:main',
            'kipp_arm_calibrate = kipp_arm_encoder.kipp_arm_calibartion:main',
            'kipp_arm_gripper = kipp_arm_encoder.kipp_gripper_can:main',            
        ],
    },
)

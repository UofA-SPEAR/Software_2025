from setuptools import find_packages, setup

package_name = 'kipp_hardware'

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
    maintainer='spearua',
    maintainer_email='kandapah@ualberta.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_node = kipp_hardware.kipp_can_drive:main',
            'xbox_node = kipp_hardware.xbox_controller_node:main',
            'gps_node = kipp_hardware.gps_serial:main'
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'kipp_sensor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Induwara Kandapahala',
    maintainer_email='kandapah@ualberta.ca',
    description='This Package contains the nodes that extract data from the GPS and IMU.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps = kipp_sensor.gps:main'
        ],
    },
)

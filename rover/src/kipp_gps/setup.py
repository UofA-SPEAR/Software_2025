from setuptools import find_packages, setup

package_name = 'kipp_gps'

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
            'gps_record = kipp_gps.save_gps_node:main',
            'gps_nav = kipp_gps.gps_navigation:main',
            'dummy_gps = kipp_gps.dummy_gps:main'
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob

package_name = 'description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/description']),
        ('share/description', ['package.xml']),
        ('share/description/launch', ['launch/display.launch.py']),  
        ('share/description/urdf', ['urdf/SPEAR_Arm.urdf']),
        ('share/description' , ['rviz_config.rviz']),
        ('share/description/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='indy03',
    maintainer_email='kandapah@ualberta.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

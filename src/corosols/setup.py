from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'corosols'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamzaboulandoum',
    maintainer_email='hamzaboulandoum@gmail.com',
    description='COROSOLS Robot ROS code',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial = corosols.serial_read:main',
            'pose_broadcaster = corosols.pose_broadcaster:main',            
            'input_client = corosols.user_input_client:main',
            'robot_speeds = corosols.robot_speeds:main',
            'imu_broadcaster = corosols.imu_broadcaster:main',  
            'station_connector = corosols.geocom',         
        ],
    },
)

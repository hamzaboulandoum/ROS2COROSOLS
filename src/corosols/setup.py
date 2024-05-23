from setuptools import find_packages, setup

package_name = 'corosols'

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
    maintainer='hamzaboulandoum',
    maintainer_email='hamzaboulandoum@gmail.com',
    description='COROSOLS Robot ROS code',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial = corosols.serial_read:main',
            'pose_broadcaster = corosols.pose_broadcaster:main',
            'user_input = corosols.user_input:main',
            'task = corosols.robot_task:main',
            
        ],
    },
)

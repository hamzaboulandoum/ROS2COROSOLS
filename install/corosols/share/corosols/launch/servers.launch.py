import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
def generate_launch_description():

    # Ensure the command is provided as a list of arguments
    ports_permissions = ExecuteProcess(
        cmd=["sudo", "chmod", "777", "/dev/ttyACM0"],
        output='screen'
    )

    return LaunchDescription([
        ports_permissions,
        
        Node(
            package='corosols',
            executable='serial',
            name='serial_reader'
        ),
        
        Node(
            package='corosols',
            executable='velocity_calculator',
            name='velocity_calculator'
        ),
        Node(
            package='corosols',
            executable='pose_broadcaster',
            name='pose_broadcaster'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

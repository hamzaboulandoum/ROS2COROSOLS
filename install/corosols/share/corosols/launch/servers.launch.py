import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, EmitEvent, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # Ensure the command is provided as a list of arguments
    ports_permissions = ExecuteProcess(
    cmd=["sudo", "bash", "-c", "chmod 777 /dev/ttyACM0 /dev/ttyACM1"],
    output='screen'
)
# /dev/ttyUSB0"],

    # Define your nodes
    serial_node = Node(
        package='corosols',
        executable='serial',
        name='serial_reader'
    )

    robot_speeds_node = Node(
        package='corosols',
        executable='robot_speeds',
        name='robot_speeds'
    )

    pose_broadcaster_node = Node(
        package='corosols',
        executable='pose_broadcaster',
        name='pose_broadcaster'
    )

    '''imu_broadcaster_node = Node(
        package='corosols',
        executable='imu_broadcaster',
        name='imu_broadcaster'
    )'''

    # Wait for ports_permissions to complete before launching nodes
    nodes_start = GroupAction([
        serial_node,
        robot_speeds_node,
        pose_broadcaster_node
    ])
    ''',
        imu_broadcaster_node'''
    nodes_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ports_permissions,
            on_exit=[nodes_start]
        )
    )

    # Create shutdown event handlers for each node
    def create_shutdown_handler(node):
        return RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason=f'{node.name} has exited'))]
            )
        )

    shutdown_group = GroupAction([
        create_shutdown_handler(serial_node),
        create_shutdown_handler(robot_speeds_node),
        create_shutdown_handler(pose_broadcaster_node)
    ])
    ''',
        create_shutdown_handler(imu_broadcaster_node)'''
    return LaunchDescription([
        ports_permissions,
        nodes_handler,
        shutdown_group
    ])

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, EmitEvent, RegisterEventHandler,GroupAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    # Ensure the command is provided as a list of arguments
    ports_permissions = ExecuteProcess(
        cmd=["sudo", "chmod", "777", "/dev/ttyACM0"],
        output='screen'
    )

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

    # Create shutdown event handlers for each node
    def create_shutdown_handler(node):
        return RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason=f'{node.name} has exited'))]
            )
        )
        
    node_group = GroupAction([
        serial_node,
        robot_speeds_node,
        pose_broadcaster_node
    ])
    shutdown_group = GroupAction([
        create_shutdown_handler(serial_node),
        create_shutdown_handler(robot_speeds_node),
        create_shutdown_handler(pose_broadcaster_node)
    ])

    return LaunchDescription([
        ports_permissions,
        node_group,
        shutdown_group
    ])

if __name__ == '__main__':
    generate_launch_description()
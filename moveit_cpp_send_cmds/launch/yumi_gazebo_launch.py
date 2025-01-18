import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            # "-topic",
            # "/robot_description",
            # "-name",
            # "rrbot_system_position",
            "-file",
            "/home/reuben/moveit_ws/src/moveit_cpp_send_cmds/meshes/cylinder_update.sdf",
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',            
        ],
    )

    left_arm_node = Node(
        package="moveit_cpp_send_cmds",
        executable="yumi_cmd",
        output="both",
    )

    right_arm_node = Node(
        package="moveit_cpp_send_cmds",
        executable="yumi_cmd_right",
        output="both",
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_right_arm_till_left = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_node,
            on_exit=[right_arm_node],
        )
    )

    nodes = [
        gz_spawn_entity,
        left_arm_node,
        delay_right_arm_till_left
    ]

    return LaunchDescription(nodes)
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments

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

    # Delay gazebo spawn after `rsp`
    delay_right_arm_till_left = TimerAction(
        period = 5.0,  # Delay in seconds
        actions=[right_arm_node]
    )    

    nodes = [
        left_arm_node,
        delay_right_arm_till_left
    ]

    return LaunchDescription(nodes)
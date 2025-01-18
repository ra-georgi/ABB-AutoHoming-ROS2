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

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("yumi_main_autohoming"),
            "config",
            "yumi_controllers.yaml",
        ]
    )

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'yumi_position_interface.urdf'
    urdf = os.path.join(
        get_package_share_directory('yumi_main_autohoming'),'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory("yumi_moveit_config_new"), "config", "basic.rviz")],
        # condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
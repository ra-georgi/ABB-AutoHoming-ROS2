import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare arguments

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("yumi_moveit_config_new"),
            "config",
            "ros2_controllers_position_interface.yaml",
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
        arguments=["yumi_arm_left_controller", "--param-file", robot_controllers],
    )

    robot_controller_spawner_right = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["yumi_arm_right_controller", "--param-file", robot_controllers],
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

    delay_right_controller_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner_right],
        )
    )


    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "yumi", package_name="yumi_moveit_config_new"
        )
        .robot_description(file_path=os.path.join(
            get_package_share_directory("yumi_main_autohoming"),
            "urdf",
            urdf_file_name
            )
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # .kinematics(file_path="config/kinematics.yaml")  # Add this line
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
            # pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[moveit_config.to_dict(), 
    #                 {"use_sim_time": is_sim},
    #                 {"publish_robot_description_semantic": True}],
    #     arguments=["--ros-args", "--log-level", "info"],
    # )


    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Delay movegroup start after `rviz`
    delay_movegroup_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[run_move_group_node],
        )
    )

    run_cmds_launch = LaunchDescription([
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("moveit_cpp_send_cmds"), '/launch', '/yumi_cmds_launch.py'])
                )])    

    # Delay movegroup start after `rviz`
    delay_cmd_after_right_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner_right,
            on_exit=[run_cmds_launch],
        )
    )


    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        delay_movegroup_after_jsb,
        delay_right_controller_after_robot_controller_spawner,
        delay_cmd_after_right_controller
    ]

    return LaunchDescription(declared_arguments + nodes)
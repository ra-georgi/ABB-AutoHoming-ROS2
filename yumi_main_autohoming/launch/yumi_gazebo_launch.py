import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

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
            default_value='true',
            description='Use simulation (Gazebo) clock if true')
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'yumi_gazebo.urdf'
    urdf = os.path.join(
        get_package_share_directory('yumi_main_autohoming'),'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    #     ),
    #     launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    # )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r empty.sdf "]
                    )
                ]
             ) 

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "rrbot_system_position",
            "-allow_renaming",
            "true",
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',            
        ],
    )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_controllers],
    #     output="both",
    # )
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

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    {"use_sim_time": use_sim_time},],
        # parameters=[moveit_config.to_dict(),{'use_sim_time': use_sim_time,}],
    )

    # Delay movegroup start after `rviz`
    delay_movegroup_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[run_move_group_node],
        )
    )

    # Delay gazebo spawn after `rsp`
    delay_gzspawn_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[gz_spawn_entity],
        )
    )

    # Delay gazebo spawn after `rsp`
    # delay_ROS2_CTRL = TimerAction(
    #     period=20.0,  # Delay in seconds
    #     actions=[control_node]
    # )

    # Delay gazebo spawn after `rsp`
    delay_CTRL_Spawn = TimerAction(
        period=6.0,  # Delay in seconds
        actions=[robot_controller_spawner]
    )

    # Set gazebo sim resource path
    # gazebo_resource_path = SetEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value=[
    #         '/home/reuben/moveit_ws/src',
    #         ]
    #     )
    
    nodes = [
        robot_state_pub_node,
        gazebo,
        # delay_ROS2_CTRL,
        delay_CTRL_Spawn,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        delay_movegroup_after_jsb,
        delay_right_controller_after_robot_controller_spawner,
        delay_gzspawn_after_rsp,
    ]

    return LaunchDescription(declared_arguments + nodes)
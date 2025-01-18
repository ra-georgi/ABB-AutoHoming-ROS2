import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import subprocess
from launch.actions import ExecuteProcess, LogInfo


from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare arguments

    get_joint_states = Node(
        package="yumi_main_autohoming",
        executable="get_joint_states",
        output="screen",
    )

    def update_urdf():
        # Define the paths to your xacro and URDF files
        description_dir = get_package_share_directory("yumi_main_autohoming")  
        xacro_file = os.path.join(description_dir,"urdf", 'yumi_socket.urdf.xacro')
        urdf_file = os.path.join(description_dir,"urdf", 'yumi_socket.urdf')
        try:
            subprocess.run(f"xacro {xacro_file} > {urdf_file}", shell=True, check=True)
            return LogInfo(msg=f"Successfully updated URDF file: {urdf_file}")
        except subprocess.CalledProcessError as e:
            return LogInfo(msg=f"Failed to update URDF file: {e}")
        
    # # Trigger the update when the first node exits
    # update_urdf_action = ExecuteProcess(
    #     # cmd=['echo', 'Updating URDF file...'],  # Dummy process to trigger the action
    #     on_exit=OnProcessExit(
    #         target_action=get_joint_states,
    #         on_exit=[update_urdf()],
    #     ),
    #     output='screen'
    # )

    # Delay update_urdf start after `rviz`
    update_urdf_action = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=get_joint_states,
            on_exit=[update_urdf()],
        )
    )    

    # log_info_action = LogInfo(msg=log_message)

    # urdf_file_name = 'yumi_socket.urdf'
    # urdf = os.path.join(
    #     get_package_share_directory('yumi_main_autohoming'),'urdf',
    #     urdf_file_name)
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

    # launch_next_file = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=update_urdf_action,
    #         on_exit=[
    #             ExecuteProcess(
    #                 cmd=["ros2", "launch", "yumi_main_autohoming", "yumi_socket_moveit_launch.py"],
    #                 output="screen"
    #             )
    #         ]
    #     )
    # )
 
    launch_socket_moveit = LaunchDescription([
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("yumi_main_autohoming"), '/launch', '/yumi_socket_moveit_launch.py'])
                )])

    #Delay for file to be written
    delay_moveit = TimerAction(
        period= 2.0,  # Delay in seconds
        actions=[launch_socket_moveit]
    )    

    # Delay update_urdf start after `rviz`
    launch_next_file = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=get_joint_states,
            on_exit=[delay_moveit],
        )
    ) 


    nodes = [
        get_joint_states,
        # update_urdf_action,
        launch_next_file
    ]

    return LaunchDescription(nodes)
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Node to start the controller_manager
    xacro_file_name = "tb3_burger.urdf.xacro"
    package_description = "ros2_urdf_project_gazebo"

    robot_model_path = get_package_share_directory(package_description)
    robot_desc_path = os.path.join(robot_model_path, "xacro", xacro_file_name)

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'use_sim_time': False, 'robot_description': Command(['xacro ', robot_desc_path])}],

        output="screen",
    )

    # Spawner for joint_state_broadcaster
    spawn_joint_state_broadcaster = TimerAction(
        period=2.0,  # Delay of 2 seconds
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            )
        ],
    )

    # Spawner for joint_trajectory_controller
    spawn_joint_trajectory_controller = TimerAction(
        period=4.0,  # Delay of 4 seconds
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller"],
                output="screen",
            )
        ],
    )

    # Spawner for velocity_controller
    spawn_velocity_controller = TimerAction(
        period=6.0,  # Delay of 6 seconds
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller"],
                output="screen",
            )
        ],
    )

    # Return the launch description
    return LaunchDescription([
        controller_manager_node,
        spawn_joint_state_broadcaster,
        spawn_joint_trajectory_controller,
        spawn_velocity_controller,
    ])

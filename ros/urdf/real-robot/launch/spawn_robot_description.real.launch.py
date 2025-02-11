#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random

from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    os.environ['LDS_MODEL'] = ""
    
    # Start the TurtleBot3 hardware drivers
    turtlebot3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            turtlebot3_node,
        ]
    )
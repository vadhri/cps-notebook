
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import LogInfo, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cartographer_config_dir = os.path.join(get_package_share_directory('localization_server'), 'config')
    map_file = cartographer_config_dir + "/turtlebot_area.yaml"
    nav2_yaml = cartographer_config_dir + "/amcl_config_with_pose.yaml"

    print ("map_file = ", map_file)

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file} 
                       ]),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server','amcl']}])
    ])

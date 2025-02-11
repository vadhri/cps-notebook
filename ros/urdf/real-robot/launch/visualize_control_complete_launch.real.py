import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import xacro

# this is the function launch  system will look for
def generate_launch_description():
    ####### DATA INPUT ##########
    xacro_file_name = "tb3_burger.urdf.xacro"
    package_description = "ros2_urdf_project_gazebo"

    robot_model_path = get_package_share_directory(package_description)
    robot_desc_path = os.path.join(robot_model_path, "xacro", xacro_file_name)

    print('XACRO : ', robot_desc_path)

    # convert XACRO file into URDF
    doc = xacro.parse(open(robot_desc_path))
    xacro.process_doc(doc, mappings={"use_sim": "False"})
    params = {'robot_description': doc.toxml()}
    params['use_sim_time'] = False

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        parameters=[params],
        output="screen"
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'tb3_burger.real.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': False}],
            arguments=['-d', rviz_config_dir])

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
            rviz_node
        ]
    )
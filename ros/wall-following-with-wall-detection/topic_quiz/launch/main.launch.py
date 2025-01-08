
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import LogInfo, RegisterEventHandler

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_quiz',
            executable='topic_quiz',
            output='screen'),
        Node(
            package='find_wall',
            executable='server',
            output='screen'),
    ])

# def generate_launch_description():
#     # Define the first node
#     server_node = Node(
#         package='find_wall',
#         executable='server',
#         output='screen',
#         name='find_wall_server',
#     )

#     # Define the second node that will be launched after the first node exits
#     topic_quiz_node = Node(
#         package='topic_quiz',
#         executable='topic_quiz',
#         output='screen',
#         name='topic_quiz_node',
#     )

#     # Event handler to launch the second node after the first node exits
#     event_handler = RegisterEventHandler(
#         OnProcessExit(
#             target_action=server_node,
#             on_exit=[topic_quiz_node],
#         )
#     )

#     return LaunchDescription([
#         server_node,
#         event_handler,  # Register the event handler
#     ])

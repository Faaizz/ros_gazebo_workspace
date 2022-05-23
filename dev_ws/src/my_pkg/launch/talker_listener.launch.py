from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'my_pkg',
            namespace = 'talker_listener',
            executable = 'talker',
            name = 'sim',
        ),
        Node(
            package = 'my_pkg',
            namespace = 'talker_listener',
            executable = 'listener',
            name = 'sim',
        ),
    ])

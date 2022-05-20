import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():

    # Configure the image_view Node
    node_robot_state_publisher = Node(
        package='image_view',
        executable='image_view',
        output='screen',
        parameters=[
            {
                'qos_overrides': {
                    'image_view_node': {
                        'subscription': {
                            'reliability': 'best_effort'
                        }
                    }
                }
            }
        ],
        remappings= [
            ('image', '/camera/image_raw'),
        ],
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
    ])

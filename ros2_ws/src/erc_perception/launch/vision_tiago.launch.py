from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="erc_perception",
                executable="perception_node",
                name="erc_perception_node",
                output="screen",
            ),
        ]
    )

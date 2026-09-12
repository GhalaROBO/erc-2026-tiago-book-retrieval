from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    image_publisher = Node(
        package="erc_perception",
        executable="image_publisher",
        name="erc_image_publisher",
        output="screen",
    )

    perception_node = Node(
        package="erc_perception",
        executable="perception_node",
        name="erc_perception_node",
        output="screen",
    )

    return LaunchDescription(
        [
            image_publisher,
            perception_node,
        ]
    )

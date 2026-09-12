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

    navigation_node = Node(
        package="erc_navigation",
        executable="navigation_node",
        name="erc_navigation_node",
        output="screen",
    )

    manipulation_node = Node(
        package="erc_manipulation",
        executable="manipulation_node",
        name="erc_manipulation_node",
        output="screen",
    )

    mission_node = Node(
        package="erc_mission",
        executable="mission_node",
        name="erc_mission_node",
        output="screen",
    )

    return LaunchDescription(
        [
            image_publisher,
            perception_node,
            navigation_node,
            manipulation_node,
            mission_node,
        ]
    )

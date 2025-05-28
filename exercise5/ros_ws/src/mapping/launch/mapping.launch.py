from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("grid_width", default_value="100", description=""),
            DeclareLaunchArgument("grid_height", default_value="100", description=""),
            DeclareLaunchArgument("grid_resolution", default_value="0.1", description=""),
            Node(
                package="mapping",
                executable="mapping",
                name="mapping",
                parameters=[
                    {"grid_width": LaunchConfiguration("grid_width")},
                    {"grid_height": LaunchConfiguration("grid_height")},
                    {"grid_resolution": LaunchConfiguration("grid_resolution")},
                ],
                output="screen"
            )
        ]
    )

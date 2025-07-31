from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("inflation_radius", default_value="1.0", description=""),
            DeclareLaunchArgument("sigma", default_value="0.2", description=""),
            DeclareLaunchArgument("threshold", default_value="45.0", description=""),
            DeclareLaunchArgument("use_sim_time", default_value="true", description=""),
            Node(
                package="path_planning",
                executable="path_planning",
                name="path_planning",
                parameters=[
                    {"inflation_radius":LaunchConfiguration("inflation_radius")},
                    {"sigma":LaunchConfiguration("sigma")},
                    {"threshold": LaunchConfiguration("threshold")},
                ],
                output="screen"
            )
        ]
    )

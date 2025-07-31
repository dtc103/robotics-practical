from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("grid_width", default_value="1000", description=""),
            DeclareLaunchArgument("grid_height", default_value="1000", description=""),
            DeclareLaunchArgument("grid_resolution", default_value="0.1", description=""),
            DeclareLaunchArgument("p_occ", default_value="0.8", description=""),
            DeclareLaunchArgument("p_free", default_value="0.45", description=""),
            DeclareLaunchArgument("use_sim_time", default_value="true", description=""),
            Node(
                package="mapping",
                executable="mapping",
                name="mapping",
                parameters=[
                    {"grid_width": LaunchConfiguration("grid_width")},
                    {"grid_height": LaunchConfiguration("grid_height")},
                    {"grid_resolution": LaunchConfiguration("grid_resolution")},
                    {"p_occ": LaunchConfiguration("p_occ")},
                    {"p_free": LaunchConfiguration("p_free")},
                ],
                output="screen"
            )
        ]
    )

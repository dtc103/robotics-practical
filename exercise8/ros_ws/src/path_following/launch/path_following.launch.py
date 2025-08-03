from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("p_gain", default_value="3.0", description=""),
            DeclareLaunchArgument("i_gain", default_value="0.0", description=""),
            DeclareLaunchArgument("d_gain", default_value="0.0", description=""),
            DeclareLaunchArgument("k_gain", default_value="3.0", description=""),
            DeclareLaunchArgument("length", default_value="0.3", description=""),
            DeclareLaunchArgument("width", default_value="0.3", description=""),
            DeclareLaunchArgument("use_sim_time", default_value="true", description=""),

            Node(
                package="path_following",
                executable="path_following",
                name="path_following",
                parameters=[
                    {"p_gain":LaunchConfiguration("p_gain")},
                    {"i_gain":LaunchConfiguration("i_gain")},
                    {"d_gain": LaunchConfiguration("d_gain")},
                    {"k_gain":LaunchConfiguration("k_gain")},
                    {"length":LaunchConfiguration("length")},
                    {"width":LaunchConfiguration("width")},
                ],
                output="screen"
            )
        ]
    )

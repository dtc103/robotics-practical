from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("kAtt", default_value="0.5", description="Number of laser segments"),
            DeclareLaunchArgument("kRep", default_value="0.4", description="Number of laser segments"),
            DeclareLaunchArgument("d_thres", default_value="0.4", description="Number of laser segments"),
            DeclareLaunchArgument("length", default_value="1.0", description="Number of laser segments"),
            DeclareLaunchArgument("width", default_value="1.0", description="Number of laser segments"),
            DeclareLaunchArgument("segments", default_value="10", description="Number of laser segments"),
            Node(
                package="obstacle_avoidance",
                executable="obstacle_avoidance",
                name="obstacle_avoidance",
                parameters=[
                    {"kAtt": LaunchConfiguration("kAtt")},
                    {"kRep": LaunchConfiguration("kRep")},
                    {"d_thres": LaunchConfiguration("d_thres")},
                    {"length": LaunchConfiguration("length")},
                    {"width": LaunchConfiguration("width")},
                    {"segments": LaunchConfiguration("segments")}
                ],
                output="screen"
            )
        ]
    )

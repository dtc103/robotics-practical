from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("kAtt", default_value="0.5", description="Scaling factor for the attractor vector in potential fields"),
            DeclareLaunchArgument("kRep", default_value="0.4", description="Scaling factor for the repulsive vector in potential fields"),
            DeclareLaunchArgument("var", default_value="1.57079", description="Defines the 'width' of the angular attenuation of the segments"),
            DeclareLaunchArgument("d_thres", default_value="0.4", description="Threshold for the repulsive calculation"),
            DeclareLaunchArgument("length", default_value="1.0", description="Length of the death zone"),
            DeclareLaunchArgument("width", default_value="1.0", description="Width of the death zone"),
            DeclareLaunchArgument("segments", default_value="10", description="Number of laser segments"),
            DeclareLaunchArgument("x_goal", default_value="0.0", description="x-coordinate of the goal position"),
            DeclareLaunchArgument("y_goal", default_value="0.0", description="y-coordinate of the goal position"),
            DeclareLaunchArgument("rot_gain", default_value="0.0", description="Gain parameter for angular motion"),
            Node(
                package="obstacle_avoidance",
                executable="obstacle_avoidance",
                name="obstacle_avoidance",
                parameters=[
                    {"kAtt": LaunchConfiguration("kAtt")},
                    {"kRep": LaunchConfiguration("kRep")},
                    {"var": LaunchConfiguration("var")},
                    {"d_thres": LaunchConfiguration("d_thres")},
                    {"length": LaunchConfiguration("length")},
                    {"width": LaunchConfiguration("width")},
                    {"segments": LaunchConfiguration("segments")},
                    {"x_goal": LaunchConfiguration("x_goal")},
                    {"y_goal": LaunchConfiguration("y_goal")},
                    {"rot_gain": LaunchConfiguration("rot_gain")}
                ],
                output="screen"
            )
        ]
    )

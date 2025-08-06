from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("p_gain", default_value="3.0", description="Proportional gain for the Stanley heading controller."),
            DeclareLaunchArgument("i_gain", default_value="0.0", description="Integral gain for the Stanley heading controller."),
            DeclareLaunchArgument("d_gain", default_value="0.0", description="Derivative gain for the Stanley heading controller."),
            DeclareLaunchArgument("k_gain", default_value="3.0", description="Cross-track error correction gain (Stanley method)."),
            

            DeclareLaunchArgument("length", default_value="0.15", description="Length of danger zone."),
            DeclareLaunchArgument("width", default_value="0.3", description="Width of danger zone."),
            DeclareLaunchArgument("near_deadband", default_value="0.2", description="Distance from robot that is ignored for obstacle avoidance."),
            DeclareLaunchArgument("min_obstacle_points", default_value="2", description="Minimum of consecutive hits necessary to detect obstacle."),
            DeclareLaunchArgument("omega_avoid", default_value="0.5", description="Angel velocity with which the robot scans environment when obstacle detected."),
            DeclareLaunchArgument("rotate_duration", default_value="2.5", description="Duration of rotation (angel = omega_avoid x rotate_duration)."),
            DeclareLaunchArgument("reverse_duration", default_value="1.5", description="Duration of backing up when obstacle detected."),

            
            DeclareLaunchArgument("v_max", default_value="0.75", description="Max. velocity."),
            DeclareLaunchArgument("a_max", default_value="0.25", description="Max. acceleration"),
            DeclareLaunchArgument("omega_max", default_value="2.0", description="Max. Angel velocity."),
            DeclareLaunchArgument("omega_slow", default_value="0.85", description="Angular velocity threshold at which forward speed scales down to zero."),
            DeclareLaunchArgument("decel_distance", default_value="1.0", description="Distance to goal at which the robot starts decelerating."),
            
            DeclareLaunchArgument("use_sim_time", default_value="true", description="simulation time"),
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
                    {"near_deadband":LaunchConfiguration("near_deadband")},
                    {"min_obstacle_points":LaunchConfiguration("min_obstacle_points")},
                    {"omega_avoid":LaunchConfiguration("omega_avoid")},
                    {"v_max":LaunchConfiguration("v_max")},
                    {"a_max":LaunchConfiguration("a_max")},
                    {"omega_slow":LaunchConfiguration("omega_slow")},
                    {"omega_max":LaunchConfiguration("omega_max")},
                    {"rotate_duration":LaunchConfiguration("rotate_duration")},
                    {"reverse_duration":LaunchConfiguration("reverse_duration")},
                    {"decel_distance":LaunchConfiguration("decel_distance")},
                    ],
                output="screen"
            )
        ]
    )

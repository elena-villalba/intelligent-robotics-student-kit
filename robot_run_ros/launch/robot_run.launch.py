"""Launch the ROS-enabled Robot Run simulator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    team_number = LaunchConfiguration("team_number")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="diff_robot"),
            DeclareLaunchArgument("team_number", default_value="0"),
            Node(
                package="robot_run_ros",
                executable="robot_run",
                name="robot_run",
                output="screen",
                parameters=[
                    {
                        "robot_name": robot_name,
                        "team_number": ParameterValue(
                            team_number,
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )


"""Starter launch file for the Phoenyx-I final project.

This file already launches Gazebo, loads the rover description, spawns the
robot and activates its controllers.

Student tasks:
    1. Add a launch argument to select the world from the terminal.
    2. Build the path to the selected world inside phoenyx_i_bringup/worlds.
    3. Start RViz2 using the configuration saved in phoenyx_i_bringup/rviz.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


# TODO (WORLD): Import the launch tools required to:
#   - declare a launch argument;
#   - read its value;
#   - construct a path using substitutions.
# Review the launch arguments introduced in Session 6.


def generate_launch_description():
    """Create the complete Phoenyx-I simulation launch sequence."""

    # ------------------------------------------------------------------
    # TODO 1 (WORLD): Declare a launch argument called "world_path".
    #
    # Requirements:
    #   - Give it one of the available Level 1 worlds as its default value.
    #   - Explain its purpose with a short description.
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # TODO 2 (WORLD): Construct the complete path to the selected world.
    #
    # The world files are installed in:
    #   phoenyx_i_bringup/worlds/
    #
    # The final path must use the value received through the "world_path"
    # launch argument. Do not hard-code one complete file path.
    # ------------------------------------------------------------------

    # Start Gazebo. Without the students' modification, Gazebo opens its
    # default empty world, which allows the starter file to be tested first.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",

            # TODO 3 (WORLD): Pass the selected world path to Gazebo here.

        }.items(),
    )

    # Load the Phoenyx-I robot description from the osr_gazebo package.
    osr_package_path = get_package_share_directory("osr_gazebo")
    xacro_file = os.path.join(
        osr_package_path,
        "urdf",
        "osr_simplified.urdf.xacro",
    )

    xacro_document = xacro.parse(open(xacro_file))
    xacro.process_doc(xacro_document)
    robot_description = {
        "robot_description": xacro_document.toxml()
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Spawn Phoenyx-I in Gazebo.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "phoenyx_i",
        ],
        output="screen",
    )

    # Start the rover controller node.
    controller_node = Node(
        package="osr_gazebo",
        executable="osr_controller",
        output="screen",
    )

    # Activate the ROS 2 controllers after the robot has been spawned.
    joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_controller",
        ],
        output="screen",
    )

    servo_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "servo_controller",
        ],
        output="screen",
    )

    load_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_controller,
                wheel_controller,
                servo_controller,
            ],
        )
    )

    # ------------------------------------------------------------------
    # TODO 4 (RVIZ): Construct the path to your RViz configuration.
    #
    # The configuration must be stored inside:
    #   phoenyx_i_bringup/rviz/
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # TODO 5 (RVIZ): Create the RViz2 Node action.
    #
    # RViz2 must open using the configuration path created above.
    # Review the RViz launch file developed in Session 3.
    # ------------------------------------------------------------------

    return LaunchDescription(
        [
            # TODO 6 (WORLD): Add the world launch argument here.

            gazebo,
            robot_state_publisher,
            spawn_entity,
            controller_node,
            load_controllers_after_spawn,

            # TODO 7 (RVIZ): Add the RViz2 node here.
        ]
    )

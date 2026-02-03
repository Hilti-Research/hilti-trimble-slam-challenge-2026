#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_name",
                default_value="floor_1_2025-05-05_run_1",
                description="Run name in format floor_X_YYYY-MM-DD_run_X",
            ),
            Node(
                name="groundtruth_server",
                package="challenge_tools_ros",
                executable="groundtruth_server.py",
                output="screen",
                arguments=[LaunchConfiguration("run_name")],
            ),
        ]
    )

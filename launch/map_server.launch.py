#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _make_nodes(context):
    run_name = LaunchConfiguration("run_name").perform(context)
    mask = LaunchConfiguration("mask").perform(context)

    parts = run_name.split("_")
    if len(parts) < 5 or parts[0] != "floor" or "run" not in parts:
        raise RuntimeError(
            "run_name must be in format floor_X_YYYY-MM-DD_run_X, got: "
            f"{run_name}"
        )

    floor_id = "_".join(parts[:2])
    filename = f"{floor_id}.png"

    share_dir = get_package_share_directory("challenge_tools_ros")
    map_path = os.path.join(share_dir, "floorplans", mask, filename)

    map_server_node = Node(
        name="map_server",
        package="challenge_tools_ros",
        executable="map_server.py",
        output="screen",
        arguments=[map_path, "0"],
    )

    static_tf_node = Node(
        name="static_transform_publisher",
        package="challenge_tools_ros",
        executable="static_transform_publisher.py",
        output="screen",
        arguments=[run_name],
    )

    return [map_server_node, static_tf_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_name",
                description="Run name in format floor_X_YYYY-MM-DD_run_X",
            ),
            DeclareLaunchArgument(
                "mask",
                default_value="masks_with_windows",
                description="Subfolder: masks_with_windows or masks_no_windows",
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )

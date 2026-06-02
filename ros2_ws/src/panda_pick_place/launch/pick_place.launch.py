"""Launch perception + executor with production parameters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("panda_pick_place")
    params = os.path.join(pkg, "config", "pick_place_params.yaml")

    return LaunchDescription([
        Node(
            package="panda_pick_place",
            executable="perception_node",
            name="perception_node",
            parameters=[params],
            output="screen",
        ),
        Node(
            package="panda_pick_place",
            executable="executor_node",
            name="pick_place_executor",
            parameters=[params],
            output="screen",
        ),
    ])

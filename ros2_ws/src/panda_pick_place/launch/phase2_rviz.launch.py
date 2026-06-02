"""Phase 2 (Gazebo unified): pick/place + optional moveit_servo.

Prerequisites (scripts/start_phase2_rviz.sh / start_phase2_unified.sh):
  - gazebo_pick_place.launch.py (desk + Franka in one Gazebo GUI)
  - move_group + spawn_controllers (no RViz demo)
  - static TF world -> panda_link0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("panda_pick_place")
    params = os.path.join(pkg, "config", "pick_place_params_rviz.yaml")

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "moveit_servo.launch.py")
        ),
    )

    perception = Node(
        package="panda_pick_place",
        executable="perception_node",
        name="perception_node",
        parameters=[params],
        output="screen",
    )

    executor = Node(
        package="panda_pick_place",
        executable="executor_node",
        name="pick_place_executor",
        parameters=[params],
        output="screen",
    )

    return LaunchDescription([servo, perception, executor])

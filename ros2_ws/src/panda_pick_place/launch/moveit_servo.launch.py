"""Launch moveit_servo with Panda MoveIt config."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory("panda_pick_place")
    servo_params = os.path.join(pkg, "config", "panda_moveit_servo.yaml")

    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="my_panda_moveit_config")
        .to_moveit_configs()
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([servo_node])

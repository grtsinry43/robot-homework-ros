"""Launch pick-and-place Gazebo scene with Franka arm + overhead RGB-D camera.

Requires vendor deps (franka_ros2) — run scripts/setup_vendor.sh first.

Phase 0 deliverable (DESIGN.md §7, §9):
  - Panda-class arm in Gazebo (official stack uses robot_type:=fer)
  - RGB-D camera bridged to /camera/color|depth/image_raw
  - Static TF: world -> camera_link -> camera_optical_frame

Not yet verified without Docker/Gazebo runtime.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("panda_sim_bringup")
    models_path = os.path.join(pkg_share, "models")
    bridge_config = os.path.join(pkg_share, "config", "ros_gz_bridge_camera.yaml")
    camera_urdf = os.path.join(pkg_share, "urdf", "overhead_camera_static.urdf")

    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="fer",
        description="Franka robot model (fer ≈ legacy Panda in official franka_ros2)",
    )
    load_gripper_arg = DeclareLaunchArgument("load_gripper", default_value="true")
    launch_franka_arg = DeclareLaunchArgument("launch_franka", default_value="true")
    launch_bridge_arg = DeclareLaunchArgument("launch_bridge", default_value="true")

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[models_path, ":", os.environ.get("GZ_SIM_RESOURCE_PATH", "")],
    )

    # Official Franka Gazebo bringup — joint position controller example spawns sim + controllers.
    franka_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("franka_gazebo_bringup"),
                "launch",
                "gazebo_joint_position_controller_example.launch.py",
            ])
        ),
        launch_arguments={
            "robot_type": LaunchConfiguration("robot_type"),
            "load_gripper": LaunchConfiguration("load_gripper"),
            "franka_hand": "franka_hand",
            # franka_gazebo_bringup accepts world filename under its own worlds/ dir;
            # we load our desk world via gz sim -r in a follow-up step once integrated.
            # TODO(phase0-verify): pass pick_place_desk.sdf once franka launch supports external world path.
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_franka")),
    )

    camera_tf = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="overhead_camera_tf",
        parameters=[{"robot_description": open(camera_urdf, encoding="utf-8").read()}],
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_camera_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_bridge")),
    )

    return LaunchDescription([
        robot_type_arg,
        load_gripper_arg,
        launch_franka_arg,
        launch_bridge_arg,
        gz_resource_path,
        franka_gazebo,
        camera_tf,
        ros_gz_bridge,
    ])

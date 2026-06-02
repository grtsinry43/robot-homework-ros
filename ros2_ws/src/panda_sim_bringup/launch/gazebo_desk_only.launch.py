"""Desk-only Gazebo world for camera/TF smoke test without Franka vendor deps.

With DISPLAY (Docker X11): starts simulation server (-s) then GUI client (-g) separately.
This avoids empty viewport / "Unable to deserialize sdf::Model" from combined ign gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("panda_sim_bringup")
    models_path = os.path.join(pkg_share, "models")
    worlds_path = os.path.join(pkg_share, "worlds", "pick_place_desk.sdf")
    bridge_config = os.path.join(pkg_share, "config", "ros_gz_bridge_camera.yaml")
    camera_urdf = os.path.join(pkg_share, "urdf", "overhead_camera_static.urdf")
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true" if os.environ.get("DISPLAY", "").strip() else "false",
        description="Start ign gazebo GUI client (-g) after server",
    )

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[models_path, ":", os.environ.get("GZ_SIM_RESOURCE_PATH", "")],
    )

    # Server only (-s): one sim, stable ROS topics; GUI attaches via separate -g process.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"{worlds_path} -r -s -v 3",
            "gz_version": "6",
        }.items(),
    )

    gz_gui = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=["ign", "gazebo", "-g", "--force-version", "6"],
                output="screen",
                shell=False,
            )
        ],
        condition=IfCondition(LaunchConfiguration("use_gui")),
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
    )

    return LaunchDescription([
        use_gui_arg,
        gz_resource_path,
        gz_sim,
        gz_gui,
        camera_tf,
        ros_gz_bridge,
    ])

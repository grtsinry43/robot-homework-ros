"""Pick-and-place Gazebo: desk world + Franka arm + overhead RGB-D bridge.

Requires franka_ros2 vendor (scripts/setup_vendor.sh).

World merge: pass pick_place_desk.sdf to Franka bringup via gz_args (replaces empty.sdf).
Camera is inlined in the world SDF; ros_gz_bridge maps /overhead_camera/* → /camera/*.
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("panda_sim_bringup")
    models_path = os.path.join(pkg_share, "models")
    worlds_path = os.path.join(pkg_share, "worlds", "pick_place_desk.sdf")
    bridge_config = os.path.join(pkg_share, "config", "ros_gz_bridge_camera.yaml")

    franka_desc_share = get_package_share_directory("franka_description")
    franka_resource = os.path.dirname(franka_desc_share)
    gz_resource = ":".join(
        p for p in (models_path, franka_resource, os.environ.get("GZ_SIM_RESOURCE_PATH", "")) if p
    )
    os.environ["GZ_SIM_RESOURCE_PATH"] = gz_resource

    robot_type_arg = DeclareLaunchArgument("robot_type", default_value="fer")
    load_gripper_arg = DeclareLaunchArgument("load_gripper", default_value="true")
    launch_franka_arg = DeclareLaunchArgument("launch_franka", default_value="true")
    launch_bridge_arg = DeclareLaunchArgument("launch_bridge", default_value="true")
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true" if os.environ.get("DISPLAY", "").strip() else "false",
    )
    # Franka example controller; override if your MoveIt Gazebo controllers differ.
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="gravity_compensation_example_controller",
    )

    gz_resource_path = SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_resource)

    # Server-only Gazebo (-s) + desk world; GUI attaches via separate ign -g (see gazebo_desk_only).
    gz_args_value = f"{worlds_path} -r -s -v 3"

    franka_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("franka_gazebo_bringup"),
                "launch",
                "gazebo_franka_arm_example_controller.launch.py",
            ])
        ),
        launch_arguments={
            "robot_type": LaunchConfiguration("robot_type"),
            "load_gripper": LaunchConfiguration("load_gripper"),
            "franka_hand": "franka_hand",
            "gz_args": gz_args_value,
            "rviz": "false",
            "controller": LaunchConfiguration("controller"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_franka")),
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

    camera_static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "camera_static_tf.launch.py")
        ),
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
        use_gui_arg,
        controller_arg,
        gz_resource_path,
        franka_gazebo,
        gz_gui,
        camera_static_tf,
        ros_gz_bridge,
    ])

"""Gazebo pick-place: desk world + Panda (gz_ros2_control) + overhead camera.

MoveIt drives panda_arm_controller inside the same simulation as desk/objects.
"""

import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _link_mass_kg(link_name: str) -> float:
    """Rough Franka-like masses; moveit_resources URDF has no inertial tags."""
    if link_name == "world":
        return 0.0
    if link_name == "panda_link0":
        return 4.0
    if link_name in {"panda_hand", "panda_leftfinger", "panda_rightfinger"}:
        return 0.5
    return 2.0


def _inject_link_inertials(urdf_xml: str) -> str:
    """moveit_resources Panda URDF has no <inertial>; Gazebo refuses spawn without them."""
    root = ET.fromstring(urdf_xml)
    for link in root.findall("link"):
        link_name = link.get("name", "")
        if link_name == "world" or link.find("inertial") is not None:
            continue
        mass_kg = _link_mass_kg(link_name)
        if mass_kg <= 0.0:
            continue
        mass = ET.SubElement(link, "inertial")
        origin = ET.SubElement(mass, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", "0 0 0")
        m = ET.SubElement(mass, "mass")
        m.set("value", str(mass_kg))
        inertia = ET.SubElement(mass, "inertia")
        i = mass_kg * 0.05
        inertia.set("ixx", str(i))
        inertia.set("iyy", str(i))
        inertia.set("izz", str(i))
        inertia.set("ixy", "0")
        inertia.set("ixz", "0")
        inertia.set("iyz", "0")
    return ET.tostring(root, encoding="unicode")


def _resolve_package_uris(urdf_xml: str) -> str:
    """Gazebo cannot resolve package:// — use file:// so arm meshes show in GUI."""
    panda_desc = get_package_share_directory("moveit_resources_panda_description")
    return urdf_xml.replace(
        "package://moveit_resources_panda_description",
        f"file://{panda_desc}",
    )


def _robot_description() -> dict:
    pkg = get_package_share_directory("panda_sim_bringup")
    xacro_path = os.path.join(pkg, "urdf", "panda_gz.urdf.xacro")
    doc = xacro.process_file(xacro_path)
    xml = _inject_link_inertials(doc.toxml())
    return {"robot_description": _resolve_package_uris(xml)}


def generate_launch_description():
    pkg_share = get_package_share_directory("panda_sim_bringup")
    models_path = os.path.join(pkg_share, "models")
    worlds_path = os.path.join(pkg_share, "worlds", "pick_place_desk.sdf")
    bridge_config = os.path.join(pkg_share, "config", "ros_gz_bridge_camera.yaml")
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    controllers_yaml = os.path.join(pkg_share, "config", "panda_gz_ros2_control.yaml")

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true" if os.environ.get("DISPLAY", "").strip() else "false",
    )
    robot_x_arg = DeclareLaunchArgument("robot_x", default_value="0.0")
    robot_y_arg = DeclareLaunchArgument("robot_y", default_value="0.0")
    robot_z_arg = DeclareLaunchArgument("robot_z", default_value="0.0")

    panda_desc_share = get_package_share_directory("moveit_resources_panda_description")
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[models_path, ":", panda_desc_share, ":", os.environ.get("GZ_SIM_RESOURCE_PATH", "")],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[_robot_description()],
    )

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

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "panda",
            "-x", LaunchConfiguration("robot_x"),
            "-y", LaunchConfiguration("robot_y"),
            "-z", LaunchConfiguration("robot_z"),
        ],
        output="screen",
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    spawn_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "panda_arm_controller",
            "panda_gripper_controller",
            "--controller-manager-timeout", "45",
        ],
        parameters=[controllers_yaml],
        output="screen",
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
    )

    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_set_pose_bridge",
        arguments=[
            "/world/pick_place_desk/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        output="screen",
    )

    # DetachableJoint attach/detach triggers (ROS std_msgs/Empty -> gz ignition.msgs.Empty).
    # gazebo_gripper_sim.py publishes here to create/remove the physical grasp joint.
    grasp_models = ["red_block_01", "green_block_01"]
    grasp_bridge_args = []
    for model in grasp_models:
        grasp_bridge_args.append(
            f"/grasp/{model}/attach@std_msgs/msg/Empty]ignition.msgs.Empty"
        )
        grasp_bridge_args.append(
            f"/grasp/{model}/detach@std_msgs/msg/Empty]ignition.msgs.Empty"
        )
    grasp_joint_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_grasp_joint_bridge",
        arguments=grasp_bridge_args,
        output="screen",
    )

    gripper_joint_state_merger = Node(
        package="panda_sim_bringup",
        executable="gripper_joint_state_merger.py",
        output="screen",
    )

    gazebo_gripper_sim = Node(
        package="panda_sim_bringup",
        executable="gazebo_gripper_sim.py",
        output="screen",
    )

    delayed_gripper_sim = TimerAction(period=8.0, actions=[gazebo_gripper_sim])

    return LaunchDescription([
        use_gui_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        gz_resource_path,
        robot_state_publisher,
        gz_sim,
        gz_gui,
        delayed_spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[spawn_controllers, delayed_gripper_sim],
            )
        ),
        camera_static_tf,
        ros_gz_bridge,
        set_pose_bridge,
        grasp_joint_bridge,
        gripper_joint_state_merger,
    ])

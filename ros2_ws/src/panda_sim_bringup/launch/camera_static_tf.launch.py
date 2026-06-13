"""Static eye-in-world camera TFs for the Gazebo RGB-D camera."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # True top-down camera at table center (matches pick_place_desk.sdf overhead_camera
    # pose 0.38 0 1.05, optical axis straight down). Optical-frame axes (derived from the
    # actual rendered image): +X(image-right) -> world -Y, +Y(image-down) -> world -X,
    # +Z(view dir) -> world -Z. That rotation is quat xyzw (0.7071,-0.7071,0,0).
    # NOTE: the REAL camera TF comes from THIS publisher, not overhead_camera_static.urdf;
    # keep the two in sync.
    camera_pose_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="desk_camera_static_tf",
        arguments=[
            "--x", "0.38",
            "--y", "0",
            "--z", "1.05",
            "--qx", "0.7071",
            "--qy", "-0.7071",
            "--qz", "0",
            "--qw", "0",
            "--frame-id", "panda_link0",
            "--child-frame-id", "overhead_camera/camera_link/rgbd",
        ],
        output="screen",
    )

    # The Gazebo bridge stamps images with the sensor frame; keep the ROS optical
    # frame expected by perception config and verification available as an alias.
    optical_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_camera_optical_alias_tf",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",
            "--frame-id", "overhead_camera/camera_link/rgbd",
            "--child-frame-id", "camera_optical_frame",
        ],
        output="screen",
    )

    return LaunchDescription([
        camera_pose_tf,
        optical_alias_tf,
    ])

"""Static eye-in-world camera TFs for the Gazebo RGB-D camera."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Composed from pick_place_desk.sdf pose + ROS optical rotation (matches tf2_echo).
    camera_pose_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="desk_camera_static_tf",
        arguments=[
            "--x", "0.45",
            "--y", "0",
            "--z", "1.05",
            "--qx", "0.692",
            "--qy", "-0.692",
            "--qz", "0.148",
            "--qw", "-0.148",
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

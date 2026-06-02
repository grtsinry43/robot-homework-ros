"""Static eye-in-world camera TF: panda_link0 -> Gazebo RGB-D optical frame."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Composed from pick_place_desk.sdf pose + ROS optical rotation (matches tf2_echo).
    return LaunchDescription([
        Node(
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
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mono_camera_capture"), "config", "mono_camera.yaml"]
                ),
                description="Path to monocular USB camera parameter file.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
                description="Published image topic for YOLO input.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera_info",
                description="Published camera info topic.",
            ),
            Node(
                package="mono_camera_capture",
                executable="mono_camera_node",
                name="mono_camera_node",
                output="screen",
                parameters=[params_file],
                remappings=[
                    ("image_raw", image_topic),
                    ("camera_info", camera_info_topic),
                ],
            ),
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission_config_arg = DeclareLaunchArgument(
        "mission_config",
        default_value="/home/banana/detect_ws/src/mavros_offboard/config/robocup_v1_params.yaml",
    )
    servo_config_arg = DeclareLaunchArgument(
        "servo_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("servo_controller"), "config", "servo_waypoints.yaml"]
        ),
    )
    dry_run_arg = DeclareLaunchArgument("dry_run", default_value="false")

    servo_driver = Node(
        package="servo_controller",
        executable="servo_driver_node",
        name="servo_driver_node",
        output="screen",
        parameters=[
            LaunchConfiguration("servo_config"),
            {"dry_run": LaunchConfiguration("dry_run")},
        ],
    )

    node = Node(
        package="mavros_offboard",
        executable="robocup_v1",
        name="mavros_offboard_node",
        output="screen",
        parameters=[
            LaunchConfiguration("mission_config"),
        ],
    )

    return LaunchDescription(
        [
            mission_config_arg,
            servo_config_arg,
            dry_run_arg,
            servo_driver,
            node,
        ]
    )

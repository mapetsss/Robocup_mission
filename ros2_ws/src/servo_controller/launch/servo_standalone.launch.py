from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("servo_controller"), "config", "servo_waypoints.yaml"]
        ),
    )

    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="true 时只打印舵机动作，不写 I2C/PWM",
    )

    servo_driver = Node(
        package="servo_controller",
        executable="servo_driver_node",
        name="servo_driver_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"dry_run": LaunchConfiguration("dry_run")},
        ],
    )

    waypoint_task = Node(
        package="servo_controller",
        executable="servo_waypoint_task",
        name="servo_waypoint_task",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, dry_run_arg, servo_driver, waypoint_task])

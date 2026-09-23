from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    servo_config_arg = DeclareLaunchArgument(
        "servo_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("servo_controller"), "config", "servo_waypoints.yaml"]
        ),
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="true 时只打印舵机动作，不写 I2C/PWM",
    )

    pose_timeout_arg = DeclareLaunchArgument("pose_timeout_s", default_value="1.0")
    reach_tolerance_arg = DeclareLaunchArgument("reach_tolerance_m", default_value="0.1")
    hover_duration_arg = DeclareLaunchArgument("hover_duration_s", default_value="1.0")

    hover_x_arg = DeclareLaunchArgument(
        "hover_x",
        default_value="0.0",
        description="起飞悬停点 X，相对任务原点",
    )
    hover_y_arg = DeclareLaunchArgument(
        "hover_y",
        default_value="0.0",
        description="起飞悬停点 Y，相对任务原点",
    )
    hover_z_arg = DeclareLaunchArgument(
        "hover_z",
        default_value="0.5",
        description="起飞悬停高度 Z，相对任务原点",
    )
    left_x_arg = DeclareLaunchArgument(
        "left_x",
        default_value="0.0",
        description="左侧指定点 X，相对任务原点",
    )
    left_y_arg = DeclareLaunchArgument(
        "left_y",
        default_value="-0.5",
        description="左侧指定点 Y，相对任务原点",
    )
    left_z_arg = DeclareLaunchArgument(
        "left_z",
        default_value="0.5",
        description="左侧指定点 Z，相对任务原点",
    )

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

    offboard_node = Node(
        package="mavros_offboard",
        executable="mavros_offboard_node",
        name="mavros_offboard_node",
        output="screen",
        parameters=[
            {
                "pose_timeout_s": LaunchConfiguration("pose_timeout_s"),
                "reach_tolerance_m": LaunchConfiguration("reach_tolerance_m"),
                "hover_duration_s": LaunchConfiguration("hover_duration_s"),
                "hover_x": LaunchConfiguration("hover_x"),
                "hover_y": LaunchConfiguration("hover_y"),
                "hover_z": LaunchConfiguration("hover_z"),
                "left_x": LaunchConfiguration("left_x"),
                "left_y": LaunchConfiguration("left_y"),
                "left_z": LaunchConfiguration("left_z"),
            }
        ],
    )

    return LaunchDescription(
        [
            servo_config_arg,
            dry_run_arg,
            pose_timeout_arg,
            reach_tolerance_arg,
            hover_duration_arg,
            hover_x_arg,
            hover_y_arg,
            hover_z_arg,
            left_x_arg,
            left_y_arg,
            left_z_arg,
            servo_driver,
            offboard_node,
        ]
    )

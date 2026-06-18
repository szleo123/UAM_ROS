from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from arm_moveit_config_utils.launch_utils import (
    common_bringup_launch_arguments,
    forward_common_bringup_arguments,
)


def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_moveit_config"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments=forward_common_bringup_arguments().items(),
    )

    geomagic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_geomagic_teleop"),
                    "launch",
                    "geomagic_teleop.launch.py",
                ]
            )
        ),
        launch_arguments={
            "dry_run": LaunchConfiguration("teleop_dry_run"),
            "require_homing": LaunchConfiguration("teleop_require_homing"),
            "armed_on_start": LaunchConfiguration("teleop_armed_on_start"),
            "emergency_stop_service": LaunchConfiguration("arm_emergency_stop_service"),
            "start_geomagic_driver": LaunchConfiguration("start_geomagic_driver"),
            "start_trajectory_command_monitor": LaunchConfiguration(
                "start_trajectory_command_monitor"
            ),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_geomagic_teleop_stack")),
    )

    return LaunchDescription(
        common_bringup_launch_arguments(
            default_use_fake_hardware="false",
            default_start_rviz="true",
            default_start_move_group="true",
            default_start_controllers="true",
            default_start_handeye="false",
            default_start_homing_button="true",
        )
        + [
            DeclareLaunchArgument(
                "start_geomagic_teleop_stack",
                default_value="false",
                description="Also start the original all-in-one Geomagic teleop stack.",
            ),
            DeclareLaunchArgument(
                "start_geomagic_driver",
                default_value="true",
                description="Forwarded to the original Geomagic teleop launch.",
            ),
            DeclareLaunchArgument(
                "teleop_dry_run",
                default_value="false",
                description="Route original teleop stack output to dry-run topic.",
            ),
            DeclareLaunchArgument(
                "teleop_require_homing",
                default_value="true",
                description="Require homing for original teleop stack motion.",
            ),
            DeclareLaunchArgument(
                "teleop_armed_on_start",
                default_value="false",
                description="Start original teleop trajectory gate armed.",
            ),
            DeclareLaunchArgument(
                "start_trajectory_command_monitor",
                default_value="false",
                description="Start final command monitor in original teleop launch.",
            ),
            bringup_launch,
            geomagic_launch,
        ]
    )

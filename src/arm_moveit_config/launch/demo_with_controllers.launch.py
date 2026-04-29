from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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

    return LaunchDescription(
        common_bringup_launch_arguments(
            default_use_fake_hardware="false",
            default_start_handeye="true",
        )
        + [bringup_launch]
    )

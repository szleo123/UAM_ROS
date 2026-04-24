from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from arm_moveit_config_utils.launch_utils import (
    build_moveit_config,
    common_bringup_launch_arguments,
    forward_common_bringup_arguments,
    robot_description_mappings,
)


def generate_launch_description():
    moveit_config = build_moveit_config(robot_description_mappings())

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_moveit_config"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments=forward_common_bringup_arguments().items(),
    )

    tracker = Node(
        package="arm_realtime_tracker",
        executable="pose_tracker_node",
        output="screen",
        parameters=[
            # MoveIt/robot params required by MoveGroupInterface
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,                 # ompl/chomp/pilz if present
            moveit_config.trajectory_execution,               # optional
            moveit_config.joint_limits,                       # optional
            # Tracker-specific params (tune as you like)
            {"arm_group_name": "arm"},
            {"gripper_group_name": "gripper"},
            {"ee_link": "tool0"},
            {"target_frame": "base_link"},
            {"target_topic": "/desired_pose"},
            {"velocity_scale": 0.5},
            {"accel_scale": 0.5},
            {"planning_time": 0.5},
            {"exec_period_ms": 100},
            {"deadband_pos_m": 0.1},
            {"deadband_rot_rad": 0.3},
            {"target_timeout_sec": 0.3},
            {"approach_offset_m": 0.20},
            {"gripper_joint_name": "tool_joint"},
        ],
    )

    return LaunchDescription(
        common_bringup_launch_arguments(
            default_use_fake_hardware="true",
            default_start_handeye="true",
        )
        + [
            bringup_launch,
            TimerAction(period=2.0, actions=[tracker]),
        ]
    )

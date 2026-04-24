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
    load_servo_parameters,
    robot_description_mappings,
)

def generate_launch_description():
    moveit_config = build_moveit_config(
        robot_description_mappings(), pipelines=["ompl"]
    )
    servo_params = load_servo_parameters()

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_moveit_config"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments=forward_common_bringup_arguments().items(),
    )

    # Launch Servo server; its default topic namespace matches the executable name ("servo_server")
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            servo_params,
        ],
        output="screen",
    )

    # Your Pose→Servo bridge. Make sure these parameter names match your C++ node.
    pose_servo_tracker = Node(
        package="arm_realtime_tracker",
        executable="pose_servo_tracker_node",
        parameters=[
            {"group_name": "arm"},
            {"ee_link": "tool0"},
            {"target_frame": "base_link"},
            {"target_topic": "/desired_pose"},
            {"servo_twist_topic": "/servo_node/delta_twist_cmds"},
            {"servo_enable_topic": "/servo_node/enable"},
            {"kp_linear": 2.0},
            {"kp_angular": 2.0},
            {"pos_deadband_m": 0.003},
            {"ang_deadband_rad": 0.02},
            {"max_linear_vel": 0.25},
            {"max_angular_vel": 0.6},
            {"loop_rate_hz": 200.0},
        ],
        output="screen",
    )

    return LaunchDescription(
        common_bringup_launch_arguments(
            default_use_fake_hardware="true",
            default_start_handeye="true",
        )
        + [
            bringup_launch,
            TimerAction(period=2.0, actions=[servo_node, pose_servo_tracker]),
        ]
    )

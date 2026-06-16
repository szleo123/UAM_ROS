from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from arm_moveit_config_utils.launch_utils import (
    build_moveit_config,
    common_bringup_launch_arguments,
    forward_common_bringup_arguments,
    load_servo_parameters,
    robot_description_mappings,
)


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )


def _jetson_runtime_nodes(context, *args, **kwargs):
    teleop_config = LaunchConfiguration("teleop_config").perform(context)
    require_homing = _as_bool(context, "require_homing")
    require_deadman = _as_bool(context, "require_deadman")
    dry_run = _as_bool(context, "dry_run")
    armed_on_start = _as_bool(context, "armed_on_start")
    output_trajectory_topic = (
        "/arm_teleop/dry_run_joint_trajectory"
        if dry_run
        else "/arm_controller/joint_trajectory"
    )

    moveit_config = build_moveit_config(
        robot_description_mappings(), pipelines=["ompl"]
    )
    servo_params = load_servo_parameters()
    servo_yaml = dict(servo_params["moveit_servo"])
    servo_yaml["cartesian_command_in_topic"] = "/arm_teleop/twist_cmd"
    servo_yaml["command_out_topic"] = "/arm_teleop/servo_raw_joint_trajectory"
    servo_yaml["robot_link_command_frame"] = "uav_mount"
    servo_yaml["ee_frame_name"] = "tool0"
    servo_yaml["check_collisions"] = _as_bool(context, "check_collisions")
    servo_params = {"moveit_servo": servo_yaml}

    return [
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="moveit_servo",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                servo_params,
            ],
            condition=IfCondition(LaunchConfiguration("start_servo")),
        ),
        Node(
            package="arm_geomagic_teleop",
            executable="teleop_safety_filter",
            name="teleop_safety_filter",
            output="screen",
            parameters=[
                teleop_config,
                {
                    "require_homing": require_homing,
                    "require_deadman": require_deadman,
                },
            ],
            condition=IfCondition(LaunchConfiguration("start_safety_filter")),
        ),
        Node(
            package="arm_geomagic_teleop",
            executable="trajectory_deadman_gate",
            name="trajectory_deadman_gate",
            output="screen",
            parameters=[
                teleop_config,
                {
                    "require_homing": require_homing,
                    "require_deadman": require_deadman,
                    "armed_on_start": armed_on_start,
                    "output_trajectory_topic": output_trajectory_topic,
                },
            ],
            condition=IfCondition(LaunchConfiguration("start_trajectory_gate")),
        ),
        Node(
            package="arm_geomagic_teleop",
            executable="trajectory_command_monitor",
            name="trajectory_command_monitor",
            output="screen",
            parameters=[
                teleop_config,
                {
                    "trajectory_topic": output_trajectory_topic,
                },
            ],
            condition=IfCondition(LaunchConfiguration("start_trajectory_command_monitor")),
        ),
    ]


def generate_launch_description():
    default_teleop_config = PathJoinSubstitution(
        [FindPackageShare("arm_geomagic_teleop"), "config", "geomagic_teleop.yaml"]
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_moveit_config"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments=forward_common_bringup_arguments().items(),
    )

    d405_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_moveit_config"), "launch", "d405.launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "camera_namespace": LaunchConfiguration("camera_namespace"),
            "serial_no": LaunchConfiguration("camera_serial_no"),
            "upside_down": LaunchConfiguration("camera_upside_down"),
            "publish_tf": LaunchConfiguration("camera_publish_tf"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_d405")),
    )

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros2_aruco"), "launch", "aruco_recognition.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_aruco")),
    )

    start_servo_services = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/moveit_servo/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/moveit_servo/unpause_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/moveit_servo/change_control_dimensions",
                    "moveit_msgs/srv/ChangeControlDimensions",
                    (
                        "{control_x_translation: true, control_y_translation: true, "
                        "control_z_translation: true, control_x_rotation: true, "
                        "control_y_rotation: true, control_z_rotation: true}"
                    ),
                ],
                output="screen",
            ),
        ],
        condition=IfCondition(LaunchConfiguration("start_servo")),
    )

    split_args = [
        DeclareLaunchArgument(
            "teleop_config",
            default_value=default_teleop_config,
            description="Shared Geomagic teleop YAML used by Jetson-side safety nodes.",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="true",
            description="Start Jetson-local MoveIt Servo for split-mode teleop.",
        ),
        DeclareLaunchArgument(
            "start_safety_filter",
            default_value="true",
            description="Start Jetson-local teleop twist safety filter.",
        ),
        DeclareLaunchArgument(
            "start_trajectory_gate",
            default_value="true",
            description="Start final Jetson-local trajectory gate before ros2_control.",
        ),
        DeclareLaunchArgument(
            "start_trajectory_command_monitor",
            default_value="false",
            description="Plot or inspect final teleop trajectory commands on Jetson.",
        ),
        DeclareLaunchArgument(
            "require_homing",
            default_value="true",
            description="Require /arm_homing/state == ALL_READY before teleop motion.",
        ),
        DeclareLaunchArgument(
            "require_deadman",
            default_value="true",
            description="Require the remote Geomagic deadman button for teleop motion.",
        ),
        DeclareLaunchArgument(
            "armed_on_start",
            default_value="false",
            description="Start the final trajectory gate armed. Keep false for first tests.",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="false",
            description="Route gated Servo output to a dry-run topic instead of the arm controller.",
        ),
        DeclareLaunchArgument(
            "check_collisions",
            default_value="false",
            description="Enable MoveIt Servo collision checking.",
        ),
        DeclareLaunchArgument(
            "start_d405",
            default_value="false",
            description="Start the Jetson-connected RealSense D405 wrapper.",
        ),
        DeclareLaunchArgument(
            "start_aruco",
            default_value="false",
            description="Start patched ros2_aruco recognition for the D405 stream.",
        ),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("camera_serial_no", default_value=""),
        DeclareLaunchArgument("camera_upside_down", default_value="false"),
        DeclareLaunchArgument("camera_publish_tf", default_value="false"),
    ]

    return LaunchDescription(
        common_bringup_launch_arguments(
            default_use_fake_hardware="false",
            default_start_rviz="false",
            default_start_move_group="true",
            default_start_controllers="true",
            default_start_handeye="false",
            default_start_homing_button="false",
            default_start_mit_gain_tuner="false",
            default_start_dynamics_monitor="true",
            default_dynamics_monitor_plot="false",
            default_start_joint_feedback_monitor="false",
            default_joint_feedback_monitor_plot="false",
        )
        + split_args
        + [
            bringup_launch,
            d405_launch,
            aruco_launch,
            OpaqueFunction(function=_jetson_runtime_nodes),
            start_servo_services,
        ]
    )

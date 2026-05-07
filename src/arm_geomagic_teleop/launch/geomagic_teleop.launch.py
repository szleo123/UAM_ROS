from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from arm_moveit_config_utils.launch_utils import build_moveit_config, load_servo_parameters


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ("1", "true", "yes", "on")


def _setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("teleop_config").perform(context)
    require_homing = _as_bool(context, "require_homing")
    dry_run = _as_bool(context, "dry_run")
    armed_on_start = _as_bool(context, "armed_on_start")
    output_trajectory_topic = (
        "/arm_teleop/dry_run_joint_trajectory" if dry_run else "/arm_controller/joint_trajectory"
    )

    moveit_config = build_moveit_config(pipelines=["ompl"])
    servo_params = load_servo_parameters()
    servo_yaml = dict(servo_params["moveit_servo"])
    servo_yaml["cartesian_command_in_topic"] = "/arm_teleop/twist_cmd"
    servo_yaml["command_out_topic"] = "/arm_teleop/servo_raw_joint_trajectory"
    servo_yaml["robot_link_command_frame"] = "uav_mount"
    servo_yaml["ee_frame_name"] = "tool0"
    servo_yaml["check_collisions"] = _as_bool(context, "check_collisions")
    servo_params = {"moveit_servo": servo_yaml}

    nodes = [
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
        ),
        Node(
            package="arm_geomagic_teleop",
            executable="geomagic_cartesian_teleop",
            name="geomagic_cartesian_teleop",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="arm_geomagic_teleop",
            executable="teleop_safety_filter",
            name="teleop_safety_filter",
            output="screen",
            parameters=[
                config_file,
                {
                    "require_homing": require_homing,
                },
            ],
        ),
        Node(
            package="arm_geomagic_teleop",
            executable="trajectory_deadman_gate",
            name="trajectory_deadman_gate",
            output="screen",
            parameters=[
                config_file,
                {
                    "require_homing": require_homing,
                    "armed_on_start": armed_on_start,
                    "output_trajectory_topic": output_trajectory_topic,
                },
            ],
        ),
    ]

    return nodes


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("arm_geomagic_teleop"), "config", "geomagic_teleop.yaml"]
    )

    geomagic_driver_node = Node(
        package="omni_common",
        executable="omni_state",
        name="geomagic_omni_state",
        output="screen",
        parameters=[
            {"omni_name": "phantom"},
            {"publish_rate": 1000},
            {"reference_frame": "/map"},
            {"units": "mm"},
        ],
        condition=IfCondition(LaunchConfiguration("start_geomagic_driver")),
    )

    adapter_node = Node(
        package="arm_geomagic_teleop",
        executable="geomagic_omni_state_adapter",
        name="geomagic_omni_state_adapter",
        output="screen",
        parameters=[LaunchConfiguration("teleop_config")],
        condition=IfCondition(LaunchConfiguration("use_omni_adapter")),
    )

    start_servo = TimerAction(
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
                        "control_z_translation: true, control_x_rotation: false, "
                        "control_y_rotation: false, control_z_rotation: false}"
                    ),
                ],
                output="screen",
            ),
        ],
        condition=IfCondition(LaunchConfiguration("start_servo")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_geomagic_driver",
                default_value="true",
                description="Start omni_common/omni_state to publish /phantom/state.",
            ),
            DeclareLaunchArgument(
                "teleop_config",
                default_value=default_config,
                description="YAML file for Geomagic teleop nodes.",
            ),
            DeclareLaunchArgument(
                "use_omni_adapter",
                default_value="true",
                description="Start adapter from /phantom/state to /geomagic_touch topics.",
            ),
            DeclareLaunchArgument(
                "require_homing",
                default_value="true",
                description="Require /arm_homing/state == 4 before motion reaches Servo/gate.",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="Route gated Servo trajectories to /arm_teleop/dry_run_joint_trajectory.",
            ),
            DeclareLaunchArgument(
                "armed_on_start",
                default_value="false",
                description="Start final trajectory gate armed. Keep false for first real tests.",
            ),
            DeclareLaunchArgument(
                "start_servo",
                default_value="true",
                description="Call MoveIt Servo start/unpause services after launch.",
            ),
            DeclareLaunchArgument(
                "check_collisions",
                default_value="false",
                description="Enable MoveIt Servo collision checking.",
            ),
            geomagic_driver_node,
            adapter_node,
            OpaqueFunction(function=_setup),
            start_servo,
        ]
    )

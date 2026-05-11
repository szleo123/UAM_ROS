from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


DEFAULT_PLANNING_PIPELINES = [
    "ompl",
    "chomp",
    "pilz_industrial_motion_planner",
]

HARDWARE_ARGUMENT_NAMES = [
    "use_fake_hardware",
    "initial_positions_file",
    "serial_port",
    "reader_port",
    "baudrate",
    "position_scale",
    "arm_joint_signs",
    "arm_joint_offsets",
    "hw_slowdown",
    "initial_read_timeout_sec",
    "first_power_on",
    "ros2_controllers_file",
]

GRIPPER_ARGUMENT_NAMES = [
    "aux_joint_min",
    "aux_joint_max",
    "gripper_port",
    "gripper_baudrate",
    "start_gripper_controller",
]

DYNAMICS_ARGUMENT_NAMES = [
    "arm_command_frame_format",
    "enable_dynamics_feedforward",
    "dynamics_mode",
    "dynamics_urdf_path",
    "dynamics_feedforward_source",
    "dynamics_feedforward_topic",
    "dynamics_topic_timeout_sec",
    "dynamics_use_commanded_position",
    "dynamics_torque_scale",
    "dynamics_torque_limits_nm",
    "dynamics_torque_joint_signs",
    "dynamics_torque_low_pass_alpha",
    "start_dynamics_monitor",
    "dynamics_monitor_rate_hz",
    "dynamics_monitor_mode",
    "dynamics_monitor_plot",
    "dynamics_monitor_plot_history_sec",
]

JOINT_FEEDBACK_MONITOR_ARGUMENT_NAMES = [
    "start_joint_feedback_monitor",
    "joint_feedback_monitor_joints",
    "joint_feedback_monitor_rate_hz",
    "joint_feedback_monitor_plot",
    "joint_feedback_monitor_history_sec",
    "joint_feedback_monitor_min_rad",
    "joint_feedback_monitor_max_rad",
    "joint_feedback_monitor_csv_enabled",
    "joint_feedback_monitor_csv_file",
    "joint_feedback_monitor_csv_append",
]

UI_ARGUMENT_NAMES = [
    "rviz_config",
    "start_rviz",
    "start_move_group",
    "start_controllers",
    "start_homing_button",
]

HANDEYE_ARGUMENT_NAMES = [
    "start_handeye",
    "handeye_name",
    "handeye_calibration_file",
]

COMMON_BRINGUP_ARGUMENT_NAMES = (
    HARDWARE_ARGUMENT_NAMES
    + GRIPPER_ARGUMENT_NAMES
    + DYNAMICS_ARGUMENT_NAMES
    + JOINT_FEEDBACK_MONITOR_ARGUMENT_NAMES
    + UI_ARGUMENT_NAMES
    + HANDEYE_ARGUMENT_NAMES
)


def _share_path(*parts):
    return PathJoinSubstitution([FindPackageShare("arm_moveit_config"), *parts])


def common_bringup_launch_arguments(
    *,
    default_use_fake_hardware="true",
    default_start_rviz="true",
    default_start_move_group="true",
    default_start_controllers="true",
    default_start_gripper_controller="true",
    default_start_handeye="false",
    default_start_homing_button="true",
    default_first_power_on="true",
):
    hardware_args = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value=default_use_fake_hardware,
            description="Use mock ros2_control hardware instead of the real arm interface.",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=_share_path("config", "initial_positions.yaml"),
            description="Initial joint positions YAML used by ros2_control.",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB1",
            description="Serial writer port for the real arm interface.",
        ),
        DeclareLaunchArgument(
            "reader_port",
            default_value="/dev/ttyUSB0",
            description="Serial reader port for the real arm interface.",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="230400",
            description="Baudrate for the arm serial ports.",
        ),
        DeclareLaunchArgument(
            "position_scale",
            default_value="1000",
            description="Hardware position scaling factor.",
        ),
        DeclareLaunchArgument(
            "arm_joint_signs",
            default_value="-1,1,-1,1,1,1",
            description="Comma-separated signs mapping ROS arm joint positions to hardware positions.",
        ),
        DeclareLaunchArgument(
            "arm_joint_offsets",
            default_value="0,0,0,0,0,0",
            # joint2: -0.031
            description="Comma-separated ROS joint offsets in radians added to arm hardware feedback.",
        ),
        DeclareLaunchArgument(
            "hw_slowdown",
            default_value="10",
            description="Hardware slowdown factor passed to the ros2_control plugin.",
        ),
        DeclareLaunchArgument(
            "initial_read_timeout_sec",
            default_value="2.0",
            description="How long to wait for initial hardware feedback before enabling writes.",
        ),
        DeclareLaunchArgument(
            "first_power_on",
            default_value=default_first_power_on,
            description=(
                "Whether this startup follows a full power cycle and must run the "
                "ROS-master homing handshake before normal motion."
            ),
        ),
        DeclareLaunchArgument(
            "ros2_controllers_file",
            default_value=_share_path("config", "ros2_controllers.yaml"),
            description="Controller manager YAML file.",
        ),
    ]

    gripper_args = [
        DeclareLaunchArgument(
            "aux_joint_min",
            default_value="-0.69",
            description="Minimum position for the auxiliary gripper/tool joint.",
        ),
        DeclareLaunchArgument(
            "aux_joint_max",
            default_value="0.0",
            description="Maximum position for the auxiliary gripper/tool joint.",
        ),
        DeclareLaunchArgument(
            "gripper_port",
            default_value="/dev/ttyUSB1",
            description="Serial port for the gripper or auxiliary joint controller.",
        ),
        DeclareLaunchArgument(
            "gripper_baudrate",
            default_value="115200",
            description="Baudrate for the gripper serial port.",
        ),
        DeclareLaunchArgument(
            "start_gripper_controller",
            default_value=default_start_gripper_controller,
            description="Spawn the gripper action controller.",
        ),
    ]

    dynamics_args = [
        DeclareLaunchArgument(
            "arm_command_frame_format",
            default_value="legacy_position",
            description="Arm serial command frame: legacy_position or position_torque.",
        ),
        DeclareLaunchArgument(
            "enable_dynamics_feedforward",
            default_value="false",
            description="Compute and send Pinocchio torque feedforward when using position_torque frames.",
        ),
        DeclareLaunchArgument(
            "dynamics_mode",
            default_value="gravity",
            description="Dynamics feedforward mode: gravity, coriolis, or full.",
        ),
        DeclareLaunchArgument(
            "dynamics_urdf_path",
            default_value="",
            description="Optional expanded URDF path for Pinocchio. Empty uses the robot_description XML.",
        ),
        DeclareLaunchArgument(
            "dynamics_feedforward_source",
            default_value="topic",
            description="Torque source for hardware feedforward: topic or internal_pinocchio.",
        ),
        DeclareLaunchArgument(
            "dynamics_feedforward_topic",
            default_value="/arm_dynamics/torques_nm",
            description="Float64MultiArray torque topic consumed by hardware when source is topic.",
        ),
        DeclareLaunchArgument(
            "dynamics_topic_timeout_sec",
            default_value="0.25",
            description="Max age for topic-sourced torque feedforward before zero torque is sent.",
        ),
        DeclareLaunchArgument(
            "dynamics_use_commanded_position",
            default_value="false",
            description="Use commanded joint positions instead of measured feedback for dynamics q.",
        ),
        DeclareLaunchArgument(
            "dynamics_torque_scale",
            default_value="1.0",
            description="Global multiplier applied before torque clamping.",
        ),
        DeclareLaunchArgument(
            "dynamics_torque_limits_nm",
            default_value="10.0,10.0,10.0,10.0,10.0,10.0",
            description="Comma-separated per-joint absolute torque feedforward limits in Nm.",
        ),
        DeclareLaunchArgument(
            "dynamics_torque_joint_signs",
            default_value="",
            description="Optional comma-separated torque signs. Empty defaults to arm_joint_signs.",
        ),
        DeclareLaunchArgument(
            "dynamics_torque_low_pass_alpha",
            default_value="0.4",
            description="Low-pass alpha for computed torque feedforward, 0..1.",
        ),
        DeclareLaunchArgument(
            "start_dynamics_monitor",
            default_value="true",
            description="Launch a Pinocchio torque preview node for fake or real hardware.",
        ),
        DeclareLaunchArgument(
            "dynamics_monitor_rate_hz",
            default_value="50.0",
            description="Publish rate for the Pinocchio torque preview node.",
        ),
        DeclareLaunchArgument(
            "dynamics_monitor_mode",
            default_value="gravity",
            description="Dynamics preview mode: gravity, coriolis, or full.",
        ),
        DeclareLaunchArgument(
            "dynamics_monitor_plot",
            default_value="true",
            description="Show a live Matplotlib torque plot for the dynamics preview node.",
        ),
        DeclareLaunchArgument(
            "dynamics_monitor_plot_history_sec",
            default_value="15.0",
            description="Seconds of torque history shown in the live dynamics plot.",
        ),
    ]

    joint_feedback_monitor_args = [
        DeclareLaunchArgument(
            "start_joint_feedback_monitor",
            default_value="true",
            description="Launch a Matplotlib joint feedback/reference monitor for fake or real hardware.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_joints",
            default_value="joint_1,joint_2,joint_3,joint_4,joint_5,joint_6",
            description="Comma-separated list of joints monitored by the joint feedback monitor.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_rate_hz",
            default_value="100",
            description="Sampling and refresh rate for the joint feedback monitor.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_plot",
            default_value="true",
            description="Show a live Matplotlib plot from the joint feedback monitor.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_history_sec",
            default_value="15.0",
            description="Seconds of joint feedback history shown in the live plot.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_min_rad",
            default_value="-1.0",
            description="Minimum joint feedback plot range in radians.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_max_rad",
            default_value="1.0",
            description="Maximum joint feedback plot range in radians.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_csv_enabled",
            default_value="false",
            description="Write joint feedback monitor samples to CSV.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_csv_file",
            default_value="/tmp/joint_monitor.csv",
            description="CSV file used when joint_feedback_monitor_csv_enabled is true.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_csv_append",
            default_value="false",
            description="Append feedback samples instead of overwriting the CSV file.",
        ),
    ]

    ui_args = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=_share_path("config", "moveit.rviz"),
            description="RViz config file used when start_rviz is true.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value=default_start_rviz,
            description="Launch RViz together with the rest of the stack.",
        ),
        DeclareLaunchArgument(
            "start_move_group",
            default_value=default_start_move_group,
            description="Launch move_group.",
        ),
        DeclareLaunchArgument(
            "start_controllers",
            default_value=default_start_controllers,
            description="Start controller_manager and spawn controllers.",
        ),
        DeclareLaunchArgument(
            "start_homing_button",
            default_value=default_start_homing_button,
            description="Launch the manual homing confirmation button window.",
        ),
    ]

    handeye_args = [
        DeclareLaunchArgument(
            "start_handeye",
            default_value=default_start_handeye,
            description="Launch easy_handeye2's handeye publisher.",
        ),
        DeclareLaunchArgument(
            "handeye_name",
            default_value="handeye_calib",
            description="Name passed to easy_handeye2 handeye_publisher.",
        ),
        DeclareLaunchArgument(
            "handeye_calibration_file",
            default_value=PathJoinSubstitution(
                [
                    EnvironmentVariable("HOME"),
                    ".ros2",
                    "easy_handeye2",
                    "calibrations",
                    "handeye_calib.calib",
                ]
            ),
            description="Calibration file used by easy_handeye2.",
        ),
    ]

    return (
        hardware_args
        + gripper_args
        + dynamics_args
        + joint_feedback_monitor_args
        + ui_args
        + handeye_args
    )


def robot_description_mappings():
    return {
        "initial_positions_file": LaunchConfiguration("initial_positions_file"),
        "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        "serial_port": LaunchConfiguration("serial_port"),
        "reader_port": LaunchConfiguration("reader_port"),
        "gripper_port": LaunchConfiguration("gripper_port"),
        "baudrate": LaunchConfiguration("baudrate"),
        "gripper_baudrate": LaunchConfiguration("gripper_baudrate"),
        "position_scale": LaunchConfiguration("position_scale"),
        "arm_joint_signs": LaunchConfiguration("arm_joint_signs"),
        "arm_joint_offsets": LaunchConfiguration("arm_joint_offsets"),
        "hw_slowdown": LaunchConfiguration("hw_slowdown"),
        "initial_read_timeout_sec": LaunchConfiguration("initial_read_timeout_sec"),
        "first_power_on": LaunchConfiguration("first_power_on"),
        "aux_joint_min": LaunchConfiguration("aux_joint_min"),
        "aux_joint_max": LaunchConfiguration("aux_joint_max"),
        "arm_command_frame_format": LaunchConfiguration("arm_command_frame_format"),
        "enable_dynamics_feedforward": LaunchConfiguration("enable_dynamics_feedforward"),
        "dynamics_mode": LaunchConfiguration("dynamics_mode"),
        "dynamics_urdf_path": LaunchConfiguration("dynamics_urdf_path"),
        "dynamics_feedforward_source": LaunchConfiguration("dynamics_feedforward_source"),
        "dynamics_feedforward_topic": LaunchConfiguration("dynamics_feedforward_topic"),
        "dynamics_topic_timeout_sec": LaunchConfiguration("dynamics_topic_timeout_sec"),
        "dynamics_use_commanded_position": LaunchConfiguration("dynamics_use_commanded_position"),
        "dynamics_torque_scale": LaunchConfiguration("dynamics_torque_scale"),
        "dynamics_torque_limits_nm": LaunchConfiguration("dynamics_torque_limits_nm"),
        "dynamics_torque_joint_signs": LaunchConfiguration("dynamics_torque_joint_signs"),
        "dynamics_torque_low_pass_alpha": LaunchConfiguration("dynamics_torque_low_pass_alpha"),
    }


def build_moveit_config(robot_description_mappings_override=None, pipelines=None):
    mappings = robot_description_mappings_override or {}
    return (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm.urdf.xacro", mappings=mappings)
        .robot_description_semantic(file_path="config/arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=list(pipelines or DEFAULT_PLANNING_PIPELINES))
        .to_moveit_configs()
    )


def forward_common_bringup_arguments():
    return {
        name: LaunchConfiguration(name) for name in COMMON_BRINGUP_ARGUMENT_NAMES
    }


def load_servo_parameters():
    servo_yaml = (
        Path(get_package_share_directory("arm_moveit_config")) / "config" / "servo.yaml"
    )
    with servo_yaml.open("r", encoding="utf-8") as file_handle:
        return {"moveit_servo": yaml.safe_load(file_handle) or {}}

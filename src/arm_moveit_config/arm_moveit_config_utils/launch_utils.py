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

COMMON_BRINGUP_ARGUMENT_NAMES = [
    "use_fake_hardware",
    "initial_positions_file",
    "serial_port",
    "reader_port",
    "gripper_port",
    "baudrate",
    "gripper_baudrate",
    "position_scale",
    "arm_joint_signs",
    "hw_slowdown",
    "initial_read_timeout_sec",
    "first_power_on",
    "aux_joint_min",
    "aux_joint_max",
    "enable_feedback_plot",
    "feedback_plot_rate_hz",
    "feedback_plot_min_rad",
    "feedback_plot_max_rad",
    "feedback_plot_joint_filter",
    "feedback_plot_csv_enabled",
    "feedback_plot_csv_file",
    "feedback_plot_csv_append",
    "ros2_controllers_file",
    "rviz_config",
    "start_rviz",
    "start_move_group",
    "start_controllers",
    "start_gripper_controller",
    "start_handeye",
    "start_homing_button",
    "handeye_name",
    "handeye_calibration_file",
]


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
    return [
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
            default_value="/dev/ttyUSB0",
            description="Serial writer port for the real arm interface.",
        ),
        DeclareLaunchArgument(
            "reader_port",
            default_value="/dev/ttyUSB1",
            description="Serial reader port for the real arm interface.",
        ),
        DeclareLaunchArgument(
            "gripper_port",
            default_value="/dev/ttyUSB6",
            description="Serial port for the gripper or auxiliary joint controller.",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="230400",
            description="Baudrate for the arm serial ports.",
        ),
        DeclareLaunchArgument(
            "gripper_baudrate",
            default_value="115200",
            description="Baudrate for the gripper serial port.",
        ),
        DeclareLaunchArgument(
            "position_scale",
            default_value="1000",
            description="Hardware position scaling factor.",
        ),
        DeclareLaunchArgument(
            "arm_joint_signs",
            default_value="1,1,-1,1,1,1",
            description="Comma-separated signs mapping ROS arm joint positions to hardware positions.",
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
            "aux_joint_min",
            default_value="-3.14",
            description="Minimum position for the auxiliary gripper/tool joint.",
        ),
        DeclareLaunchArgument(
            "aux_joint_max",
            default_value="3.14",
            description="Maximum position for the auxiliary gripper/tool joint.",
        ),
        DeclareLaunchArgument(
            "enable_feedback_plot",
            default_value="false",
            description="Enable the realtime feedback plot window in the custom hardware plugin.",
        ),
        DeclareLaunchArgument(
            "feedback_plot_rate_hz",
            default_value="100",
            description="Refresh rate for the feedback plot window.",
        ),
        DeclareLaunchArgument(
            "feedback_plot_min_rad",
            default_value="-1.0",
            description="Minimum plot range in radians.",
        ),
        DeclareLaunchArgument(
            "feedback_plot_max_rad",
            default_value="1.0",
            description="Maximum plot range in radians.",
        ),
        DeclareLaunchArgument(
            "feedback_plot_joint_filter",
            default_value="joint_1,joint_2,joint_3",
            description="Comma-separated list of joints shown in the realtime plot.",
        ),
        DeclareLaunchArgument(
            "feedback_plot_csv_enabled",
            default_value="false",
            description="Write feedback plot samples to CSV.",
        ),
        DeclareLaunchArgument(
            "feedback_plot_csv_file",
            default_value="/tmp/joint_monitor.csv",
            description="CSV file used when feedback_plot_csv_enabled is true.",
        ),
        DeclareLaunchArgument(
            "feedback_plot_csv_append",
            default_value="false",
            description="Append feedback samples instead of overwriting the CSV file.",
        ),
        DeclareLaunchArgument(
            "ros2_controllers_file",
            default_value=_share_path("config", "ros2_controllers.yaml"),
            description="Controller manager YAML file.",
        ),
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
            "start_gripper_controller",
            default_value=default_start_gripper_controller,
            description="Spawn the gripper action controller.",
        ),
        DeclareLaunchArgument(
            "start_handeye",
            default_value=default_start_handeye,
            description="Launch easy_handeye2's handeye publisher.",
        ),
        DeclareLaunchArgument(
            "start_homing_button",
            default_value=default_start_homing_button,
            description="Launch the manual homing confirmation button window.",
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
        "hw_slowdown": LaunchConfiguration("hw_slowdown"),
        "initial_read_timeout_sec": LaunchConfiguration("initial_read_timeout_sec"),
        "first_power_on": LaunchConfiguration("first_power_on"),
        "aux_joint_min": LaunchConfiguration("aux_joint_min"),
        "aux_joint_max": LaunchConfiguration("aux_joint_max"),
        "enable_feedback_plot": LaunchConfiguration("enable_feedback_plot"),
        "feedback_plot_rate_hz": LaunchConfiguration("feedback_plot_rate_hz"),
        "feedback_plot_min_rad": LaunchConfiguration("feedback_plot_min_rad"),
        "feedback_plot_max_rad": LaunchConfiguration("feedback_plot_max_rad"),
        "feedback_plot_joint_filter": LaunchConfiguration("feedback_plot_joint_filter"),
        "feedback_plot_csv_enabled": LaunchConfiguration("feedback_plot_csv_enabled"),
        "feedback_plot_csv_file": LaunchConfiguration("feedback_plot_csv_file"),
        "feedback_plot_csv_append": LaunchConfiguration("feedback_plot_csv_append"),
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

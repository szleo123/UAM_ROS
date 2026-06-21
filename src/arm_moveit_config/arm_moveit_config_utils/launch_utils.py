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
    "feedback_velocity_low_pass_alpha",
    "enable_arm_command_limiter",
    "arm_command_velocity_limits_rad_s",
    "arm_command_acceleration_limits_rad_s2",
    "initial_read_timeout_sec",
    "first_power_on",
    "ros2_controllers_file",
]

STM32_ARGUMENT_NAMES = [
    "stm32_control_mode",
    "stm32_kp",
    "stm32_kd",
    "stm32_v_des_limits_rad_s",
    "stm32_heartbeat_duration_sec",
    "stm32_trigger_duration_sec",
    "stm32_feedback_wait_timeout_sec",
    "enable_stm32_zero_trigger",
    "stm32_recover_safe_drop_on_start",
    "stm32_recover_duration_sec",
    "stm32_trace_enabled",
    "stm32_trace_directory",
    "stm32_trace_run_label",
    "stm32_trace_file",
    "stm32_trace_append",
    "stm32_trace_decimation",
]

GRIPPER_ARGUMENT_NAMES = [
    "gripper_backend",
    "aux_joint_min",
    "aux_joint_max",
    "gripper_port",
    "gripper_baudrate",
    "dynamixel_id",
    "dynamixel_protocol_version",
    "dynamixel_open_position_ticks",
    "dynamixel_close_position_ticks",
    "dynamixel_goal_current_ma",
    "dynamixel_open_current_ma",
    "dynamixel_current_limit_ma",
    "dynamixel_temperature_limit_c",
    "dynamixel_bus_watchdog_ms",
    "dynamixel_configure_on_start",
    "dynamixel_apply_limits_on_start",
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
    "start_manual_torque_tuner",
    "manual_torque_initial_nm",
    "manual_torque_abs_max_nm",
    "manual_torque_publish_rate_hz",
    "manual_torque_live_update",
]

JOINT_FEEDBACK_MONITOR_ARGUMENT_NAMES = [
    "start_joint_feedback_monitor",
    "joint_feedback_monitor_joints",
    "joint_feedback_monitor_rate_hz",
    "joint_feedback_monitor_plot",
    "joint_feedback_monitor_history_sec",
    "joint_feedback_monitor_min_rad",
    "joint_feedback_monitor_max_rad",
    "joint_feedback_monitor_velocity_abs",
    "joint_feedback_monitor_show_raw_velocity",
    "joint_feedback_monitor_raw_velocity_topic",
    "joint_feedback_monitor_torque_abs",
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
    "start_arm_emergency_stop",
    "arm_emergency_stop_service",
    "arm_emergency_stop_hold_duration_s",
    "arm_emergency_stop_joint_state_timeout_s",
    "arm_emergency_stop_publish_hold",
    "arm_emergency_stop_cancel_before_hold",
    "enable_emergency_gripper_open",
    "emergency_gripper_action_name",
    "emergency_gripper_open_position",
    "emergency_gripper_max_effort",
    "emergency_gripper_cancel_before_open",
    "enable_gripper_maintenance_ui",
    "enable_gripper_parameter_writes",
    "enable_gripper_flash_save",
    "gripper_status_poll_s",
    "start_mit_gain_tuner",
    "mit_gain_tuner_kp_max",
    "mit_gain_tuner_kd_max",
    "mit_gain_tuner_live_update",
]

HANDEYE_ARGUMENT_NAMES = [
    "start_handeye",
    "handeye_name",
    "handeye_calibration_file",
]

COMMON_BRINGUP_ARGUMENT_NAMES = (
    HARDWARE_ARGUMENT_NAMES
    + STM32_ARGUMENT_NAMES
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
    default_start_mit_gain_tuner="true",
    default_first_power_on="true",
    default_start_dynamics_monitor="true",
    default_dynamics_monitor_plot="true",
    default_start_joint_feedback_monitor="true",
    default_joint_feedback_monitor_plot="true",
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
            default_value="/dev/ttyACM0",
            description="Full-duplex STM32 USB CDC port for the real arm interface.",
        ),
        DeclareLaunchArgument(
            "reader_port",
            default_value="/dev/ttyUSB0",
            description="Deprecated split-reader port kept for legacy protocol compatibility.",
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
            default_value="1,1,1,1,1,1",
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
            "feedback_velocity_low_pass_alpha",
            default_value="0.25",
            description=(
                "Low-pass alpha for STM32 velocity feedback state, one value or six "
                "comma-separated values. 1.0 is raw velocity, lower is smoother."
            ),
        ),
        DeclareLaunchArgument(
            "enable_arm_command_limiter",
            default_value="false",
            description=(
                "Apply final per-joint position-command velocity/acceleration limits "
                "inside the real hardware interface before sending STM32 frames."
            ),
        ),
        DeclareLaunchArgument(
            "arm_command_velocity_limits_rad_s",
            default_value="2.0,2.0,0.5,2.0,1.0,2.0",
            description=(
                "Comma-separated final arm command velocity limits in rad/s. "
                "Applies equally to MoveIt, rqt, and teleop commands."
            ),
        ),
        DeclareLaunchArgument(
            "arm_command_acceleration_limits_rad_s2",
            default_value="1.0,1.0,0.2,1.0,0.5,1.0",
            description=(
                "Comma-separated final arm command acceleration limits in rad/s^2. "
                "Applies equally to MoveIt, rqt, and teleop commands."
            ),
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

    stm32_args = [
        DeclareLaunchArgument(
            "stm32_control_mode",
            default_value="2",
            description=(
                "STM32 motor command mode: 0/position_only, 1/position_torque, "
                "or 2/full_mit."
            ),
        ),
        DeclareLaunchArgument(
            "stm32_kp",
            default_value="460,480,320,100,60,10",
            description=(
                "Comma-separated per-joint Kp sent to STM32 in full MIT mode. "
                "Ignored by STM32 position-only mode."
            ),
        ),
        DeclareLaunchArgument(
            "stm32_kd",
            default_value="2.0,2.0,1.0,0.7,0.3,1.0",
            description=(
                "Comma-separated per-joint Kd sent to STM32 in full MIT mode. "
                "Ignored by STM32 position-only mode."
            ),
        ),
        DeclareLaunchArgument(
            "stm32_v_des_limits_rad_s",
            default_value="2.0,2.0,0.5,2.0,1.0,2.0",
            description=(
                "Comma-separated per-joint absolute clamps for velocity commands sent "
                "to STM32 v_des in full MIT mode. Defaults match joint_limits.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "stm32_heartbeat_duration_sec",
            default_value="1.0",
            description="Duration of 100 Hz safe-lock heartbeat frames during STM32 boot.",
        ),
        DeclareLaunchArgument(
            "stm32_trigger_duration_sec",
            default_value="3.0",
            description=(
                "Duration of 100 Hz STM32 homing trigger frames. Only used when "
                "enable_stm32_zero_trigger is true."
            ),
        ),
        DeclareLaunchArgument(
            "stm32_feedback_wait_timeout_sec",
            default_value="2.0",
            description="How long activation waits for valid STM32 feedback after boot handshake.",
        ),
        DeclareLaunchArgument(
            "enable_stm32_zero_trigger",
            default_value="true",
            description=(
                "Enable the operator Confirm Drop Pose packet that triggers STM32 joint-3 zeroing. "
                "Keep false on hardware until the MCU zeroing sequence is bench-validated."
            ),
        ),
        DeclareLaunchArgument(
            "stm32_recover_safe_drop_on_start",
            default_value="true",
            description=(
                "Send a deliberate recovery frame at activation to clear an STM32 safe-drop latch "
                "from a previous ROS session. USB replug alone still cannot recover the latch."
            ),
        ),
        DeclareLaunchArgument(
            "stm32_recover_duration_sec",
            default_value="2.0",
            description="Duration of 100 Hz STM32 safe-drop recovery frames at activation.",
        ),
        DeclareLaunchArgument(
            "stm32_trace_enabled",
            default_value="true",
            description="Write a per-run STM32 command/feedback/torque CSV trace from the hardware interface.",
        ),
        DeclareLaunchArgument(
            "stm32_trace_file",
            default_value="",
            description=(
                "Optional exact STM32 trace CSV path. Empty creates a timestamped file "
                "inside stm32_trace_directory."
            ),
        ),
        DeclareLaunchArgument(
            "stm32_trace_directory",
            default_value="/home/li/UAM_ROS/run_logs",
            description="Folder for timestamped STM32 trace CSV files.",
        ),
        DeclareLaunchArgument(
            "stm32_trace_run_label",
            default_value="",
            description="Optional label appended to timestamped STM32 trace filenames.",
        ),
        DeclareLaunchArgument(
            "stm32_trace_append",
            default_value="false",
            description="Append to the STM32 trace CSV instead of overwriting it.",
        ),
        DeclareLaunchArgument(
            "stm32_trace_decimation",
            default_value="1",
            description="Write one STM32 trace row every N trace samples.",
        ),
    ]

    gripper_args = [
        DeclareLaunchArgument(
            "gripper_backend",
            default_value="linear_actuator",
            description="Gripper backend: linear_actuator or dynamixel_xw430.",
        ),
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
            default_value="/dev/ttyUSB0",
            description="Serial port for the gripper or auxiliary joint controller.",
        ),
        DeclareLaunchArgument(
            "gripper_baudrate",
            default_value="115200",
            description="Baudrate for the gripper serial port.",
        ),
        DeclareLaunchArgument(
            "dynamixel_id",
            default_value="1",
            description="DYNAMIXEL ID used when gripper_backend:=dynamixel_xw430.",
        ),
        DeclareLaunchArgument(
            "dynamixel_protocol_version",
            default_value="2.0",
            description="DYNAMIXEL protocol version.",
        ),
        DeclareLaunchArgument(
            "dynamixel_open_position_ticks",
            default_value="2048",
            description="Conservative open position tick value for DYNAMIXEL bench bringup.",
        ),
        DeclareLaunchArgument(
            "dynamixel_close_position_ticks",
            default_value="2600",
            description="Conservative close/sampling position tick value for DYNAMIXEL bench bringup.",
        ),
        DeclareLaunchArgument(
            "dynamixel_goal_current_ma",
            default_value="150.0",
            description="Default DYNAMIXEL goal current used for normal gripper moves.",
        ),
        DeclareLaunchArgument(
            "dynamixel_open_current_ma",
            default_value="150.0",
            description="DYNAMIXEL goal current used for emergency/open recovery moves.",
        ),
        DeclareLaunchArgument(
            "dynamixel_current_limit_ma",
            default_value="500.0",
            description="Bench-safe DYNAMIXEL EEPROM current limit. Applied only when dynamixel_apply_limits_on_start is true or via UI.",
        ),
        DeclareLaunchArgument(
            "dynamixel_temperature_limit_c",
            default_value="75.0",
            description="Bench-safe DYNAMIXEL EEPROM temperature limit. Applied only when dynamixel_apply_limits_on_start is true or via UI.",
        ),
        DeclareLaunchArgument(
            "dynamixel_bus_watchdog_ms",
            default_value="200",
            description="DYNAMIXEL bus watchdog in milliseconds; 0 disables.",
        ),
        DeclareLaunchArgument(
            "dynamixel_configure_on_start",
            default_value="true",
            description="Torque off, set current-based position mode, then torque on during DYNAMIXEL startup.",
        ),
        DeclareLaunchArgument(
            "dynamixel_apply_limits_on_start",
            default_value="false",
            description="Apply DYNAMIXEL EEPROM current/temperature limits during startup. Disabled by default for cautious bench bringup.",
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
            default_value="true",
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
            default_value="1.0,1.0,1.0,1.0,1.0,1.0",
            description=(
                "Torque multiplier applied before clamping. Use one value for all joints "
                "or six comma-separated values for joint_1..joint_6."
            ),
        ),
        DeclareLaunchArgument(
            "dynamics_torque_limits_nm",
            default_value="10.0,20.0,10.0,10.0,10.0,10.0",
            description="Comma-separated per-joint absolute torque feedforward limits in Nm.",
        ),
        DeclareLaunchArgument(
            "dynamics_torque_joint_signs",
            default_value="",
            description="Optional comma-separated torque signs. Empty defaults to arm_joint_signs.",
        ),
        DeclareLaunchArgument(
            "dynamics_torque_low_pass_alpha",
            default_value="0.8",
            description="Low-pass alpha for computed torque feedforward, 0..1.",
        ),
        DeclareLaunchArgument(
            "start_dynamics_monitor",
            default_value=default_start_dynamics_monitor,
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
            default_value=default_dynamics_monitor_plot,
            description="Show a live Matplotlib torque plot for the dynamics preview node.",
        ),
        DeclareLaunchArgument(
            "dynamics_monitor_plot_history_sec",
            default_value="15.0",
            description="Seconds of torque history shown in the live dynamics plot.",
        ),
        DeclareLaunchArgument(
            "start_manual_torque_tuner",
            default_value="false",
            description=(
                "Launch a manual torque feedforward tuner that publishes to "
                "dynamics_feedforward_topic. Use with start_dynamics_monitor=false."
            ),
        ),
        DeclareLaunchArgument(
            "manual_torque_initial_nm",
            default_value="0.0,0.0,0.0,0.0,0.0,0.0",
            description="Initial manual torque feedforward values in Nm for joint_1..joint_6.",
        ),
        DeclareLaunchArgument(
            "manual_torque_abs_max_nm",
            default_value="10.0,10.0,10.0,10.0,10.0,10.0",
            description="Per-joint absolute slider limits for manual torque feedforward in Nm.",
        ),
        DeclareLaunchArgument(
            "manual_torque_publish_rate_hz",
            default_value="20.0",
            description=(
                "Continuous publish rate for manual torque feedforward. Keep this faster "
                "than dynamics_topic_timeout_sec."
            ),
        ),
        DeclareLaunchArgument(
            "manual_torque_live_update",
            default_value="false",
            description="Publish manual torque changes while sliders move.",
        ),
    ]

    joint_feedback_monitor_args = [
        DeclareLaunchArgument(
            "start_joint_feedback_monitor",
            default_value=default_start_joint_feedback_monitor,
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
            default_value=default_joint_feedback_monitor_plot,
            description="Show a live Matplotlib plot from the joint feedback monitor.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_history_sec",
            default_value="15.0",
            description="Seconds of joint feedback history shown in the live plot.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_min_rad",
            default_value="-3.14",
            description="Minimum joint feedback plot range in radians.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_max_rad",
            default_value="3.14",
            description="Maximum joint feedback plot range in radians.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_velocity_abs",
            default_value="2.0",
            description="Initial symmetric velocity plot range in rad/s.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_show_raw_velocity",
            default_value="false",
            description="Overlay unfiltered STM32 velocity feedback on the live velocity plot.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_raw_velocity_topic",
            default_value="/my_arm_system/raw_feedback_velocities_rad_s",
            description="Topic used by the live monitor for unfiltered STM32 velocity feedback.",
        ),
        DeclareLaunchArgument(
            "joint_feedback_monitor_torque_abs",
            default_value="10.0",
            description="Initial symmetric torque plot range in Nm.",
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
        DeclareLaunchArgument(
            "start_arm_emergency_stop",
            default_value="true",
            description="Start the shared arm emergency stop service.",
        ),
        DeclareLaunchArgument(
            "arm_emergency_stop_service",
            default_value="/arm_emergency_stop/trigger",
            description="Trigger service for canceling active arm motion and publishing hold.",
        ),
        DeclareLaunchArgument(
            "arm_emergency_stop_hold_duration_s",
            default_value="0.2",
            description="Duration of the hold-current-position trajectory sent by emergency stop.",
        ),
        DeclareLaunchArgument(
            "arm_emergency_stop_joint_state_timeout_s",
            default_value="0.5",
            description="Maximum joint-state age accepted for emergency hold publication.",
        ),
        DeclareLaunchArgument(
            "arm_emergency_stop_publish_hold",
            default_value="true",
            description="Publish a hold-current-position trajectory during arm emergency stop.",
        ),
        DeclareLaunchArgument(
            "arm_emergency_stop_cancel_before_hold",
            default_value="true",
            description="Request cancellation of active arm trajectory goals during emergency stop.",
        ),
        DeclareLaunchArgument(
            "enable_emergency_gripper_open",
            default_value="true",
            description="Add the emergency cancel-and-open gripper button to the homing/operator UI.",
        ),
        DeclareLaunchArgument(
            "emergency_gripper_action_name",
            default_value="/gripper_controller/gripper_cmd",
            description="Gripper action used by the emergency open button.",
        ),
        DeclareLaunchArgument(
            "emergency_gripper_open_position",
            default_value="0.0",
            description="Open position sent by the emergency gripper button.",
        ),
        DeclareLaunchArgument(
            "emergency_gripper_max_effort",
            default_value="0.0",
            description="Max effort sent by the emergency gripper button.",
        ),
        DeclareLaunchArgument(
            "emergency_gripper_cancel_before_open",
            default_value="true",
            description="Cancel current gripper action goals before sending emergency open.",
        ),
        DeclareLaunchArgument(
            "enable_gripper_maintenance_ui",
            default_value="true",
            description="Add gripper status, recovery, and protection controls to the homing/operator UI.",
        ),
        DeclareLaunchArgument(
            "enable_gripper_parameter_writes",
            default_value="false",
            description="Allow the homing/operator UI to write gripper protection parameters to RAM.",
        ),
        DeclareLaunchArgument(
            "enable_gripper_flash_save",
            default_value="false",
            description="Allow the homing/operator UI to save current gripper parameters to actuator Flash.",
        ),
        DeclareLaunchArgument(
            "gripper_status_poll_s",
            default_value="1.0",
            description="Polling period for gripper status in the homing/operator UI.",
        ),
        DeclareLaunchArgument(
            "start_mit_gain_tuner",
            default_value=default_start_mit_gain_tuner,
            description="Launch the STM32 FULL MIT Kp/Kd tuning window.",
        ),
        DeclareLaunchArgument(
            "mit_gain_tuner_kp_max",
            default_value="1000.0",
            description="Maximum Kp slider value in the FULL MIT gain tuner.",
        ),
        DeclareLaunchArgument(
            "mit_gain_tuner_kd_max",
            default_value="100.0",
            description="Maximum Kd slider value in the FULL MIT gain tuner.",
        ),
        DeclareLaunchArgument(
            "mit_gain_tuner_live_update",
            default_value="false",
            description="Publish Kp/Kd changes while sliders move in the FULL MIT gain tuner.",
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
        + stm32_args
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
        "gripper_backend": LaunchConfiguration("gripper_backend"),
        "gripper_port": LaunchConfiguration("gripper_port"),
        "baudrate": LaunchConfiguration("baudrate"),
        "gripper_baudrate": LaunchConfiguration("gripper_baudrate"),
        "dynamixel_id": LaunchConfiguration("dynamixel_id"),
        "dynamixel_protocol_version": LaunchConfiguration("dynamixel_protocol_version"),
        "dynamixel_open_position_ticks": LaunchConfiguration("dynamixel_open_position_ticks"),
        "dynamixel_close_position_ticks": LaunchConfiguration("dynamixel_close_position_ticks"),
        "dynamixel_goal_current_ma": LaunchConfiguration("dynamixel_goal_current_ma"),
        "dynamixel_open_current_ma": LaunchConfiguration("dynamixel_open_current_ma"),
        "dynamixel_current_limit_ma": LaunchConfiguration("dynamixel_current_limit_ma"),
        "dynamixel_temperature_limit_c": LaunchConfiguration("dynamixel_temperature_limit_c"),
        "dynamixel_bus_watchdog_ms": LaunchConfiguration("dynamixel_bus_watchdog_ms"),
        "dynamixel_configure_on_start": LaunchConfiguration("dynamixel_configure_on_start"),
        "dynamixel_apply_limits_on_start": LaunchConfiguration("dynamixel_apply_limits_on_start"),
        "position_scale": LaunchConfiguration("position_scale"),
        "arm_joint_signs": LaunchConfiguration("arm_joint_signs"),
        "arm_joint_offsets": LaunchConfiguration("arm_joint_offsets"),
        "hw_slowdown": LaunchConfiguration("hw_slowdown"),
        "feedback_velocity_low_pass_alpha": LaunchConfiguration("feedback_velocity_low_pass_alpha"),
        "enable_arm_command_limiter": LaunchConfiguration("enable_arm_command_limiter"),
        "arm_command_velocity_limits_rad_s": LaunchConfiguration("arm_command_velocity_limits_rad_s"),
        "arm_command_acceleration_limits_rad_s2": LaunchConfiguration("arm_command_acceleration_limits_rad_s2"),
        "initial_read_timeout_sec": LaunchConfiguration("initial_read_timeout_sec"),
        "first_power_on": LaunchConfiguration("first_power_on"),
        "aux_joint_min": LaunchConfiguration("aux_joint_min"),
        "aux_joint_max": LaunchConfiguration("aux_joint_max"),
        "arm_command_frame_format": LaunchConfiguration("arm_command_frame_format"),
        "stm32_control_mode": LaunchConfiguration("stm32_control_mode"),
        "stm32_kp": LaunchConfiguration("stm32_kp"),
        "stm32_kd": LaunchConfiguration("stm32_kd"),
        "stm32_v_des_limits_rad_s": LaunchConfiguration("stm32_v_des_limits_rad_s"),
        "stm32_heartbeat_duration_sec": LaunchConfiguration("stm32_heartbeat_duration_sec"),
        "stm32_trigger_duration_sec": LaunchConfiguration("stm32_trigger_duration_sec"),
        "stm32_feedback_wait_timeout_sec": LaunchConfiguration("stm32_feedback_wait_timeout_sec"),
        "enable_stm32_zero_trigger": LaunchConfiguration("enable_stm32_zero_trigger"),
        "stm32_recover_safe_drop_on_start": LaunchConfiguration("stm32_recover_safe_drop_on_start"),
        "stm32_recover_duration_sec": LaunchConfiguration("stm32_recover_duration_sec"),
        "stm32_trace_enabled": LaunchConfiguration("stm32_trace_enabled"),
        "stm32_trace_directory": LaunchConfiguration("stm32_trace_directory"),
        "stm32_trace_run_label": LaunchConfiguration("stm32_trace_run_label"),
        "stm32_trace_file": LaunchConfiguration("stm32_trace_file"),
        "stm32_trace_append": LaunchConfiguration("stm32_trace_append"),
        "stm32_trace_decimation": LaunchConfiguration("stm32_trace_decimation"),
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

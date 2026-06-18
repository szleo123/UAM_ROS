from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from arm_moveit_config_utils.launch_utils import build_moveit_config


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )


def _rviz_node(context, *args, **kwargs):
    if not _as_bool(context, "start_rviz"):
        return []

    moveit_config = build_moveit_config(pipelines=["ompl"])
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="operator_rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )
    ]


def _dynamics_preview_node(context, *args, **kwargs):
    if not _as_bool(context, "start_dynamics_preview"):
        return []

    moveit_config = build_moveit_config()
    return [
        Node(
            package="my_arm_hardware",
            executable="dynamics_monitor.py",
            name="operator_dynamics_preview",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {
                    "joint_names": LaunchConfiguration("joint_names"),
                    "output_topic": LaunchConfiguration("dynamics_preview_output_topic"),
                    "per_joint_topic_prefix": LaunchConfiguration(
                        "dynamics_preview_per_joint_topic_prefix"
                    ),
                    "rate_hz": LaunchConfiguration("dynamics_preview_rate_hz"),
                    "mode": LaunchConfiguration("dynamics_preview_mode"),
                    "torque_scale": LaunchConfiguration("dynamics_preview_torque_scale"),
                    "torque_limits_nm": LaunchConfiguration("dynamics_preview_torque_limits_nm"),
                    "low_pass_alpha": LaunchConfiguration("dynamics_preview_low_pass_alpha"),
                    "enable_plot": "true",
                    "plot_history_sec": LaunchConfiguration("plot_history_sec"),
                },
            ],
        )
    ]


def generate_launch_description():
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "moveit.rviz"]
    )

    homing_button = Node(
        package="my_arm_hardware",
        executable="homing_button.py",
        name="operator_arm_homing_button",
        output="screen",
        parameters=[
            {
                "initial_pose_trajectory_topic": LaunchConfiguration(
                    "homing_initial_pose_trajectory_topic"
                ),
                "initial_pose_joints": LaunchConfiguration("joint_names"),
                "initial_pose_positions": LaunchConfiguration("homing_initial_pose_positions"),
                "initial_pose_duration_s": LaunchConfiguration("homing_initial_pose_duration_s"),
                "enable_emergency_gripper_open": LaunchConfiguration(
                    "enable_emergency_gripper_open"
                ),
                "gripper_action_name": LaunchConfiguration("emergency_gripper_action_name"),
                "gripper_open_position": LaunchConfiguration("emergency_gripper_open_position"),
                "gripper_max_effort": LaunchConfiguration("emergency_gripper_max_effort"),
                "gripper_cancel_before_open": LaunchConfiguration(
                    "emergency_gripper_cancel_before_open"
                ),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_homing_button")),
    )

    joint_feedback_monitor = Node(
        package="my_arm_hardware",
        executable="joint_feedback_monitor.py",
        name="operator_joint_feedback_monitor",
        output="screen",
        parameters=[
            {
                "joint_names": LaunchConfiguration("joint_names"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "controller_state_topic": LaunchConfiguration("controller_state_topic"),
                "raw_velocity_topic": LaunchConfiguration("raw_velocity_topic"),
                "torque_reference_topic": LaunchConfiguration("torque_reference_topic"),
                "output_error_topic": LaunchConfiguration("joint_error_topic"),
                "rate_hz": LaunchConfiguration("joint_feedback_monitor_rate_hz"),
                "plot": "true",
                "plot_history_sec": LaunchConfiguration("plot_history_sec"),
                "plot_rate_hz": LaunchConfiguration("plot_rate_hz"),
                "plot_min_rad": LaunchConfiguration("joint_feedback_min_rad"),
                "plot_max_rad": LaunchConfiguration("joint_feedback_max_rad"),
                "plot_velocity_abs": LaunchConfiguration("joint_feedback_velocity_abs"),
                "plot_raw_velocity": LaunchConfiguration("show_raw_velocity"),
                "plot_torque_abs": LaunchConfiguration("joint_feedback_torque_abs"),
                "csv_enabled": LaunchConfiguration("joint_feedback_csv_enabled"),
                "csv_file": LaunchConfiguration("joint_feedback_csv_file"),
                "csv_append": LaunchConfiguration("joint_feedback_csv_append"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_joint_feedback_monitor")),
    )

    trajectory_command_monitor = Node(
        package="arm_geomagic_teleop",
        executable="trajectory_command_monitor",
        name="operator_trajectory_command_monitor",
        output="screen",
        parameters=[
            {
                "trajectory_topic": LaunchConfiguration("trajectory_topic"),
                "joint_names": LaunchConfiguration("joint_names"),
                "plot": "true",
                "plot_history_sec": LaunchConfiguration("plot_history_sec"),
                "plot_rate_hz": LaunchConfiguration("plot_rate_hz"),
                "position_abs_rad": LaunchConfiguration("trajectory_position_abs_rad"),
                "step_abs_rad": LaunchConfiguration("trajectory_step_abs_rad"),
                "velocity_abs_rad_s": LaunchConfiguration("trajectory_velocity_abs_rad_s"),
                "csv_enabled": LaunchConfiguration("trajectory_csv_enabled"),
                "csv_file": LaunchConfiguration("trajectory_csv_file"),
                "csv_append": LaunchConfiguration("trajectory_csv_append"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_trajectory_command_monitor")),
    )

    mit_gain_tuner = Node(
        package="my_arm_hardware",
        executable="mit_gain_tuner.py",
        name="operator_stm32_mit_gain_tuner",
        output="screen",
        parameters=[
            {
                "kp_max": LaunchConfiguration("mit_gain_tuner_kp_max"),
                "kd_max": LaunchConfiguration("mit_gain_tuner_kd_max"),
                "live_update": LaunchConfiguration("mit_gain_tuner_live_update"),
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("enable_write_tools"),
                    "'.lower() == 'true' and '",
                    LaunchConfiguration("start_mit_gain_tuner"),
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    manual_torque_tuner = Node(
        package="my_arm_hardware",
        executable="manual_torque_tuner.py",
        name="operator_manual_torque_tuner",
        output="screen",
        parameters=[
            {
                "output_topic": LaunchConfiguration("manual_torque_output_topic"),
                "joint_names": LaunchConfiguration("joint_names"),
                "initial_torques_nm": LaunchConfiguration("manual_torque_initial_nm"),
                "torque_abs_max_nm": LaunchConfiguration("manual_torque_abs_max_nm"),
                "publish_rate_hz": LaunchConfiguration("manual_torque_publish_rate_hz"),
                "live_update": LaunchConfiguration("manual_torque_live_update"),
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("enable_write_tools"),
                    "'.lower() == 'true' and '",
                    LaunchConfiguration("start_manual_torque_tuner"),
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz on the laptop as an operator console.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file for laptop operator view.",
            ),
            DeclareLaunchArgument(
                "start_joint_feedback_monitor",
                default_value="true",
                description="Show the read-mostly joint feedback/reference plot on the laptop.",
            ),
            DeclareLaunchArgument(
                "start_homing_button",
                default_value="true",
                description="Show the remote homing/zeroing GUI on the laptop.",
            ),
            DeclareLaunchArgument(
                "start_trajectory_command_monitor",
                default_value="true",
                description="Show final arm trajectory commands on the laptop.",
            ),
            DeclareLaunchArgument(
                "start_dynamics_preview",
                default_value="false",
                description="Show a laptop-local dynamics preview plot on a non-command topic.",
            ),
            DeclareLaunchArgument(
                "enable_write_tools",
                default_value="false",
                description="Required guard for laptop tools that publish hardware-affecting commands.",
            ),
            DeclareLaunchArgument(
                "start_mit_gain_tuner",
                default_value="false",
                description="Start remote STM32 FULL MIT Kp/Kd tuner. Requires enable_write_tools:=true.",
            ),
            DeclareLaunchArgument(
                "start_manual_torque_tuner",
                default_value="false",
                description="Start remote manual torque feedforward tuner. Requires enable_write_tools:=true.",
            ),
            DeclareLaunchArgument(
                "joint_names",
                default_value="joint_1,joint_2,joint_3,joint_4,joint_5,joint_6",
                description="Comma-separated arm joints shown in operator plots.",
            ),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            DeclareLaunchArgument(
                "controller_state_topic",
                default_value="/arm_controller/controller_state",
            ),
            DeclareLaunchArgument(
                "raw_velocity_topic",
                default_value="/my_arm_system/raw_feedback_velocities_rad_s",
            ),
            DeclareLaunchArgument(
                "torque_reference_topic",
                default_value="/my_arm_system/dynamics_tau_ff_nm",
            ),
            DeclareLaunchArgument(
                "joint_error_topic",
                default_value="/operator_tools/joint_feedback_error_rad",
            ),
            DeclareLaunchArgument(
                "trajectory_topic",
                default_value="/arm_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "homing_initial_pose_trajectory_topic",
                default_value="/arm_controller/joint_trajectory",
                description="Trajectory topic used by the homing GUI Move Initial Pose button.",
            ),
            DeclareLaunchArgument(
                "homing_initial_pose_positions",
                default_value="0.0,0.0,0.0,0.0,0.0,0.0",
                description="Comma-separated positions for the homing GUI Move Initial Pose button.",
            ),
            DeclareLaunchArgument(
                "homing_initial_pose_duration_s",
                default_value="8.0",
                description="Duration for the homing GUI Move Initial Pose trajectory.",
            ),
            DeclareLaunchArgument(
                "enable_emergency_gripper_open",
                default_value="true",
                description="Add the emergency cancel-and-open gripper button to the operator GUI.",
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
            DeclareLaunchArgument("plot_history_sec", default_value="15.0"),
            DeclareLaunchArgument("plot_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("joint_feedback_monitor_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("joint_feedback_min_rad", default_value="-3.14"),
            DeclareLaunchArgument("joint_feedback_max_rad", default_value="3.14"),
            DeclareLaunchArgument("joint_feedback_velocity_abs", default_value="2.0"),
            DeclareLaunchArgument("joint_feedback_torque_abs", default_value="10.0"),
            DeclareLaunchArgument("show_raw_velocity", default_value="false"),
            DeclareLaunchArgument("joint_feedback_csv_enabled", default_value="false"),
            DeclareLaunchArgument(
                "joint_feedback_csv_file",
                default_value="/tmp/operator_joint_feedback_monitor.csv",
            ),
            DeclareLaunchArgument("joint_feedback_csv_append", default_value="false"),
            DeclareLaunchArgument("trajectory_position_abs_rad", default_value="3.2"),
            DeclareLaunchArgument("trajectory_step_abs_rad", default_value="0.006"),
            DeclareLaunchArgument("trajectory_velocity_abs_rad_s", default_value="0.5"),
            DeclareLaunchArgument("trajectory_csv_enabled", default_value="false"),
            DeclareLaunchArgument(
                "trajectory_csv_file",
                default_value="/tmp/operator_trajectory_command_monitor.csv",
            ),
            DeclareLaunchArgument("trajectory_csv_append", default_value="false"),
            DeclareLaunchArgument(
                "dynamics_preview_output_topic",
                default_value="/operator_tools/dynamics_preview_torques_nm",
                description="Safe laptop preview topic, not the Jetson hardware feedforward topic.",
            ),
            DeclareLaunchArgument(
                "dynamics_preview_per_joint_topic_prefix",
                default_value="/operator_tools/dynamics_preview",
                description="Safe laptop preview prefix for per-joint torque diagnostics.",
            ),
            DeclareLaunchArgument("dynamics_preview_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("dynamics_preview_mode", default_value="gravity"),
            DeclareLaunchArgument(
                "dynamics_preview_torque_scale",
                default_value="1.0,1.0,1.0,1.0,1.0,1.0",
            ),
            DeclareLaunchArgument(
                "dynamics_preview_torque_limits_nm",
                default_value="10.0,20.0,10.0,10.0,10.0,10.0",
            ),
            DeclareLaunchArgument("dynamics_preview_low_pass_alpha", default_value="0.8"),
            DeclareLaunchArgument("mit_gain_tuner_kp_max", default_value="1000.0"),
            DeclareLaunchArgument("mit_gain_tuner_kd_max", default_value="100.0"),
            DeclareLaunchArgument("mit_gain_tuner_live_update", default_value="false"),
            DeclareLaunchArgument(
                "manual_torque_output_topic",
                default_value="/arm_dynamics/torques_nm",
                description="Hardware feedforward topic; only active when write tools are enabled.",
            ),
            DeclareLaunchArgument(
                "manual_torque_initial_nm",
                default_value="0.0,0.0,0.0,0.0,0.0,0.0",
            ),
            DeclareLaunchArgument(
                "manual_torque_abs_max_nm",
                default_value="10.0,10.0,10.0,10.0,10.0,10.0",
            ),
            DeclareLaunchArgument("manual_torque_publish_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("manual_torque_live_update", default_value="false"),
            OpaqueFunction(function=_rviz_node),
            homing_button,
            joint_feedback_monitor,
            trajectory_command_monitor,
            OpaqueFunction(function=_dynamics_preview_node),
            mit_gain_tuner,
            manual_torque_tuner,
        ]
    )

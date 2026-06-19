from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from arm_moveit_config_utils.launch_utils import build_moveit_config


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )


def _input_nodes(context, *args, **kwargs):
    input_device = LaunchConfiguration("input_device").perform(context).strip().lower()
    nodes = []

    if input_device not in ("geomagic", "keyboard", "none"):
        raise RuntimeError("input_device must be 'geomagic', 'keyboard', or 'none'.")

    if input_device == "geomagic":
        if _as_bool(context, "start_geomagic_driver"):
            nodes.append(
                Node(
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
                )
            )
        if _as_bool(context, "use_omni_adapter"):
            nodes.append(
                Node(
                    package="arm_geomagic_teleop",
                    executable="geomagic_omni_state_adapter",
                    name="geomagic_omni_state_adapter",
                    output="screen",
                    parameters=[LaunchConfiguration("teleop_config")],
                )
            )
        if _as_bool(context, "start_geomagic_teleop"):
            nodes.append(
                Node(
                    package="arm_geomagic_teleop",
                    executable="geomagic_cartesian_teleop",
                    name="geomagic_cartesian_teleop",
                    output="screen",
                    parameters=[
                        LaunchConfiguration("teleop_config"),
                        {
                            "output_twist_topic": LaunchConfiguration("raw_twist_topic"),
                        },
                    ],
                )
            )

    if input_device == "keyboard":
        nodes.append(
            Node(
                package="arm_geomagic_teleop",
                executable="keyboard_twist_teleop",
                name="keyboard_twist_teleop",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("teleop_config"),
                    {
                        "keyboard_backend": LaunchConfiguration("keyboard_backend"),
                        "keyboard_device_path": LaunchConfiguration("keyboard_device_path"),
                        "keyboard_device_name": LaunchConfiguration("keyboard_device_name"),
                        "keyboard_grab_device": ParameterValue(
                            LaunchConfiguration("keyboard_grab_device"),
                            value_type=bool,
                        ),
                        "output_twist_topic": LaunchConfiguration("raw_twist_topic"),
                        "output_buttons_topic": LaunchConfiguration("buttons_topic"),
                        "command_frame": LaunchConfiguration("keyboard_command_frame"),
                        "angular_command_frame": LaunchConfiguration(
                            "keyboard_angular_command_frame"
                        ),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("keyboard_publish_rate_hz"),
                            value_type=float,
                        ),
                        "linear_speed_m_s": ParameterValue(
                            LaunchConfiguration("keyboard_linear_speed_m_s"),
                            value_type=float,
                        ),
                        "angular_speed_rad_s": ParameterValue(
                            LaunchConfiguration("keyboard_angular_speed_rad_s"),
                            value_type=float,
                        ),
                        "fast_scale": ParameterValue(
                            LaunchConfiguration("keyboard_fast_scale"),
                            value_type=float,
                        ),
                        "slow_scale": ParameterValue(
                            LaunchConfiguration("keyboard_slow_scale"),
                            value_type=float,
                        ),
                        "low_pass_alpha": ParameterValue(
                            LaunchConfiguration("keyboard_low_pass_alpha"),
                            value_type=float,
                        ),
                        "terminal_key_hold_s": ParameterValue(
                            LaunchConfiguration("keyboard_terminal_key_hold_s"),
                            value_type=float,
                        ),
                        "gripper_pulse_s": ParameterValue(
                            LaunchConfiguration("keyboard_gripper_pulse_s"),
                            value_type=float,
                        ),
                        "deadman_key": LaunchConfiguration("keyboard_deadman_key"),
                        "gripper_key": LaunchConfiguration("keyboard_gripper_key"),
                        "positive_x_key": LaunchConfiguration("keyboard_positive_x_key"),
                        "negative_x_key": LaunchConfiguration("keyboard_negative_x_key"),
                        "positive_y_key": LaunchConfiguration("keyboard_positive_y_key"),
                        "negative_y_key": LaunchConfiguration("keyboard_negative_y_key"),
                        "positive_z_key": LaunchConfiguration("keyboard_positive_z_key"),
                        "negative_z_key": LaunchConfiguration("keyboard_negative_z_key"),
                        "positive_tool_z_key": LaunchConfiguration(
                            "keyboard_positive_tool_z_key"
                        ),
                        "negative_tool_z_key": LaunchConfiguration(
                            "keyboard_negative_tool_z_key"
                        ),
                        "fast_key": LaunchConfiguration("keyboard_fast_key"),
                        "slow_key": LaunchConfiguration("keyboard_slow_key"),
                        "quit_key": LaunchConfiguration("keyboard_quit_key"),
                    },
                ],
            )
        )

    if input_device != "none" and _as_bool(context, "start_gripper_toggle"):
        nodes.append(
            Node(
                package="arm_geomagic_teleop",
                executable="geomagic_gripper_toggle",
                name="geomagic_gripper_toggle",
                output="screen",
                parameters=[
                    LaunchConfiguration("teleop_config"),
                    {
                        "input_buttons_topic": LaunchConfiguration("buttons_topic"),
                    },
                ],
            )
        )

    return nodes


def _rviz_node(context, *args, **kwargs):
    if not _as_bool(context, "start_rviz"):
        return []

    moveit_config = build_moveit_config(pipelines=["ompl"])
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
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


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("arm_geomagic_teleop"), "config", "geomagic_teleop.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "moveit.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_device",
                default_value="geomagic",
                description="Laptop input backend: geomagic, keyboard, or none.",
            ),
            DeclareLaunchArgument(
                "teleop_config",
                default_value=default_config,
                description="YAML file for laptop-side teleop input nodes.",
            ),
            DeclareLaunchArgument(
                "raw_twist_topic",
                default_value="/arm_teleop/raw_twist_cmd",
                description="Raw teleop twist topic consumed by Jetson-side safety filter.",
            ),
            DeclareLaunchArgument(
                "buttons_topic",
                default_value="/geomagic_touch/buttons",
                description="Joy button topic consumed by gripper toggle and Jetson gate.",
            ),
            DeclareLaunchArgument(
                "start_geomagic_driver",
                default_value="true",
                description="Start omni_common/omni_state on the laptop.",
            ),
            DeclareLaunchArgument(
                "use_omni_adapter",
                default_value="true",
                description="Adapt /phantom/state to /geomagic_touch pose/buttons.",
            ),
            DeclareLaunchArgument(
                "start_geomagic_teleop",
                default_value="true",
                description="Publish raw twist intent for the Jetson safety pipeline.",
            ),
            DeclareLaunchArgument(
                "start_gripper_toggle",
                default_value="true",
                description="Let the second Geomagic button call the remote gripper action.",
            ),
            DeclareLaunchArgument(
                "keyboard_backend",
                default_value="evdev",
                description="Keyboard backend: evdev for true key release, terminal as fallback.",
            ),
            DeclareLaunchArgument(
                "keyboard_device_path",
                default_value="",
                description="Optional /dev/input/eventX keyboard device for evdev backend.",
            ),
            DeclareLaunchArgument(
                "keyboard_device_name",
                default_value="",
                description="Optional substring to select an evdev keyboard device by name.",
            ),
            DeclareLaunchArgument(
                "keyboard_grab_device",
                default_value="false",
                description="Exclusively grab the evdev keyboard device while teleop is running.",
            ),
            DeclareLaunchArgument(
                "keyboard_command_frame",
                default_value="uav_mount",
                description="Frame for keyboard linear twist commands.",
            ),
            DeclareLaunchArgument(
                "keyboard_angular_command_frame",
                default_value="tool0",
                description="Frame whose local Z axis is used for Q/E angular commands.",
            ),
            DeclareLaunchArgument(
                "keyboard_publish_rate_hz",
                default_value="100.0",
                description="Keyboard raw twist/button publish rate.",
            ),
            DeclareLaunchArgument(
                "keyboard_linear_speed_m_s",
                default_value="0.04",
                description="Nominal keyboard Cartesian linear speed.",
            ),
            DeclareLaunchArgument(
                "keyboard_angular_speed_rad_s",
                default_value="0.20",
                description="Nominal keyboard angular speed around tool Z.",
            ),
            DeclareLaunchArgument(
                "keyboard_fast_scale",
                default_value="2.0",
                description="Multiplier while the keyboard fast key is held.",
            ),
            DeclareLaunchArgument(
                "keyboard_slow_scale",
                default_value="0.35",
                description="Multiplier while the keyboard slow key is held.",
            ),
            DeclareLaunchArgument(
                "keyboard_low_pass_alpha",
                default_value="0.35",
                description="Keyboard command smoothing factor in [0, 1].",
            ),
            DeclareLaunchArgument(
                "keyboard_terminal_key_hold_s",
                default_value="0.18",
                description="Terminal backend key hold timeout; evdev ignores this.",
            ),
            DeclareLaunchArgument("keyboard_deadman_key", default_value="space"),
            DeclareLaunchArgument("keyboard_gripper_key", default_value="g"),
            DeclareLaunchArgument("keyboard_positive_x_key", default_value="w"),
            DeclareLaunchArgument("keyboard_negative_x_key", default_value="s"),
            DeclareLaunchArgument("keyboard_positive_y_key", default_value="a"),
            DeclareLaunchArgument("keyboard_negative_y_key", default_value="d"),
            DeclareLaunchArgument("keyboard_positive_z_key", default_value="z"),
            DeclareLaunchArgument("keyboard_negative_z_key", default_value="x"),
            DeclareLaunchArgument("keyboard_positive_tool_z_key", default_value="q"),
            DeclareLaunchArgument("keyboard_negative_tool_z_key", default_value="e"),
            DeclareLaunchArgument("keyboard_fast_key", default_value="leftshift"),
            DeclareLaunchArgument("keyboard_slow_key", default_value="leftctrl"),
            DeclareLaunchArgument("keyboard_quit_key", default_value="esc"),
            DeclareLaunchArgument(
                "keyboard_gripper_pulse_s",
                default_value="0.15",
                description="Duration of Joy button pulse generated by the gripper key.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
                description="Start RViz on the laptop as an operator console.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file for laptop operator view.",
            ),
            OpaqueFunction(function=_input_nodes),
            OpaqueFunction(function=_rviz_node),
        ]
    )

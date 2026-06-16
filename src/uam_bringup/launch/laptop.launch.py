from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    teleop_node = Node(
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
        condition=IfCondition(LaunchConfiguration("start_geomagic_teleop")),
    )

    gripper_toggle_node = Node(
        package="arm_geomagic_teleop",
        executable="geomagic_gripper_toggle",
        name="geomagic_gripper_toggle",
        output="screen",
        parameters=[LaunchConfiguration("teleop_config")],
        condition=IfCondition(LaunchConfiguration("start_gripper_toggle")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "teleop_config",
                default_value=default_config,
                description="YAML file for laptop-side Geomagic input nodes.",
            ),
            DeclareLaunchArgument(
                "raw_twist_topic",
                default_value="/arm_teleop/raw_twist_cmd",
                description="Raw teleop twist topic consumed by Jetson-side safety filter.",
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
                "start_rviz",
                default_value="false",
                description="Start RViz on the laptop as an operator console.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file for laptop operator view.",
            ),
            geomagic_driver_node,
            adapter_node,
            teleop_node,
            gripper_toggle_node,
            OpaqueFunction(function=_rviz_node),
        ]
    )

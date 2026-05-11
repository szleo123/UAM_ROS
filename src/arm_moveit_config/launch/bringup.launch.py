from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from arm_moveit_config_utils.launch_utils import (
    build_moveit_config,
    common_bringup_launch_arguments,
    robot_description_mappings,
)


def generate_launch_description():
    moveit_config = build_moveit_config(robot_description_mappings())

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    static_virtual_joint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "uav_mount",
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[LaunchConfiguration("ros2_controllers_file")],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_controllers")),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_controllers")),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_gripper_controller")),
    )

    delayed_controller_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        ),
        condition=IfCondition(LaunchConfiguration("start_controllers")),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(LaunchConfiguration("start_move_group")),
    )

    rviz_node = Node(
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
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    handeye_publisher = Node(
        package="easy_handeye2",
        executable="handeye_publisher",
        name="handeye_publisher",
        parameters=[
            {
                "name": LaunchConfiguration("handeye_name"),
                "calibration_file": LaunchConfiguration("handeye_calibration_file"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_handeye")),
    )

    homing_button = Node(
        package="my_arm_hardware",
        executable="homing_button.py",
        name="arm_homing_button",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("start_homing_button"),
                    "'.lower() == 'true' and '",
                    LaunchConfiguration("use_fake_hardware"),
                    "'.lower() != 'true' and '",
                    LaunchConfiguration("first_power_on"),
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    dynamics_monitor = Node(
        package="my_arm_hardware",
        executable="dynamics_monitor.py",
        name="arm_dynamics_monitor",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "joint_names": "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6",
                "rate_hz": LaunchConfiguration("dynamics_monitor_rate_hz"),
                "mode": LaunchConfiguration("dynamics_monitor_mode"),
                "torque_scale": LaunchConfiguration("dynamics_torque_scale"),
                "torque_limits_nm": LaunchConfiguration("dynamics_torque_limits_nm"),
                "low_pass_alpha": LaunchConfiguration("dynamics_torque_low_pass_alpha"),
                "enable_plot": LaunchConfiguration("dynamics_monitor_plot"),
                "plot_history_sec": LaunchConfiguration("dynamics_monitor_plot_history_sec"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_dynamics_monitor")),
    )

    joint_feedback_monitor = Node(
        package="my_arm_hardware",
        executable="joint_feedback_monitor.py",
        name="joint_feedback_monitor",
        output="screen",
        parameters=[
            {
                "joint_names": LaunchConfiguration("joint_feedback_monitor_joints"),
                "rate_hz": LaunchConfiguration("joint_feedback_monitor_rate_hz"),
                "plot": LaunchConfiguration("joint_feedback_monitor_plot"),
                "plot_history_sec": LaunchConfiguration("joint_feedback_monitor_history_sec"),
                "plot_rate_hz": LaunchConfiguration("joint_feedback_monitor_rate_hz"),
                "plot_min_rad": LaunchConfiguration("joint_feedback_monitor_min_rad"),
                "plot_max_rad": LaunchConfiguration("joint_feedback_monitor_max_rad"),
                "csv_enabled": LaunchConfiguration("joint_feedback_monitor_csv_enabled"),
                "csv_file": LaunchConfiguration("joint_feedback_monitor_csv_file"),
                "csv_append": LaunchConfiguration("joint_feedback_monitor_csv_append"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_joint_feedback_monitor")),
    )

    return LaunchDescription(
        common_bringup_launch_arguments()
        + [
            static_virtual_joint_tf,
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            delayed_controller_spawners,
            move_group_node,
            rviz_node,
            handeye_publisher,
            homing_button,
            dynamics_monitor,
            joint_feedback_monitor,
        ]
    )

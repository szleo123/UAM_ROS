from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
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
        condition=IfCondition(LaunchConfiguration("start_homing_button")),
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
        ]
    )

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from pathlib import Path
import yaml
import os

def generate_launch_description():
    # MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm.urdf.xacro")
        .robot_description_semantic(file_path="config/arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Paths
    cfg_share = get_package_share_directory("arm_moveit_config")
    ros2_controllers_path = os.path.join(cfg_share, "config", "ros2_controllers.yaml")
    rviz_config_path = os.path.join(cfg_share, "config", "moveit.rviz")
    servo_yaml_path = Path(cfg_share) / "config" / "servo.yaml"

    # Load servo.yaml safely
    with servo_yaml_path.open("r") as f:
        servo_params = yaml.safe_load(f)
    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(servo_yaml_path)
        .to_dict()
    }


    # Nodes
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
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
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    # Launch Servo server; its default topic namespace matches the executable name ("servo_server")
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            servo_params,
        ],
        output="screen",
    )

    # Your Pose→Servo bridge. Make sure these parameter names match your C++ node.
    pose_servo_tracker = Node(
        package="arm_realtime_tracker",
        executable="pose_servo_tracker_node",
        parameters=[
            {"group_name": "arm"},
            {"ee_link": "tool0"},
            {"target_frame": "base_link"},
            {"target_topic": "/desired_pose"},
            {"servo_twist_topic": "/servo_node/delta_twist_cmds"},
            {"servo_enable_topic": "/servo_node/enable"},
            {"kp_linear": 2.0},
            {"kp_angular": 2.0},
            {"pos_deadband_m": 0.003},
            {"ang_deadband_rad": 0.02},
            {"max_linear_vel": 0.25},
            {"max_angular_vel": 0.6},
            {"loop_rate_hz": 200.0},
        ],
        output="screen",
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
        robot_state_publisher_node,
        servo_node,
        pose_servo_tracker,
    ])

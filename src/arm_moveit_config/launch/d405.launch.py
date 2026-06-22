"""Convenience launch wrapper for Intel RealSense D405."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include_realsense(context, *args, **kwargs):
    upside_down = LaunchConfiguration("upside_down").perform(context).lower() == "true"
    rotation_enable = "true" if upside_down else "false"
    rotation_angle = "180.0" if upside_down else "0.0"

    return [
        GroupAction(
            scoped=True,
            actions=[
                SetLaunchConfiguration("camera_name", LaunchConfiguration("camera_name")),
                SetLaunchConfiguration("camera_namespace", LaunchConfiguration("camera_namespace")),
                SetLaunchConfiguration("serial_no", LaunchConfiguration("serial_no")),
                SetLaunchConfiguration("enable_color", LaunchConfiguration("enable_color")),
                SetLaunchConfiguration("enable_depth", LaunchConfiguration("enable_depth")),
                SetLaunchConfiguration("enable_infra1", LaunchConfiguration("enable_infra1")),
                SetLaunchConfiguration("enable_infra2", LaunchConfiguration("enable_infra2")),
                SetLaunchConfiguration(
                    "rgb_camera.color_profile", LaunchConfiguration("color_profile")
                ),
                SetLaunchConfiguration(
                    "depth_module.color_profile", LaunchConfiguration("color_profile")
                ),
                SetLaunchConfiguration(
                    "depth_module.depth_profile", LaunchConfiguration("depth_profile")
                ),
                SetLaunchConfiguration(
                    "depth_module.infra_profile", LaunchConfiguration("infra_profile")
                ),
                SetLaunchConfiguration("enable_sync", LaunchConfiguration("enable_sync")),
                SetLaunchConfiguration(
                    "pointcloud.enable", LaunchConfiguration("pointcloud_enable")
                ),
                SetLaunchConfiguration(
                    "align_depth.enable", LaunchConfiguration("align_depth_enable")
                ),
                SetLaunchConfiguration("rotation_filter.enable", rotation_enable),
                SetLaunchConfiguration("rotation_filter.rotation", rotation_angle),
                SetLaunchConfiguration("publish_tf", LaunchConfiguration("publish_tf")),
                SetLaunchConfiguration("log_level", LaunchConfiguration("log_level")),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("realsense2_camera"),
                                "launch",
                                "rs_launch.py",
                            ]
                        )
                    ),
                ),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="camera",
                description="RealSense camera name.",
            ),
            DeclareLaunchArgument(
                "camera_namespace",
                default_value="camera",
                description="ROS namespace for the D405.",
            ),
            DeclareLaunchArgument(
                "serial_no",
                default_value="",
                description="Optional camera serial number used to select a specific D405.",
            ),
            DeclareLaunchArgument(
                "enable_color",
                default_value="true",
                description="Enable the D405 color stream.",
            ),
            DeclareLaunchArgument(
                "enable_depth",
                default_value="true",
                description="Enable the D405 depth stream.",
            ),
            DeclareLaunchArgument(
                "enable_infra1",
                default_value="false",
                description="Enable infrared stream 1.",
            ),
            DeclareLaunchArgument(
                "enable_infra2",
                default_value="false",
                description="Enable infrared stream 2.",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="640,480,15",
                description="D405 color profile as width,height,fps. Use lower values to reduce Wi-Fi bandwidth.",
            ),
            DeclareLaunchArgument(
                "depth_profile",
                default_value="640,480,15",
                description="D405 depth profile as width,height,fps.",
            ),
            DeclareLaunchArgument(
                "infra_profile",
                default_value="640,480,15",
                description="D405 infrared profile as width,height,fps.",
            ),
            DeclareLaunchArgument(
                "enable_sync",
                default_value="false",
                description="Enable RealSense stream synchronization.",
            ),
            DeclareLaunchArgument(
                "pointcloud_enable",
                default_value="false",
                description="Enable the RealSense point cloud output.",
            ),
            DeclareLaunchArgument(
                "align_depth_enable",
                default_value="false",
                description="Align depth to color.",
            ),
            DeclareLaunchArgument(
                "upside_down",
                default_value="false",
                description="Rotate the output 180 degrees for upside-down camera mounting.",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="false",
                description="Publish RealSense TF frames.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Driver log level.",
            ),
            OpaqueFunction(function=_include_realsense),
        ]
    )

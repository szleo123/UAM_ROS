from glob import glob
from setuptools import setup

package_name = "arm_geomagic_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Safe Geomagic Touch Cartesian teleoperation for the UAM arm.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "geomagic_omni_state_adapter = arm_geomagic_teleop.geomagic_omni_state_adapter:main",
            "geomagic_cartesian_teleop = arm_geomagic_teleop.geomagic_cartesian_teleop:main",
            "geomagic_gripper_toggle = arm_geomagic_teleop.geomagic_gripper_toggle:main",
            "keyboard_twist_teleop = arm_geomagic_teleop.keyboard_twist_teleop:main",
            "teleop_safety_filter = arm_geomagic_teleop.teleop_safety_filter:main",
            "trajectory_deadman_gate = arm_geomagic_teleop.trajectory_deadman_gate:main",
            "teleop_test_jog = arm_geomagic_teleop.teleop_test_jog:main",
            "direct_joint_step = arm_geomagic_teleop.direct_joint_step:main",
            "trajectory_delta_probe = arm_geomagic_teleop.trajectory_delta_probe:main",
            "trajectory_command_monitor = arm_geomagic_teleop.trajectory_command_monitor:main",
        ],
    },
)

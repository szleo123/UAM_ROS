# Setup From a Blank Linux System

This document describes how to bring up this workspace on a fresh laptop for
testing the custom 6-DOF arm stack.

The assumed target system is:

- Ubuntu 22.04 LTS
- ROS 2 Humble
- MoveIt 2
- ros2_control

The workspace contains real hardware control code. Start with fake hardware,
then move to conservative real-hardware settings after the build and ROS graph
are known-good.

## 1. Install Base System Packages

Enable Ubuntu `universe` first. Some Python/dev packages live there, and this
also matches the normal ROS 2 apt setup path.

```bash
sudo add-apt-repository universe -y
sudo apt update
sudo apt install -y \
  curl gnupg lsb-release software-properties-common locales \
  git build-essential cmake pkg-config \
  python3-pip python3-venv python3-tk python3-numpy python3-matplotlib \
  python3-opencv python3-transforms3d python3-yaml \
  libserial-dev libncurses5 libncurses5-dev

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

## 2. Install ROS 2 Humble

Install the ROS 2 apt source package. This is preferred over manually adding a
long-lived key file because the package owns both the apt source and ROS signing
keyring.

```bash
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo apt update

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | cut -d '"' -f 4)

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"

sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

Add ROS to your shell:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

Initialize rosdep:

```bash
sudo rosdep init || true
rosdep update
```

## 3. Install Main ROS Dependencies

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-forward-command-controller \
  ros-humble-gripper-controllers \
  ros-humble-ros2controlcli \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-rqt-joint-trajectory-controller \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-control-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-rosidl-typesupport-c \
  ros-humble-rosidl-typesupport-cpp \
  ros-humble-rosidl-typesupport-fastrtps-c \
  ros-humble-rosidl-typesupport-fastrtps-cpp \
  ros-humble-rosidl-typesupport-introspection-c \
  ros-humble-rosidl-typesupport-introspection-cpp \
  ros-humble-rmw-fastrtps-cpp
```

The dynamics monitor uses Pinocchio. Try installing the ROS package:

```bash
sudo apt install -y ros-humble-pinocchio || true
```

If Pinocchio is unavailable, disable dynamics at launch time:

```bash
start_dynamics_monitor:=false enable_dynamics_feedforward:=false
```

## 4. Optional Hardware-Specific Packages

### Intel RealSense D405

Install the RealSense ROS 2 wrapper if camera/Aruco/hand-eye testing is needed:

```bash
sudo apt install -y ros-humble-realsense2-camera ros-humble-librealsense2*
```

### Geomagic Touch

The Geomagic driver in `src/geomagic_touch_ros2` links against OpenHaptics
libraries named `HD` and `HDU`.

Install OpenHaptics for Linux before building this workspace if Geomagic teleop
is needed. After installing it, make sure the headers and libraries are visible
to CMake. Typical files include:

- `HD/hd.h`
- `HL/hl.h`
- `HDU/hdu.h`
- `libHD.so`
- `libHDU.so`

If the laptop does not need Geomagic teleop yet, the rest of the arm stack can
still be tested without the device.

## 5. Get the Workspace

If this repository is not already present on the laptop:

```bash
cd ~
git clone <your-repo-url> UAM_ROS
cd ~/UAM_ROS
```

If the repository is already copied to the laptop:

```bash
cd ~/UAM_ROS
```

Fetch pinned third-party ROS packages:

```bash
vcs import src < dependencies.repos
./scripts/apply_third_party_patches.sh
```

Install package dependencies that rosdep can resolve:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

## 6. Build the Workspace

```bash
cd ~/UAM_ROS
source /opt/ros/humble/setup.bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

On a low-power laptop, build with fewer workers:

```bash
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Add the workspace to your shell:

```bash
echo 'source ~/UAM_ROS/install/setup.bash' >> ~/.bashrc
```

## 7. First Test: Fake Hardware

Run fake hardware before connecting motors:

```bash
cd ~/UAM_ROS
source install/setup.bash

ros2 launch arm_moveit_config demo.launch.py \
  start_dynamics_monitor:=false \
  start_joint_feedback_monitor:=false
```

In another terminal:

```bash
source ~/UAM_ROS/install/setup.bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Expected signs:

- `joint_state_broadcaster` is active.
- `arm_controller` is active.
- `gripper_controller` is active if enabled.
- `/joint_states` publishes `joint_1` through `joint_6` and `tool_joint`.

Do not continue to real hardware until fake hardware works.

## 8. USB Permissions and Device Names

Allow the user to access serial devices:

```bash
sudo usermod -aG dialout $USER
```

Log out and log back in.

Find connected devices:

```bash
ls -l /dev/serial/by-id
ls -l /dev/serial/by-path
dmesg -w
```

Prefer stable paths such as:

```text
/dev/serial/by-id/<stm32-device>
/dev/serial/by-id/<gripper-device>
```

over unstable names like:

```text
/dev/ttyACM0
/dev/ttyUSB0
```

## 9. First Real Hardware Bring-Up

Use conservative settings for the first hardware run on the new laptop:

```bash
cd ~/UAM_ROS
source install/setup.bash

ros2 launch my_arm_hardware myarm.launch.py \
  serial_port:=/dev/serial/by-id/<stm32-device> \
  gripper_port:=/dev/serial/by-id/<gripper-device> \
  stm32_control_mode:=0 \
  enable_dynamics_feedforward:=false \
  start_dynamics_monitor:=false \
  enable_stm32_zero_trigger:=false
```

In another terminal, monitor:

```bash
source ~/UAM_ROS/install/setup.bash

ros2 topic echo /arm_homing/status_text
ros2 topic echo /arm_homing/stm32_sys_state
ros2 topic echo /joint_states
```

Expected signs:

- `/arm_homing/stm32_sys_state` reports STM32 state changes.
- `/joint_states` reports finite positions.
- Feedback is stable before any motion command is sent.
- The hardware interface does not repeatedly report stale feedback.

## 10. Normal Real Hardware Bring-Up

After the conservative test is successful, use the tuned defaults:

```bash
cd ~/UAM_ROS
source install/setup.bash

ros2 launch my_arm_hardware myarm.launch.py \
  serial_port:=/dev/serial/by-id/<stm32-device> \
  gripper_port:=/dev/serial/by-id/<gripper-device>
```

The default launch starts:

- robot_state_publisher
- ros2_control
- joint_state_broadcaster
- arm trajectory controller
- gripper controller
- MoveIt move_group
- RViz
- homing button UI
- STM32 MIT gain tuner if enabled
- dynamics monitor if enabled
- joint feedback monitor if enabled

## 11. Homing and STM32 Safety Notes

Important:

- Start with `enable_stm32_zero_trigger:=false` on a newly moved laptop.
- Only use `enable_stm32_zero_trigger:=true` when intentionally testing the MCU
  joint-3 zeroing sequence.
- Keep an emergency stop or power cut procedure physically available.
- Confirm that the STM32 firmware flashed on the board matches the ROS protocol
  in this workspace.

Useful status topics:

```bash
ros2 topic echo /arm_homing/status_text
ros2 topic echo /arm_homing/state
ros2 topic echo /arm_homing/stm32_sys_state
ros2 topic echo /my_arm_system/stm32_control_mode
ros2 topic echo /my_arm_system/stm32_mit_gains_current
```

## 12. Small Motion Test

After real feedback is stable, send a tiny joint command from the helper node:

```bash
ros2 run arm_geomagic_teleop direct_joint_step \
  --ros-args \
  -p joint_trajectory_topic:=/arm_controller/joint_trajectory \
  -p joint_name:=joint_1 \
  -p delta_rad:=0.03 \
  -p duration_s:=3.0
```

Watch:

```bash
ros2 topic echo /joint_states
ros2 topic echo /arm_controller/controller_state
```

## 13. Geomagic Teleop

First run teleop in dry-run mode:

```bash
ros2 launch arm_geomagic_teleop geomagic_teleop.launch.py \
  dry_run:=true \
  require_homing:=false
```

Dry-run output goes to:

```text
/arm_teleop/dry_run_joint_trajectory
```

When the teleop trajectory monitor looks sane, run against the real controller:

```bash
ros2 launch arm_geomagic_teleop geomagic_teleop.launch.py \
  dry_run:=false \
  require_homing:=true
```

Useful teleop status topics:

```bash
ros2 topic echo /arm_teleop/teleop_status
ros2 topic echo /arm_teleop/safety_status
ros2 topic echo /arm_teleop/trajectory_gate_status
ros2 topic echo /arm_teleop/gripper_status
```

The final trajectory gate can be armed/disarmed with:

```bash
ros2 service call /trajectory_deadman_gate/set_armed std_srvs/srv/SetBool "{data: true}"
ros2 service call /trajectory_deadman_gate/set_armed std_srvs/srv/SetBool "{data: false}"
```

## 14. RealSense, Aruco, and Hand-Eye Calibration

Start the D405 wrapper:

```bash
ros2 launch arm_moveit_config d405.launch.py
```

Start Aruco detection:

```bash
ros2 launch ros2_aruco aruco_recognition.launch.py
```

The patched Aruco config defaults to:

```text
image_topic: /camera/camera/color/image_raw
camera_info_topic: /camera/camera/color/camera_info
target_marker_id: 66
```

Run hand-eye calibration helper:

```bash
ros2 launch easy_handeye2 calibrate_with_params.launch.py \
  name:=handeye_calib \
  calibration_type:=eye_on_base \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=aruco_marker_66 \
  robot_base_frame:=base_link \
  robot_effector_frame:=tool0
```

Publish a saved calibration by name:

```bash
ros2 launch easy_handeye2 publish.launch.py name:=handeye_calib
```

Or publish from an explicit file:

```bash
ros2 launch easy_handeye2 publish.launch.py \
  calibration_file:=/absolute/path/to/calibration.yaml
```

## 15. Common Troubleshooting

### Package not found

Source both ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/UAM_ROS/install/setup.bash
```

### ROS apt key error

If `sudo apt update` reports:

```text
NO_PUBKEY F42ED6FBAB17C654
The repository 'http://packages.ros.org/ros2/ubuntu jammy InRelease' is not signed.
```

remove the old manual ROS source and install the ROS apt source package:

```bash
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo apt update

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | cut -d '"' -f 4)

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"

sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update
```

### Serial permission denied

Check group membership:

```bash
groups
```

If `dialout` is missing:

```bash
sudo usermod -aG dialout $USER
```

Then log out and log back in.

### Wrong USB port

Check stable device links:

```bash
ls -l /dev/serial/by-id
ls -l /dev/serial/by-path
```

Launch again with explicit `serial_port:=...` and `gripper_port:=...`.

### Pinocchio missing

Disable dynamics:

```bash
start_dynamics_monitor:=false enable_dynamics_feedforward:=false
```

### Geomagic build fails

Make sure OpenHaptics headers and libraries are installed and visible to CMake.
If teleop is not needed, temporarily remove or skip `src/geomagic_touch_ros2`
while validating the rest of the workspace.

### Real hardware does not move

Check:

```bash
ros2 topic echo /arm_homing/status_text
ros2 topic echo /arm_homing/state
ros2 topic echo /joint_states
ros2 control list_controllers
```

The hardware interface intentionally sends safe-lock frames until feedback and
release conditions are satisfied.

### Build error: No `rosidl_typesupport_c` found

If `colcon build` fails with:

```text
CMake Error ... get_used_typesupports.cmake
No 'rosidl_typesupport_c' found
```

install the missing ROS interface typesupport packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-rosidl-typesupport-c \
  ros-humble-rosidl-typesupport-cpp \
  ros-humble-rosidl-typesupport-fastrtps-c \
  ros-humble-rosidl-typesupport-fastrtps-cpp \
  ros-humble-rosidl-typesupport-introspection-c \
  ros-humble-rosidl-typesupport-introspection-cpp \
  ros-humble-rmw-fastrtps-cpp

source /opt/ros/humble/setup.bash
rm -rf build/arm_realtime_tracker install/arm_realtime_tracker log
colcon build --symlink-install
```

## 16. Recommended Bring-Up Order

1. Install OS dependencies.
2. Install ROS 2 Humble.
3. Import third-party repos and apply patches.
4. Build.
5. Run fake hardware.
6. Identify USB devices.
7. Run real hardware in conservative position-only mode.
8. Confirm stable feedback.
9. Test a tiny joint move.
10. Enable normal launch defaults.
11. Test camera/Aruco/hand-eye.
12. Test Geomagic teleop in dry-run.
13. Test Geomagic teleop on hardware.

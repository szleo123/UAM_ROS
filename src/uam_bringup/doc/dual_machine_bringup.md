# Dual-Machine Bringup

This package adds role-based launch wrappers for running the same workspace on
both the Jetson NX mounted on the UAV and the ThinkPad operator laptop.

The existing one-machine launch files are intentionally left in place. Use this
package when the system is split across the Jetson and laptop.

## Machine Roles

Jetson NX on the UAV owns hardware-facing and safety-critical execution:

- STM32 USB CDC arm hardware interface
- gripper serial hardware
- RealSense D405 and ArUco detection when enabled
- robot state publisher, ros2_control, controllers, MoveIt move_group
- MoveIt Servo, teleop safety filter, and final trajectory gate
- homing state, STM32 gain topics, dynamics and feedback monitors when enabled

ThinkPad operator laptop owns human input and operator visualization:

- Geomagic Touch driver
- adapter from `/phantom/state` to `/geomagic_touch/pose` and `/geomagic_touch/buttons`
- raw teleop twist intent on `/arm_teleop/raw_twist_cmd`
- gripper button action client
- optional RViz operator view
- optional monitor windows for Jetson-published feedback and command topics

The laptop sends intent. The Jetson validates and executes.

## Network Environment

Both machines should use the same ROS 2 distribution and workspace revision.
Before launching, set matching ROS network variables in every terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/UAM_ROS/install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Use the same `ROS_DOMAIN_ID` on both machines. If Wi-Fi discovery is unreliable,
configure a Fast DDS discovery server or static DDS peer configuration instead
of relying on multicast discovery.

Keep the machines time-synchronized. `chrony` is a good default on Ubuntu.

## Build Commands

Use the same source revision on both machines, but build different package sets.
The Geomagic/OpenHaptics driver is only needed on the laptop; OpenHaptics does
not support the Jetson ARM64 target.

Install Ninja if you want faster local builds:

```bash
sudo apt install ninja-build
```

Jetson/UAV build:

```bash
colcon build --symlink-install --parallel-workers 4 \
  --packages-skip omni_common omni_broadcaster omni_description \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

The skipped packages are only for the Geomagic Touch/OpenHaptics side. Jetson
still builds and runs the hardware interface, MoveIt, Servo, safety filter,
trajectory gate, camera, and monitor infrastructure.
Do not skip `arm_geomagic_teleop` on the Jetson; its Jetson-side safety filter,
trajectory gate, and command monitors do not require OpenHaptics.

Laptop/operator build:

```bash
colcon build --symlink-install --parallel-workers 4 \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

For keyboard fallback teleop with true key press/release handling, install
`evdev` and allow the operator user to read keyboard input events:

```bash
sudo apt install python3-evdev
sudo usermod -aG input $USER
```

Log out and back in after changing group membership.

For a Jetson-only binary build that will not be copied to another CPU, you can
add native ARM CPU tuning:

```bash
colcon build --symlink-install --parallel-workers 4 \
  --packages-skip omni_common omni_broadcaster omni_description \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG -mcpu=native"
```

Use the native flags only for local Jetson builds. Plain `Release` is safer when
you want portable binaries across machines.

If you switch an existing build directory from Makefiles to Ninja and see
symlink or generator errors, remove the stale build artifacts before rebuilding:

```bash
rm -rf build install log
```

## Jetson Launch

Start the hardware-facing UAV stack on the Jetson:

```bash
ros2 launch uam_bringup jetson.launch.py \
  serial_port:=/dev/serial/by-id/<stm32-device> \
  gripper_port:=/dev/serial/by-id/<gripper-device> \
  start_d405:=true \
  start_aruco:=true
```

For first tests, keep the final teleop trajectory gate disarmed:

```bash
ros2 launch uam_bringup jetson.launch.py \
  serial_port:=/dev/serial/by-id/<stm32-device> \
  gripper_port:=/dev/serial/by-id/<gripper-device> \
  armed_on_start:=false \
  dry_run:=true
```

After verifying topics and safety behavior, use `dry_run:=false` and arm the
gate with:

```bash
ros2 service call /trajectory_deadman_gate/set_armed std_srvs/srv/SetBool "{data: true}"
```

## Laptop Launch

Start the operator-side Geomagic input stack on the laptop:

```bash
ros2 launch uam_bringup laptop.launch.py
```

The laptop input backend is selectable. Geomagic remains the default:

```bash
ros2 launch uam_bringup laptop.launch.py input_device:=geomagic
```

If the Geomagic Touch is unavailable, start keyboard fallback input instead:

```bash
ros2 launch uam_bringup laptop.launch.py input_device:=keyboard
```

Keyboard fallback publishes the same teleop contract as the Geomagic adapter:
`/arm_teleop/raw_twist_cmd` and `/geomagic_touch/buttons`. The Jetson-side
safety filter, MoveIt Servo, trajectory deadman gate, homing checks, and
emergency-stop behavior are unchanged.

Default keyboard controls:

- hold Space: deadman
- W/S: positive/negative X command
- A/D: positive/negative Y command
- Z/X: positive/negative Z command
- Q/E: rotate around the current `tool0` Z axis
- G: gripper toggle pulse
- Left Shift: fast scale
- Left Ctrl: slow scale
- Esc: quit and publish zero command

Tune keyboard speeds and scaling from launch:

```bash
ros2 launch uam_bringup laptop.launch.py \
  input_device:=keyboard \
  keyboard_linear_speed_m_s:=0.03 \
  keyboard_angular_speed_rad_s:=0.12 \
  keyboard_fast_scale:=1.5 \
  keyboard_slow_scale:=0.25
```

The preferred keyboard backend is `evdev`, because it sees real key-release
events for the deadman key. If auto-selection picks the wrong input device, list
devices with `ls /dev/input/event*` and pass the correct one:

```bash
ros2 launch uam_bringup laptop.launch.py \
  input_device:=keyboard \
  keyboard_device_path:=/dev/input/eventX
```

The keyboard device is not exclusively grabbed by default, so Ctrl-C and other
desktop input still work. For a dedicated bench test you can request exclusive
input with `keyboard_grab_device:=true`.

A terminal fallback exists for quick tests, but it cannot see true key release:

```bash
ros2 launch uam_bringup laptop.launch.py \
  input_device:=keyboard \
  keyboard_backend:=terminal
```

Start the same stack with RViz:

```bash
ros2 launch uam_bringup laptop.launch.py start_rviz:=true
```

The laptop launch does not start MoveIt Servo or the final trajectory gate.
Those stay on the Jetson.

## Laptop Operator Tools

Start RViz plus laptop-side monitor windows:

```bash
ros2 launch uam_bringup operator_tools.launch.py
```

By default this starts:

- RViz
- homing/zeroing, emergency arm-stop, and emergency gripper-open buttons
- joint feedback/reference monitor
- final trajectory command monitor

The monitor windows run on the laptop, but they subscribe to topics published by
the Jetson over ROS 2. This keeps plotting and operator visualization off the
Jetson while leaving hardware execution on the Jetson.

The homing button also runs on the laptop by default. It watches
`/arm_homing/state`, `/arm_homing/status_text`, and `/arm_homing/stm32_sys_state`
from the Jetson. When you click Confirm Drop Pose / Zero Joint 3, it calls the
Jetson service `/arm_homing/confirm_drop_pose`; the Jetson hardware interface is
still the only process that sends the STM32 zeroing packet.

The same window also includes Emergency Stop Arm and Emergency Open Gripper.
Emergency Stop Arm calls `/arm_emergency_stop/trigger` on the Jetson. That
service requests cancellation of active `/arm_controller/follow_joint_trajectory`
goals and publishes a short hold-current-position trajectory using the latest
`/joint_states`. Emergency Open Gripper requests cancellation of active
`/gripper_controller/gripper_cmd` goals and then sends the configured open
position. Both buttons are also available in `single_machine.launch.py` because
it uses the shared `arm_moveit_config` bringup path.

Optional dynamics preview plotting is available on a safe laptop-local topic:

```bash
ros2 launch uam_bringup operator_tools.launch.py start_dynamics_preview:=true
```

This uses `/operator_tools/dynamics_preview_torques_nm` by default, not the
hardware feedforward topic `/arm_dynamics/torques_nm`.

Remote write-capable tools are disabled unless explicitly guarded:

```bash
ros2 launch uam_bringup operator_tools.launch.py \
  enable_write_tools:=true \
  start_mit_gain_tuner:=true
```

The MIT gain tuner publishes to `/my_arm_system/stm32_mit_gains_cmd`, which the
Jetson hardware interface consumes in FULL MIT mode. Use it for deliberate bench
tuning sessions, not as a default UAV operator display.

## Gripper Maintenance

The gripper serial protocol is owned by the Jetson hardware interface. The
laptop operator UI only calls Jetson ROS services, so there should not be a
second process opening the gripper USB port.

The homing/operator window includes a gripper section by default:

```bash
ros2 launch uam_bringup operator_tools.launch.py
```

It polls `/gripper/query_status` and shows position, current, temperature, and
protection flags. The recovery buttons call:

- `/gripper/clear_fault`
- `/gripper/clear_fault_and_open`

`Clear Fault + Open` is the preferred branch-release action because it clears
latched stall/over-current faults, re-enables the actuator, and sends an open
command. Over-temperature faults cannot be cleared by command; the actuator
must cool below its recovery temperature.

Protection parameter writes are guarded separately from STM32 write tools:

```bash
ros2 launch uam_bringup operator_tools.launch.py \
  enable_gripper_parameter_writes:=true
```

This enables editing the actuator protection registers in RAM:

- over-current limit, register `32~33`, range `300..1500 mA`
- over-temperature limit, register `98~99`, unit `degC * 10`
- recovery temperature, register `100~101`, unit `degC * 10`

To allow saving the current actuator RAM settings into internal Flash so they
survive power cycling, also enable:

```bash
ros2 launch uam_bringup operator_tools.launch.py \
  enable_gripper_parameter_writes:=true \
  enable_gripper_flash_save:=true
```

Only save to Flash after a temporary RAM setting has been tested on the bench.

### DYNAMIXEL XW430 Snow-Sampling Gripper

The same ROS gripper surface can be used with a DYNAMIXEL XW430-T200-R backend:

```bash
ros2 launch uam_bringup jetson.launch.py \
  gripper_backend:=dynamixel_xw430 \
  gripper_port:=/dev/ttyUSB0 \
  gripper_baudrate:=57600
```

The expected hardware path is Jetson USB to U2D2/USB2Dynamixel, then RS485 to
the XW430 with a proper external DYNAMIXEL power supply. Conservative bench
defaults are used until the actual mechanism is calibrated:

- `dynamixel_id:=1`
- `dynamixel_open_position_ticks:=2048`
- `dynamixel_close_position_ticks:=2600`
- `dynamixel_goal_current_ma:=150.0`
- `dynamixel_open_current_ma:=150.0`
- `dynamixel_current_limit_ma:=500.0`
- `dynamixel_temperature_limit_c:=75.0`

The DYNAMIXEL backend configures current-based position mode by default. Normal
gripper commands write `Goal Current(102)` and `Goal Position(116)`. Status reads
`Present Current(126)`, `Present Position(132)`, `Present Temperature(146)`, and
`Hardware Error Status(70)`.

DYNAMIXEL protection settings differ from the branch gripper. `Current Limit(38)`
and `Temperature Limit(31)` are EEPROM fields, so changing them is persistent and
requires torque to be disabled briefly. There is no recovery-temperature register
on this backend; the operator UI marks it as unavailable after loading protection
data.

## Joint-3 Zeroing From Laptop

For split mode, keep the Jetson headless:

```bash
ros2 launch uam_bringup jetson.launch.py start_homing_button:=false
```

Then start laptop operator tools:

```bash
ros2 launch uam_bringup operator_tools.launch.py
```

The laptop homing window shows the Jetson homing state. When it reaches
`WAITING_OPERATOR_DROP_POSE`, move the arm to the required drop pose if needed,
then click Confirm Drop Pose / Zero Joint 3. The same action can be done from
any terminal with:

```bash
ros2 service call /arm_homing/confirm_drop_pose std_srvs/srv/Trigger {}
```

## Topics Across Wi-Fi

Laptop to Jetson:

- `/geomagic_touch/pose`
- `/geomagic_touch/buttons` from Geomagic or keyboard fallback
- `/arm_teleop/raw_twist_cmd`
- `/arm_emergency_stop/trigger` for emergency arm stop
- `/gripper_controller/gripper_cmd` action requests
- `/gripper_controller/gripper_cmd/_action/cancel_goal` for emergency gripper release
- optional `/my_arm_system/stm32_mit_gains_cmd` when remote write tools are enabled
- optional `/arm_dynamics/torques_nm` when remote manual torque tuning is enabled
- optional RViz/MoveIt requests to Jetson `move_group`

Jetson to laptop:

- `/joint_states`
- `/arm_controller/controller_state`
- `/tf` and `/tf_static`
- `/arm_homing/state`
- `/arm_homing/status_text`
- `/my_arm_system/raw_feedback_velocities_rad_s`
- `/my_arm_system/dynamics_tau_ff_nm`
- `/my_arm_system/stm32_control_mode`
- `/my_arm_system/stm32_mit_gains_current`
- `/arm_teleop/teleop_status`
- `/arm_teleop/safety_status`
- `/arm_teleop/trajectory_gate_status`
- `/arm_controller/joint_trajectory`
- optional camera, ArUco, and hand-eye topics for display

## Safety On Disconnect

The Jetson-side teleop safety filter and trajectory gate use freshness checks
for raw twist, buttons, homing state, and active command state. If Wi-Fi drops
or the laptop process stops, the Jetson sees stale input and blocks teleop
motion locally.

When the trajectory gate has been forwarding teleop motion and then blocks for
`deadman_released`, `homing_not_ready`, or `disarmed`, it also calls the shared
`/arm_emergency_stop/trigger` service. The normal no-active-command case only
publishes the gate's local hold trajectory and does not spam emergency-stop
requests.

The final trajectory gate can publish a hold trajectory when motion becomes
blocked. The hardware interface also sends STM32 safe-lock frames when homing is
not released or fresh STM32 feedback is unavailable.

## One-Machine Compatibility

Existing launch files remain valid. This package also provides a wrapper:

```bash
ros2 launch uam_bringup single_machine.launch.py
```

To include the original all-in-one Geomagic teleop stack on one machine:

```bash
ros2 launch uam_bringup single_machine.launch.py \
  start_geomagic_teleop_stack:=true
```

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
- homing/zeroing button
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
- `/geomagic_touch/buttons`
- `/arm_teleop/raw_twist_cmd`
- `/gripper_controller/gripper_cmd` action requests
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

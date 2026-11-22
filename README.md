# FeeTech SCS / STS ROS 2 driver — PyLoT fork

ROS 2 Humble driver and examples for FeeTech serial servos. This project builds
on the work of **Ar-ray and fateshelled**; thank you to the upstream authors.
PyLoT maintains separate SO-ARM100 and pan-tilt development branches.

## Choose a branch first

| Branch | Purpose | Main interfaces / executables |
|---|---|---|
| `dev_so100` (default) | SO-ARM100 arm, IDs 1–6, calibrated joint angles | `calibration_servo_config`, `read_write_position`, `ping`, `read_pos`, `read_spd` |
| `dev_sts` | STS3215 pan-tilt head | `pantilt_read_write_node`, `ping`, `read_pos_simple`, `read_pos_node`, `read_spd` |

These branches are not interchangeable. `dev_sts` does not build the SO100
calibration/read-write executables; `dev_so100` does not build the pan-tilt node.
The former `write_pos` example is present as source but commented out in both
branches' CMake configuration, so it is not an installed executable.
The driver interfaces cover SCS and STS families; the fork's README previously
listed SCS0009 and STS3215 as supported. Other devices require validation.

## Install common dependencies

Start from Ubuntu 22.04 / ROS 2 Humble with rosdep installed and initialized.
Choose **one** driver branch per workspace.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/feetech_ws/src
cd ~/feetech_ws/src
git clone --branch humble https://github.com/HarvestX/h6x_serial_interface.git
# SO100 arm; replace dev_so100 with dev_sts for a pan-tilt workspace.
git clone --branch dev_so100 https://github.com/PyLoT-Robotics/feetech_scs_ros2_driver.git
```

For `dev_sts` also provide `dynamixel_sdk_custom_interfaces`, which defines the
`SetPosition` message and `GetPosition` service (the actuators here remain FeeTech):

```bash
cd ~/feetech_ws/src
git clone --branch humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

That repository contains the interface package under `ros/`. If your workspace
already provides it, do not clone another copy.

Then build the selected branch:

```bash
cd ~/feetech_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to feetech_sts_example
source install/setup.bash
ros2 pkg executables feetech_sts_example
```

Source `/opt/ros/humble/setup.bash` and `~/feetech_ws/install/setup.bash` in each
new terminal. Do not switch branch underneath a running driver. Use separate
workspaces for the two branches to avoid old installed executables remaining
visible after a branch switch.

## Connection check (both branches)

Supply the actual serial device and servo ID. Serial access must be allowed for
your user (typically through the `dialout` group).

```bash
ros2 run feetech_sts_example ping 1 /dev/ttyACM0 1000000
```

On `dev_so100`, read position with:

```bash
ros2 run feetech_sts_example read_pos 1 /dev/ttyACM0 1000000
```

On `dev_sts`, the equivalent simple executable is `read_pos_simple`, not
`read_pos` (that name is used for a component library).
`read_pos_simple` takes only port and baud arguments and polls IDs 1 and 2:

```bash
ros2 run feetech_sts_example read_pos_simple /dev/ttyACM0 1000000
```

Confirm power, baud rate, unique IDs, and port permissions if there is no reply.

## SO-ARM100 (`dev_so100`)

### Calibrate and save the mapping

The calibration tool accepts positional arguments: port, baud rate, output YAML.
It scans servo IDs 1–6. Follow its interactive prompts and record the mechanical
range of each joint; the generated YAML maps input angles to servo positions.

```bash
mkdir -p ~/so100_config
cd ~/so100_config
ros2 run feetech_sts_example calibration_servo_config /dev/ttyACM0 1000000 servo_config.yaml
```

### Run the joint driver

```bash
ros2 run feetech_sts_example read_write_position --ros-args \
  -p config_file:=$HOME/so100_config/servo_config.yaml
```

This executable currently fixes the serial port to **`/dev/ttyACM0`** and baud rate
to **1,000,000** in source. Unlike the calibration tool and pan-tilt node, it has
no port/baud ROS parameters. A different device requires adapting those source
constants and rebuilding. The node enables torque on responding servos at startup.

| Topic | Type | Direction | Units / names |
|---|---|---|---|
| `/joint_command` | `sensor_msgs/msg/JointState` | Subscribe | `name`: servo-ID strings (`'1'`–`'6'`); `position`: radians |
| `/joint_states_raw` | `sensor_msgs/msg/JointState` | Publish | Servo-ID strings and measured angles in radians |

A command example, after checking the calibration and mechanism's range:

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: ['1'], position: [0.0]}"
ros2 topic echo /joint_states_raw --once
```

Parameters declared by this node are `config_file` (default `servo_config.yaml`,
relative to the working directory), `publish_rate` (10 Hz), and `qos_depth` (10).
For RViz, [kinematics_so100](https://github.com/PyLoT-Robotics/kinematics_so100)
provides `translate_joint_name` to map `/joint_states_raw` IDs to URDF names on
`/joint_states`. A GUI `/joint_states` publisher alone does not command the arm.

## Pan-tilt (`dev_sts`)

Build a separate workspace using `--branch dev_sts` in the clone command above,
including `dynamixel_sdk_custom_interfaces`.

```bash
ros2 run feetech_sts_example pantilt_read_write_node --ros-args \
  -p port_name:=/dev/ttyACM0 -p baudrate:=1000000 \
  -p tilt_id:=1 -p pan_id:=2
```

The default device is `/dev/ttyUSB_sts3215`; use `port_name` for another path.
Both IDs are torque-enabled on startup and torque-disabled on normal shutdown.

| Interface | Type | Purpose |
|---|---|---|
| `/set_position` | `dynamixel_sdk_custom_interfaces/msg/SetPosition` | Position commands in raw servo ticks |
| `/present_position` | Same message | Measured ticks, each servo polled at 10 Hz |
| `/get_position` | `dynamixel_sdk_custom_interfaces/srv/GetPosition` | Read one ID; failure returns position `-1` |

```bash
ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "{id: 2}"
ros2 topic echo /present_position
```

Pan-tilt values are **ticks, not radians**. Current source clamps the configured
tilt servo to 0–1200 ticks. Those limits and origin constants are mechanism-specific;
check them before sending motion commands. Additional parameters include
`qos_depth`, `scan_id_start`, `scan_id_end`, `read_retry_count`, and
`read_retry_interval_ms` for initialization diagnostics.

## Troubleshooting / validation

- Missing executable: check `git branch --show-current` and
  `ros2 pkg executables feetech_sts_example`; use the correct branch/workspace.
- Missing `h6x_serial_interface`: clone its `humble` branch into the same workspace.
- Missing `dynamixel_sdk_custom_interfaces`: provide it for `dev_sts`, then rebuild.
- No RViz movement: arm feedback is `/joint_states_raw`; run the joint-name converter.
- No arm motion: inspect calibration path, ID strings and radian command units.

The branch matrix, executable names and topic/parameter descriptions were checked
against source. This README update does not constitute a clean Humble build or a
hardware test.

## License

[Apache-2.0](LICENSE). Preserve upstream attribution and license notices.

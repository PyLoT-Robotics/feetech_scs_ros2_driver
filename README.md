## Acknowledgement
This project is forked from the original repository created by **Ar-ray and fateshelled**.

I would like to sincerely thank the original author for making their work open-source and publicly available.
This repository builds upon that foundation with additional modifications and experiments for education.

## Support 📜⚙️

### ROS2 Distro 🐢

| ROS2 Distro | Support |
| --- | --- |
| Humble        | ✔️      |

### FeeTech-SCS ⚙️

https://pages.switch-science.com/comparison/feetech-servos#serial

| Product Name | Support |
| --- | --- |
| SCS0009 | ✔️ |
| SCS215 | |
| SCS115 | |
| SCS20-360T | |
| SCS15 | |

### FeeTech-STS

| Product Name | Support |
| --- | --- |
| STS3032 |  |
| STS3215 | ✔️ |

<br>

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/HarvestX/h6x_serial_interface.git -b humble
git clone https://github.com/basalte1199/feetech_scs_ros2_driver -b dev_so100
git clone https://github.com/basalte1199/kinematics_so100

cd ../

colcon build
```

## Usage (Example)

Run `ros2 run <target_exec> <ID> <port> <baudrate>` to execute the example.

### Ping connection

```bash
ros2 run feetech_sts_example ping 1 /dev/ttyUSB0 1000000
```

### Read position

```bash
ros2 run feetech_sts_example read_pos 1 /dev/ttyUSB0 1000000
```

### Write Position

```bash
ros2 run feetech_sts_example write_pos 1 /dev/ttyUSB0 1000000
```

## Usage(So-100 arm)
### Calibration
```bash
ros2 run feetech_sts_example calibration_servo_config
```


### Launch read and write position
```bash
ros2 run feetech_sts_example read_write_position
```


### Check the output
```bash
ros2 topic echo /joint_state
```


### Enter position (example)
The unit of position is radians.

```bash
ros2 topic pub /joint_command sensor_msgs/msg/JointState "{name: ['1'], position: [0.0]}" --once

ros2 topic pub /joint_command sensor_msgs/msg/JointState "{name: ['1','2'], position: [0.0, 0.0]}" --once

```

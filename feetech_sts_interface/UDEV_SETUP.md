# STS3215 udev Setup Guide

## Overview

This package includes udev rules for the **Feetech STS3215 servo control board** (CH340-based USB serial adapter).

The udev rule will:
- Create a convenient symbolic link `/dev/ttyUSB_sts3215` for the device
- Grant proper permissions for reading/writing
- Make the device reliably accessible after system restarts

## Device Info

- **Chip**: CH340 (QinHeng Electronics)
- **USB ID**: `1a86:7523`
- **Default symlink**: `/dev/ttyUSB_sts3215`
- **Serial port baud rate**: Configure as needed in your application (typically 1 Mbps for STS3215)

## Installation

### Option 1: Automatic Installation (Recommended)

After building the package, run:

```bash
cd install/feetech_sts_interface
sudo ./share/feetech_sts_interface/scripts/install_udev_rules.sh
```

Or if you're in the source directory:

```bash
cd src/drivers/feetech_scs_ros2_driver/feetech_sts_interface
sudo ./scripts/install_udev_rules.sh
```

### Option 2: Manual Installation

Copy the rules file directly:

```bash
sudo cp udev_rules/99-sts3215.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Verification

After installation, connect the STS3215 board via USB and verify:

```bash
# List USB devices
lsusb | grep "1a86:7523"

# Check if symlink exists
ls -la /dev/ttyUSB_sts3215

# Monitor device connection (in another terminal)
udevadm monitor --property
```

When you connect the device, you should see:

```
/dev/ttyUSB_sts3215 -> ../../ttyUSB0  (or similar)
```

## Troubleshooting

### Device not appearing as `/dev/ttyUSB_sts3215`

1. **Rules not loaded**: Verify the rule file is in `/etc/udev/rules.d/`:
   ```bash
   ls -la /etc/udev/rules.d/99-sts3215.rules
   ```

2. **Rules syntax error**: Check udev logs:
   ```bash
   sudo journalctl -u systemd-udevd | tail -20
   ```

3. **Device already connected**: Disconnect and reconnect the board:
   ```bash
   sudo udevadm trigger
   ```

### Permission denied when opening port

Ensure your user is in the `dialout` group:

```bash
# Add current user to dialout group
sudo usermod -a -G dialout $USER

# Apply group changes (logout/login required, or use)
newgrp dialout

# Verify membership
groups $USER
```

## Using the device in ROS 2

Configure your launch file or parameter to use the device:

```yaml
# In your launch file or config:
port: /dev/ttyUSB_sts3215
baudrate: 1000000  # Adjust as needed
```

Or in Python:

```python
port_name = '/dev/ttyUSB_sts3215'
baudrate = 1000000
```

## System Integration

The udev rule is automatically installed to `/etc/udev/rules.d/` during the installation process. To persist across system restarts, no additional configuration is needed.

To remove the rule later:

```bash
sudo rm /etc/udev/rules.d/99-sts3215.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

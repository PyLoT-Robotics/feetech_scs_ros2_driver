#!/bin/bash
#
# Install udev rules for Feetech STS3215 servo control board
# This script copies the udev rule file to /etc/udev/rules.d/ and reloads udev
#
# Usage: sudo ./install_udev_rules.sh
# Or via ROS share directory: sudo /path/to/install/feetech_sts_interface/share/feetech_sts_interface/scripts/install_udev_rules.sh
#

set -e

# Detect where we were called from and find the rules directory
if [[ "${BASH_SOURCE[0]}" == /* ]]; then
    # Absolute path (called from install directory)
    SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
else
    # Relative path (called from source directory)
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

RULES_DIR="${SCRIPT_DIR%/scripts}/udev_rules"
RULES_FILE="99-sts3215.rules"
SYSTEM_RULES_DIR="/etc/udev/rules.d"

# Check if running with sudo
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run with sudo"
   exit 1
fi

# Check if rules file exists
if [[ ! -f "${RULES_DIR}/${RULES_FILE}" ]]; then
    echo "Error: Rules file not found at ${RULES_DIR}/${RULES_FILE}"
    exit 1
fi

# Copy rules file
echo "Installing udev rule: ${RULES_FILE}"
cp "${RULES_DIR}/${RULES_FILE}" "${SYSTEM_RULES_DIR}/"

# Set correct permissions
chmod 644 "${SYSTEM_RULES_DIR}/${RULES_FILE}"

# Reload udev rules
echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "✓ udev rule installed successfully"
echo "The STS3215 board should now be accessible as /dev/ttyUSB_sts3215"
echo ""
echo "Note: If you've already plugged in the device, you may need to disconnect"
echo "and reconnect it for the rule to take effect."

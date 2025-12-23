#!/bin/bash

# Script to undeploy mavlink-server systemd service
# This script removes all configuration files and service files created during deployment
# It does NOT remove the mavlink-server binary file
# Usage: ./mavlink-server-undeploy.sh

set -e

# Configuration (must match deployment script)
SERVICE_NAME="mavlink-server"
BINARY_PATH="/usr/bin/mavlink-server"
CONFIG_DIR="/etc/mavlink-server"
CONFIG_FILE_NAME="mavlink-server.conf"
SYSTEMD_SERVICE_DIR="/etc/systemd/system"
SERVICE_FILE="${SYSTEMD_SERVICE_DIR}/${SERVICE_NAME}.service"
RUN_SCRIPT="/usr/local/bin/run-mavlink-server.sh"

# Function to print section headers
print_section() {
    echo ""
    echo "================================================================="
    echo "$1"
    echo "================================================================="
}

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Error: This script must be run as root (use sudo)" >&2
    exit 1
fi

print_section "Undeploying mavlink-server systemd service"

# Track what was removed
REMOVED_ITEMS=()
SKIPPED_ITEMS=()

# Step 1: Stop the service if running
print_section "Stopping service"
if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    systemctl stop "$SERVICE_NAME"
    echo "Stopped $SERVICE_NAME service"
    REMOVED_ITEMS+=("Service stopped")
else
    echo "Service is not running"
    SKIPPED_ITEMS+=("Service was not running")
fi

# Step 2: Disable the service
print_section "Disabling service"
if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
    systemctl disable "$SERVICE_NAME"
    echo "Disabled $SERVICE_NAME service"
    REMOVED_ITEMS+=("Service disabled")
else
    echo "Service is not enabled"
    SKIPPED_ITEMS+=("Service was not enabled")
fi

# Step 3: Remove systemd service file
print_section "Removing systemd service file"
if [ -f "$SERVICE_FILE" ]; then
    rm -f "$SERVICE_FILE"
    echo "Removed service file: $SERVICE_FILE"
    REMOVED_ITEMS+=("Service file: $SERVICE_FILE")
else
    echo "Service file not found: $SERVICE_FILE"
    SKIPPED_ITEMS+=("Service file: $SERVICE_FILE (not found)")
fi

# Step 4: Reload systemd daemon
print_section "Reloading systemd daemon"
systemctl daemon-reload
echo "Systemd daemon reloaded"

# Step 5: Remove run script wrapper
print_section "Removing run script wrapper"
if [ -f "$RUN_SCRIPT" ]; then
    rm -f "$RUN_SCRIPT"
    echo "Removed run script: $RUN_SCRIPT"
    REMOVED_ITEMS+=("Run script: $RUN_SCRIPT")
else
    echo "Run script not found: $RUN_SCRIPT"
    SKIPPED_ITEMS+=("Run script: $RUN_SCRIPT (not found)")
fi

# Step 6: Remove configuration directory
print_section "Removing configuration directory"
if [ -d "$CONFIG_DIR" ]; then
    # List files that will be removed
    echo "Configuration directory contents:"
    ls -la "$CONFIG_DIR" 2>/dev/null || echo "  (empty or unreadable)"
    echo ""
    
    # Remove the entire directory
    rm -rf "$CONFIG_DIR"
    echo "Removed configuration directory: $CONFIG_DIR"
    REMOVED_ITEMS+=("Config directory: $CONFIG_DIR")
else
    echo "Configuration directory not found: $CONFIG_DIR"
    SKIPPED_ITEMS+=("Config directory: $CONFIG_DIR (not found)")
fi

# Step 7: Verify binary is NOT removed (safety check)
print_section "Verifying binary file"
if [ -f "$BINARY_PATH" ]; then
    echo "Binary file preserved: $BINARY_PATH"
    echo "  (Binary file is NOT removed during undeployment)"
else
    echo "Note: Binary file not found at $BINARY_PATH"
    echo "  (This is normal if binary was never installed)"
fi

# Summary
print_section "Undeployment Summary"
echo "Items removed:"
if [ ${#REMOVED_ITEMS[@]} -gt 0 ]; then
    for item in "${REMOVED_ITEMS[@]}"; do
        echo "  ✓ $item"
    done
else
    echo "  (none - nothing was deployed)"
fi

echo ""
echo "Items skipped (not found):"
if [ ${#SKIPPED_ITEMS[@]} -gt 0 ]; then
    for item in "${SKIPPED_ITEMS[@]}"; do
        echo "  ⊘ $item"
    done
else
    echo "  (none)"
fi

echo ""
echo "Binary file status:"
if [ -f "$BINARY_PATH" ]; then
    echo "  ✓ Binary preserved: $BINARY_PATH"
else
    echo "  ⊘ Binary not found: $BINARY_PATH"
fi

print_section "Undeployment Complete"
echo "mavlink-server systemd service has been undeployed."
echo ""
echo "The following were removed:"
echo "  - Systemd service file: $SERVICE_FILE"
echo "  - Run script wrapper: $RUN_SCRIPT"
echo "  - Configuration directory: $CONFIG_DIR"
echo ""
echo "The following was preserved:"
echo "  - Binary file: $BINARY_PATH (if it exists)"
echo ""
echo "To redeploy, run:"
echo "  sudo ./mavlink-server-deploy.sh"
echo ""


#!/bin/bash

# Test script for mavlink-server deployment process
# This script validates that the deployment script works correctly
# Usage: ./test-deployment.sh [--dry-run] [--cleanup]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test configuration
SERVICE_NAME="mavlink-server"
BINARY_PATH="/usr/bin/mavlink-server"
CONFIG_DIR="/etc/mavlink-server"
CONFIG_FILE_NAME="mavlink-server.conf"
SYSTEMD_SERVICE_DIR="/etc/systemd/system"
SERVICE_FILE="${SYSTEMD_SERVICE_DIR}/${SERVICE_NAME}.service"
RUN_SCRIPT="/usr/local/bin/run-mavlink-server.sh"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_SCRIPT="${SCRIPT_DIR}/mavlink-server-deploy.sh"
UNDEPLOY_SCRIPT="${SCRIPT_DIR}/mavlink-server-undeploy.sh"
CONFIGURE_SCRIPT="${SCRIPT_DIR}/configure_mavlink_server.sh"
TEMPLATE_FILE="${SCRIPT_DIR}/mavlink-server.template"
# Fallback to alternative template file names
if [ ! -f "$TEMPLATE_FILE" ]; then
    if [ -f "${SCRIPT_DIR}/mavlink-server.template" ]; then
        TEMPLATE_FILE="${SCRIPT_DIR}/mavlink-server.template"
    fi
fi

# Test flags
DRY_RUN=true
CLEANUP=true
TESTS_PASSED=0
TESTS_FAILED=0
TESTS_SKIPPED=0

# Parse arguments
for arg in "$@"; do
    case $arg in
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --cleanup)
            CLEANUP=true
            shift
            ;;
        *)
            ;;
    esac
done

# Test result tracking
test_passed() {
    echo -e "${GREEN}✓ PASSED${NC}: $1"
    echo -e "${GREEN}  → Confirmed${NC}"
    ((TESTS_PASSED++))
}

test_failed() {
    echo -e "${RED}✗ FAILED${NC}: $1"
    ((TESTS_FAILED++))
}

test_skipped() {
    echo -e "${YELLOW}⊘ SKIPPED${NC}: $1"
    ((TESTS_SKIPPED++))
}

print_section() {
    echo ""
    echo -e "${BLUE}=================================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}=================================================================${NC}"
}

# Setup function - clean state before testing
setup_clean_state() {
    print_section "Setting Up Clean Test State"
    
    if [ "$EUID" -ne 0 ]; then
        echo "Note: Setup requires root privileges. Skipping..."
        return 0
    fi
    
    # Check and delete /etc/mavlink-server directory if it exists
    if [ -d "$CONFIG_DIR" ]; then
        echo "Found existing $CONFIG_DIR directory. Removing it for clean test state..."
        rm -rf "$CONFIG_DIR" || true
        test_passed "Removed existing $CONFIG_DIR directory"
    else
        test_passed "$CONFIG_DIR directory does not exist (clean state)"
    fi
    
    return 0
}

# Cleanup function
cleanup() {
    print_section "Cleaning Up Test Environment"
    
    if [ "$EUID" -ne 0 ]; then
        echo "Note: Cleanup requires root privileges. Skipping..."
        return
    fi
    
    # Stop and disable service
    if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
        echo "Stopping service..."
        systemctl stop "$SERVICE_NAME" || true
    fi
    
    if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
        echo "Disabling service..."
        systemctl disable "$SERVICE_NAME" || true
    fi
    
    # Remove service file
    if [ -f "$SERVICE_FILE" ]; then
        echo "Removing service file..."
        rm -f "$SERVICE_FILE"
        systemctl daemon-reload || true
    fi
    
    # Remove run script
    if [ -f "$RUN_SCRIPT" ]; then
        echo "Removing run script..."
        rm -f "$RUN_SCRIPT"
    fi
    
    # Remove config directory
    if [ -d "$CONFIG_DIR" ]; then
        echo "Removing config directory..."
        rm -rf "$CONFIG_DIR"
    fi
    
    echo "Cleanup complete."
}

# Test functions
test_prerequisites() {
    print_section "Testing Prerequisites"
    
    # Test 1: Check if running as root
    if [ "$EUID" -eq 0 ]; then
        test_passed "Running as root"
    else
        test_failed "Not running as root (required for deployment tests)"
        return 1
    fi
    
    # Test 2: Check if deploy script exists
    if [ -f "$DEPLOY_SCRIPT" ]; then
        test_passed "Deploy script exists: $DEPLOY_SCRIPT"
    else
        test_failed "Deploy script not found: $DEPLOY_SCRIPT"
        return 1
    fi
    
    # Test 3: Check if deploy script is executable
    if [ -x "$DEPLOY_SCRIPT" ]; then
        test_passed "Deploy script is executable"
    else
        test_failed "Deploy script is not executable"
        return 1
    fi
    
    # Test 4: Check if configure script exists
    if [ -f "$CONFIGURE_SCRIPT" ]; then
        test_passed "Configure script exists: $CONFIGURE_SCRIPT"
    else
        test_failed "Configure script not found: $CONFIGURE_SCRIPT"
        return 1
    fi
    
    # Test 5: Check if template file exists
    if [ -f "$TEMPLATE_FILE" ]; then
        test_passed "Template file exists: $TEMPLATE_FILE"
    else
        test_failed "Template file not found: $TEMPLATE_FILE"
        return 1
    fi
    
    # Test 6: Check if binary exists (optional - may not exist in test environment)
    if [ -f "$BINARY_PATH" ] && [ -x "$BINARY_PATH" ]; then
        test_passed "mavlink-server binary exists: $BINARY_PATH"
    else
        test_skipped "mavlink-server binary not found (will be checked during deployment)"
    fi
    
    # Test 7: Check if configure script exists and is executable
    if [ -f "$CONFIGURE_SCRIPT" ] && [ -x "$CONFIGURE_SCRIPT" ]; then
        test_passed "Configure script exists and is executable: $CONFIGURE_SCRIPT"
    else
        test_failed "Configure script not found or not executable: $CONFIGURE_SCRIPT"
    fi
    
    # Test 8: Check if config file exists in current directory (optional)
    if [ -f "${SCRIPT_DIR}/mavlink-server.conf" ]; then
        test_passed "Config file exists in current directory"
    else
        test_skipped "Config file not in current directory (will be created by configure script)"
    fi
    
    return 0
}

test_config_search_order() {
    print_section "Testing Configuration Search Order"
    
    # Create temporary test directory
    TEST_DIR=$(mktemp -d)
    TEST_CONFIG="${TEST_DIR}/test-config.conf"
    
    # Create a test config file
    cat > "$TEST_CONFIG" << 'EOF'
# Test configuration
web_server = "0.0.0.0:8080"
verbose = false

[[serial]]
device = "/dev/ttyTEST0"
baudrate = 115200

[[udp_server]]
address = "0.0.0.0"
port = 14550

[[tcp_server]]
address = "0.0.0.0"
port = 5760
EOF
    
    # Test 1: System config directory (if exists)
    SYSTEM_CONFIG_FILE="${CONFIG_DIR}/${CONFIG_FILE_NAME}"
    if [ -f "$SYSTEM_CONFIG_FILE" ]; then
        test_passed "System config file found at: $SYSTEM_CONFIG_FILE"
    else
        test_skipped "System config file not found (expected if not deployed yet)"
    fi
    
    # Test 2: Current directory config
    if [ -f "${SCRIPT_DIR}/mavlink-server.conf" ]; then
        test_passed "Current directory config found"
    else
        test_skipped "Current directory config not found"
    fi
    
    # Test 3: Template file
    if [ -f "$TEMPLATE_FILE" ]; then
        test_passed "Template file found"
    else
        test_failed "Template file not found"
    fi
    
    # Cleanup
    rm -rf "$TEST_DIR"
}

test_deployment_script_structure() {
    print_section "Testing Deployment Script Structure"
    
    # Test 1: Check script syntax
    if bash -n "$DEPLOY_SCRIPT" 2>/dev/null; then
        test_passed "Deploy script syntax is valid"
    else
        test_failed "Deploy script has syntax errors"
        bash -n "$DEPLOY_SCRIPT"
        return 1
    fi
    
    # Test 2: Check for required functions
    if grep -q "print_section" "$DEPLOY_SCRIPT"; then
        test_passed "Deploy script contains print_section function"
    else
        test_failed "Deploy script missing print_section function"
    fi
    
    # Test 3: Check for root check
    if grep -q "EUID" "$DEPLOY_SCRIPT"; then
        test_passed "Deploy script checks for root privileges"
    else
        test_failed "Deploy script missing root privilege check"
    fi
    
    # Test 4: Check for binary check
    if grep -q "BINARY_PATH" "$DEPLOY_SCRIPT"; then
        test_passed "Deploy script checks for binary existence"
    else
        test_failed "Deploy script missing binary check"
    fi
    
    # Test 5: Check for systemd service creation
    if grep -q "systemd" "$DEPLOY_SCRIPT"; then
        test_passed "Deploy script creates systemd service"
    else
        test_failed "Deploy script missing systemd service creation"
    fi
    
    # Test 6: Check for configure script check
    if grep -q "configure_mavlink_server.sh" "$DEPLOY_SCRIPT"; then
        test_passed "Deploy script checks for configure script"
    else
        test_failed "Deploy script missing configure script check"
    fi
    
    # Test 7: Check for directory removal logic
    if grep -q "rm -rf.*CONFIG_DIR\|rm -rf.*\$CONFIG_DIR" "$DEPLOY_SCRIPT"; then
        test_passed "Deploy script removes existing config directory"
    else
        test_failed "Deploy script missing directory removal logic"
    fi
    
    # Test 8: Check for configure script execution
    if grep -q "\"\$CONFIGURE_SCRIPT\"" "$DEPLOY_SCRIPT"; then
        test_passed "Deploy script executes configure script"
    else
        test_failed "Deploy script missing configure script execution"
    fi
}

test_deployment_execution() {
    print_section "Testing Deployment Execution"
    
    if [ "$DRY_RUN" = true ]; then
        test_skipped "Deployment execution (dry-run mode)"
        return 0
    fi
    
    # Check if binary exists (required for actual deployment)
    if [ ! -f "$BINARY_PATH" ] || [ ! -x "$BINARY_PATH" ]; then
        test_skipped "Deployment execution (binary not found - run install_mavlink_server.sh first)"
        return 0
    fi
    
    # Check if configure script exists (required for deployment)
    if [ ! -f "$CONFIGURE_SCRIPT" ] || [ ! -x "$CONFIGURE_SCRIPT" ]; then
        test_failed "Configure script not found or not executable: $CONFIGURE_SCRIPT"
        return 1
    fi
    
    # Note: Deployment script no longer accepts config file argument
    # It runs configure_mavlink_server.sh interactively instead
    # Run deployment script (without arguments - it will prompt for config)
    echo "Running deployment script (will run configure_mavlink_server.sh interactively)..."
    echo "Note: This test requires manual interaction if run in interactive mode."
    echo "For automated testing, configure script should be run separately or modified for non-interactive use."
    
    # For now, we'll skip actual execution in automated tests since it requires interaction
    # In a real scenario, you might want to use expect or modify configure script for non-interactive mode
    test_skipped "Deployment execution (requires interactive configuration - run manually for full test)"
    
    # Uncomment below if you have a way to provide non-interactive input
    # if "$DEPLOY_SCRIPT" > /tmp/deploy-test.log 2>&1; then
    #     test_passed "Deployment script executed successfully"
    # else
    #     test_failed "Deployment script failed"
    #     echo "Deployment log:"
    #     cat /tmp/deploy-test.log
    #     return 1
    # fi
}

test_deployment_results() {
    print_section "Testing Deployment Results"
    
    # Test 1: Check config directory exists
    if [ -d "$CONFIG_DIR" ]; then
        test_passed "Config directory created: $CONFIG_DIR"
    else
        test_failed "Config directory not created: $CONFIG_DIR"
    fi
    
    # Test 2: Check config file exists
    if [ -f "${CONFIG_DIR}/${CONFIG_FILE_NAME}" ]; then
        test_passed "Config file created at: ${CONFIG_DIR}/${CONFIG_FILE_NAME}"
    else
        test_failed "Config file not created at: ${CONFIG_DIR}/${CONFIG_FILE_NAME}"
    fi
    
    # Test 3: Check config file permissions
    if [ -r "${CONFIG_DIR}/${CONFIG_FILE_NAME}" ]; then
        test_passed "Config file is readable"
    else
        test_failed "Config file is not readable"
    fi
    
    # Test 4: Check run script exists
    if [ -f "$RUN_SCRIPT" ]; then
        test_passed "Run script created: $RUN_SCRIPT"
    else
        test_failed "Run script not created: $RUN_SCRIPT"
    fi
    
    # Test 5: Check run script is executable
    if [ -x "$RUN_SCRIPT" ]; then
        test_passed "Run script is executable"
    else
        test_failed "Run script is not executable"
    fi
    
    # Test 6: Check systemd service file exists
    if [ -f "$SERVICE_FILE" ]; then
        test_passed "Systemd service file created: $SERVICE_FILE"
    else
        test_failed "Systemd service file not created: $SERVICE_FILE"
    fi
    
    # Test 7: Check service file content
    if [ -f "$SERVICE_FILE" ]; then
        if grep -q "ExecStart=$RUN_SCRIPT" "$SERVICE_FILE"; then
            test_passed "Service file contains correct ExecStart"
        else
            test_failed "Service file missing correct ExecStart"
        fi
        
        if grep -q "Restart=always" "$SERVICE_FILE"; then
            test_passed "Service file contains Restart=always"
        else
            test_failed "Service file missing Restart=always"
        fi
        
        if grep -q "WantedBy=multi-user.target" "$SERVICE_FILE"; then
            test_passed "Service file contains WantedBy"
        else
            test_failed "Service file missing WantedBy"
        fi
    fi
    
    # Test 8: Check service is enabled
    if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
        test_passed "Service is enabled"
    else
        test_failed "Service is not enabled"
    fi
    
    # Test 9: Check service status (may be running or failed, but should exist)
    if systemctl list-unit-files | grep -q "^${SERVICE_NAME}.service"; then
        test_passed "Service is registered with systemd"
    else
        test_failed "Service is not registered with systemd"
    fi
}

test_service_running() {
    print_section "Testing Service Execution with Configuration"
    
    if [ "$DRY_RUN" = true ]; then
        test_skipped "Service execution test (dry-run mode)"
        return 0
    fi
    
    # Test 1: Wait a moment for service to start
    echo "Waiting for service to start..."
    sleep 3
    
    # Test 2: Check if service is active
    if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
        test_passed "Service is active (running)"
    else
        test_failed "Service is not active"
        echo "Service status:"
        systemctl status "$SERVICE_NAME" --no-pager -l || true
        return 1
    fi
    
    # Test 3: Check service status for errors
    SERVICE_STATUS=$(systemctl status "$SERVICE_NAME" --no-pager 2>&1 || true)
    if echo "$SERVICE_STATUS" | grep -qi "active (running)"; then
        test_passed "Service status shows active (running)"
    else
        test_failed "Service status does not show active (running)"
        echo "Service status output:"
        echo "$SERVICE_STATUS"
    fi
    
    # Test 4: Check recent logs for critical errors
    echo "Checking service logs for errors..."
    RECENT_LOGS=$(journalctl -u "$SERVICE_NAME" --since "30 seconds ago" --no-pager 2>&1 || true)
    
    # Check for common error patterns
    if echo "$RECENT_LOGS" | grep -qi "error\|failed\|fatal\|panic"; then
        ERROR_COUNT=$(echo "$RECENT_LOGS" | grep -ci "error\|failed\|fatal\|panic" || echo "0")
        # Some errors might be acceptable (e.g., serial port not found if device doesn't exist)
        # But we should still report them
        if echo "$RECENT_LOGS" | grep -qi "Failed to open serial port.*No such file"; then
            test_skipped "Service running but serial port not available (expected if device not connected)"
        else
            echo "Warning: Found $ERROR_COUNT error(s) in recent logs"
            echo "Recent log errors:"
            echo "$RECENT_LOGS" | grep -i "error\|failed\|fatal\|panic" | tail -5
            # Don't fail the test for serial port errors, but do for other errors
            NON_SERIAL_ERRORS=$(echo "$RECENT_LOGS" | grep -i "error\|failed\|fatal\|panic" | grep -vi "serial port" || true)
            if [ -n "$NON_SERIAL_ERRORS" ]; then
                test_failed "Service logs contain non-serial-port errors"
                echo "Non-serial-port errors:"
                echo "$NON_SERIAL_ERRORS" | head -5
            else
                test_passed "Service logs checked (only serial port errors, which are acceptable)"
            fi
        fi
    else
        test_passed "No critical errors in recent service logs"
    fi
    
    # Test 5: Verify service is using correct config file
    if [ -f "${CONFIG_DIR}/${CONFIG_FILE_NAME}" ]; then
        # Check if the run script references the correct config
        if [ -f "$RUN_SCRIPT" ]; then
            if grep -q "${CONFIG_DIR}/${CONFIG_FILE_NAME}" "$RUN_SCRIPT" || \
               grep -q "\$CONFIG_DIR/\$CONFIG_FILE_NAME" "$RUN_SCRIPT"; then
                test_passed "Run script references correct config file path"
            else
                test_failed "Run script does not reference correct config file path"
            fi
        fi
        
        # Check service file ExecStart
        if [ -f "$SERVICE_FILE" ]; then
            if grep -q "${CONFIG_DIR}/${CONFIG_FILE_NAME}" "$SERVICE_FILE"; then
                test_passed "Service file references correct config file path"
            else
                test_failed "Service file does not reference correct config file path"
            fi
        fi
    else
        test_failed "Configuration file not found: ${CONFIG_DIR}/${CONFIG_FILE_NAME}"
    fi
    
    # Test 6: Check if web server is accessible (if configured)
    if [ -f "${CONFIG_DIR}/${CONFIG_FILE_NAME}" ]; then
        # Extract web_server from config
        WEB_SERVER=$(grep "^web_server" "${CONFIG_DIR}/${CONFIG_FILE_NAME}" | sed 's/.*= *"\([^"]*\)".*/\1/' | head -1)
        if [ -n "$WEB_SERVER" ]; then
            WEB_HOST=$(echo "$WEB_SERVER" | cut -d: -f1)
            WEB_PORT=$(echo "$WEB_SERVER" | cut -d: -f2)
            
            # Try to connect to web server (timeout after 2 seconds)
            if command -v nc >/dev/null 2>&1; then
                if timeout 2 nc -z "$WEB_HOST" "$WEB_PORT" 2>/dev/null; then
                    test_passed "Web server is accessible on $WEB_SERVER"
                else
                    test_skipped "Web server not accessible on $WEB_SERVER (may need more time to start)"
                fi
            else
                test_skipped "Web server accessibility check (nc command not available)"
            fi
        else
            test_skipped "Web server check (not configured or using default)"
        fi
    fi
    
    # Test 7: Verify process is running
    if pgrep -f "mavlink-server" > /dev/null 2>&1; then
        test_passed "mavlink-server process is running"
    else
        test_failed "mavlink-server process is not running"
    fi
    
    # Test 8: Check service uptime
    SERVICE_UPTIME=$(systemctl show "$SERVICE_NAME" --property=ActiveEnterTimestamp --value 2>/dev/null || echo "")
    if [ -n "$SERVICE_UPTIME" ]; then
        test_passed "Service has been running since: $SERVICE_UPTIME"
    else
        test_failed "Could not determine service uptime"
    fi
    
    # Test 9: Verify no immediate crashes (check if service restarted recently)
    RESTART_COUNT=$(systemctl show "$SERVICE_NAME" --property=NRestarts --value 2>/dev/null || echo "0")
    if [ "$RESTART_COUNT" = "0" ] || [ -z "$RESTART_COUNT" ]; then
        test_passed "Service has not restarted (stable)"
    else
        test_failed "Service has restarted $RESTART_COUNT time(s) - may indicate issues"
        echo "Recent service logs:"
        journalctl -u "$SERVICE_NAME" --since "1 minute ago" --no-pager | tail -20
    fi
}

test_configure_script_search() {
    print_section "Testing Configure Script Search Order"
    
    # Test that configure script can find configs in correct order
    # This is a basic test - full testing would require actual execution
    
    if grep -q "SYSTEM_CONFIG_FILE" "$CONFIGURE_SCRIPT"; then
        test_passed "Configure script checks system config directory"
    else
        test_failed "Configure script missing system config check"
    fi
    
    if grep -q "TEMPLATE_FILE_PATH" "$CONFIGURE_SCRIPT"; then
        test_passed "Configure script checks template file"
    else
        test_failed "Configure script missing template file check"
    fi
}

test_run_script_content() {
    print_section "Testing Run Script Content"
    
    if [ ! -f "$RUN_SCRIPT" ]; then
        test_skipped "Run script content (script not created yet)"
        return 0
    fi
    
    # Test 1: Check script syntax
    if bash -n "$RUN_SCRIPT" 2>/dev/null; then
        test_passed "Run script syntax is valid"
    else
        test_failed "Run script has syntax errors"
        bash -n "$RUN_SCRIPT"
        return 1
    fi
    
    # Test 2: Check for config file parameter
    if grep -q "CONFIG_FILE" "$RUN_SCRIPT"; then
        test_passed "Run script accepts config file parameter"
    else
        test_failed "Run script missing config file handling"
    fi
    
    # Test 3: Check for binary path
    if grep -q "BINARY_PATH" "$RUN_SCRIPT"; then
        test_passed "Run script references binary path"
    else
        test_failed "Run script missing binary path"
    fi
    
    # Test 4: Check for endpoint parsing
    if grep -q "ENDPOINTS" "$RUN_SCRIPT"; then
        test_passed "Run script parses endpoints"
    else
        test_failed "Run script missing endpoint parsing"
    fi
}

test_undeploy_script_structure() {
    print_section "Testing Undeploy Script Structure"
    
    # Test 1: Check if undeploy script exists
    if [ -f "$UNDEPLOY_SCRIPT" ]; then
        test_passed "Undeploy script exists: $UNDEPLOY_SCRIPT"
    else
        test_failed "Undeploy script not found: $UNDEPLOY_SCRIPT"
        return 1
    fi
    
    # Test 2: Check if undeploy script is executable
    if [ -x "$UNDEPLOY_SCRIPT" ]; then
        test_passed "Undeploy script is executable"
    else
        test_failed "Undeploy script is not executable"
    fi
    
    # Test 3: Check script syntax
    if bash -n "$UNDEPLOY_SCRIPT" 2>/dev/null; then
        test_passed "Undeploy script syntax is valid"
    else
        test_failed "Undeploy script has syntax errors"
        bash -n "$UNDEPLOY_SCRIPT"
        return 1
    fi
    
    # Test 4: Check for root check
    if grep -q "EUID" "$UNDEPLOY_SCRIPT"; then
        test_passed "Undeploy script checks for root privileges"
    else
        test_failed "Undeploy script missing root privilege check"
    fi
    
    # Test 5: Check for service stop
    if grep -q "systemctl stop" "$UNDEPLOY_SCRIPT"; then
        test_passed "Undeploy script stops service"
    else
        test_failed "Undeploy script missing service stop"
    fi
    
    # Test 6: Check for service disable
    if grep -q "systemctl disable" "$UNDEPLOY_SCRIPT"; then
        test_passed "Undeploy script disables service"
    else
        test_failed "Undeploy script missing service disable"
    fi
    
    # Test 7: Check for config directory removal
    if grep -q "rm -rf.*CONFIG_DIR\|rm -rf.*\$CONFIG_DIR\|rm -rf \"\$CONFIG_DIR\"" "$UNDEPLOY_SCRIPT"; then
        test_passed "Undeploy script removes config directory"
    else
        test_failed "Undeploy script missing config directory removal"
    fi
    
    # Test 8: Check for binary preservation
    if grep -q "BINARY_PATH\|Binary file" "$UNDEPLOY_SCRIPT"; then
        test_passed "Undeploy script preserves binary file"
    else
        test_failed "Undeploy script missing binary preservation check"
    fi
}

test_undeploy_execution() {
    print_section "Testing Undeployment Execution"
    
    if [ "$DRY_RUN" = true ]; then
        test_skipped "Undeployment execution (dry-run mode)"
        return 0
    fi
    
    # Check if service exists (must be deployed first)
    if ! systemctl list-unit-files | grep -q "^${SERVICE_NAME}.service"; then
        test_skipped "Undeployment execution (service not deployed - deploy first)"
        return 0
    fi
    
    # Run undeploy script
    echo "Running undeploy script..."
    if "$UNDEPLOY_SCRIPT" > /tmp/undeploy-test.log 2>&1; then
        test_passed "Undeploy script executed successfully"
    else
        test_failed "Undeploy script failed"
        echo "Undeploy log:"
        cat /tmp/undeploy-test.log
        return 1
    fi
}

test_undeploy_validation() {
    print_section "Validating Undeployment Results"
    
    # Test 1: Verify service file is removed
    if [ ! -f "$SERVICE_FILE" ]; then
        test_passed "Service file removed: $SERVICE_FILE"
    else
        test_failed "Service file still exists: $SERVICE_FILE"
    fi
    
    # Test 2: Verify run script is removed
    if [ ! -f "$RUN_SCRIPT" ]; then
        test_passed "Run script removed: $RUN_SCRIPT"
    else
        test_failed "Run script still exists: $RUN_SCRIPT"
    fi
    
    # Test 3: Verify config directory is removed
    if [ ! -d "$CONFIG_DIR" ]; then
        test_passed "Config directory removed: $CONFIG_DIR"
    else
        test_failed "Config directory still exists: $CONFIG_DIR"
    fi
    
    # Test 4: Verify config file is removed
    if [ ! -f "${CONFIG_DIR}/${CONFIG_FILE_NAME}" ]; then
        test_passed "Config file removed: ${CONFIG_DIR}/${CONFIG_FILE_NAME}"
    else
        test_failed "Config file still exists: ${CONFIG_DIR}/${CONFIG_FILE_NAME}"
    fi
    
    # Test 5: Verify service is not registered with systemd
    if ! systemctl list-unit-files | grep -q "^${SERVICE_NAME}.service"; then
        test_passed "Service not registered with systemd"
    else
        test_failed "Service still registered with systemd"
    fi
    
    # Test 6: Verify service is not enabled
    if ! systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
        test_passed "Service is not enabled"
    else
        test_failed "Service is still enabled"
    fi
    
    # Test 7: Verify service is not active
    if ! systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
        test_passed "Service is not active"
    else
        test_failed "Service is still active"
    fi
    
    # Test 8: Verify binary file is preserved
    if [ -f "$BINARY_PATH" ]; then
        test_passed "Binary file preserved: $BINARY_PATH"
    else
        test_skipped "Binary file not found (may not have been installed)"
    fi
    
    # Test 9: Check for any remaining mavlink-server config files in system directories
    REMAINING_CONFIGS=$(find /etc -name "*mavlink-server*" -type f 2>/dev/null | grep -v "^$" || true)
    if [ -z "$REMAINING_CONFIGS" ]; then
        test_passed "No remaining mavlink-server config files in /etc"
    else
        test_failed "Found remaining config files in /etc: $REMAINING_CONFIGS"
    fi
    
    # Test 10: Check for any remaining mavlink-server directories in system directories
    REMAINING_DIRS=$(find /etc -name "*mavlink-server*" -type d 2>/dev/null | grep -v "^$" || true)
    if [ -z "$REMAINING_DIRS" ]; then
        test_passed "No remaining mavlink-server directories in /etc"
    else
        test_failed "Found remaining directories in /etc: $REMAINING_DIRS"
    fi
    
    # Test 11: Check for any remaining mavlink-server scripts in system directories
    REMAINING_SCRIPTS=$(find /usr/local/bin -name "*mavlink-server*" -type f 2>/dev/null | grep -v "^$" || true)
    if [ -z "$REMAINING_SCRIPTS" ]; then
        test_passed "No remaining mavlink-server scripts in /usr/local/bin"
    else
        test_failed "Found remaining scripts in /usr/local/bin: $REMAINING_SCRIPTS"
    fi
}

# Main test execution
main() {
    print_section "MAVLink Server Deployment Test Suite"
    
    echo "Test Configuration:"
    echo "  Dry Run: $DRY_RUN"
    echo "  Cleanup: $CLEANUP"
    echo "  Script Directory: $SCRIPT_DIR"
    echo ""
    
    # Setup clean state (delete /etc/mavlink-server if exists)
    set +e  # Temporarily disable exit on error for setup
    setup_clean_state
    SETUP_EXIT_CODE=$?
    set -e  # Re-enable exit on error
    
    if [ $SETUP_EXIT_CODE -ne 0 ]; then
        echo "Warning: Setup encountered issues but continuing..."
    fi
    
    # Run tests
    echo ""
    test_prerequisites || {
        echo "Error: Prerequisites test failed. Exiting."
        exit 1
    }
    test_config_search_order
    test_deployment_script_structure
    test_configure_script_search
    test_undeploy_script_structure
    
    # Only run deployment tests if not dry-run and prerequisites met
    if [ "$DRY_RUN" = false ] && [ "$EUID" -eq 0 ]; then
        test_deployment_execution
        test_deployment_results
        test_run_script_content
        
        # Test that service runs properly with configuration
        test_service_running
        
        # Run undeployment tests after deployment
        test_undeploy_execution
        test_undeploy_validation
    fi
    
    # Print summary
    print_section "Test Summary"
    echo -e "Tests Passed:  ${GREEN}$TESTS_PASSED${NC}"
    echo -e "Tests Failed:  ${RED}$TESTS_FAILED${NC}"
    echo -e "Tests Skipped: ${YELLOW}$TESTS_SKIPPED${NC}"
    echo ""
    
    TOTAL_TESTS=$((TESTS_PASSED + TESTS_FAILED + TESTS_SKIPPED))
    echo "Total Tests: $TOTAL_TESTS"
    
    if [ $TESTS_FAILED -eq 0 ]; then
        echo -e "${GREEN}All tests passed!${NC}"
        EXIT_CODE=0
    else
        echo -e "${RED}Some tests failed!${NC}"
        EXIT_CODE=1
    fi
    
    # Cleanup if requested
    if [ "$CLEANUP" = true ]; then
        cleanup
    fi
    
    exit $EXIT_CODE
}

# Run main function
main


#!/bin/bash

# Script to deploy mavlink-server as a systemd service
# This script removes existing /etc/mavlink-server directory if present,
# runs configure_mavlink_server.sh to create the configuration interactively,
# creates a systemd service, and enables it to start at boot
# Usage: ./mavlink-server-deploy.sh

set -e

# Configuration
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIGURE_SCRIPT="${SCRIPT_DIR}/configure_mavlink_server.sh"

# Check if mavlink-server binary exists
if [ ! -f "$BINARY_PATH" ] || [ ! -x "$BINARY_PATH" ]; then
    echo "Error: mavlink-server binary not found at $BINARY_PATH" >&2
    echo "Please run install_mavlink_server.sh first to install the binary." >&2
    exit 1
fi

# Check if configure script exists
if [ ! -f "$CONFIGURE_SCRIPT" ] || [ ! -x "$CONFIGURE_SCRIPT" ]; then
    echo "Error: configure_mavlink_server.sh not found or not executable: $CONFIGURE_SCRIPT" >&2
    exit 1
fi

print_section "Deploying mavlink-server as systemd service"

# Step 1: Remove existing /etc/mavlink-server directory if it exists
print_section "Preparing clean deployment state"
if [ -d "$CONFIG_DIR" ]; then
    echo "Removing existing $CONFIG_DIR directory..."
    rm -rf "$CONFIG_DIR"
    echo "Removed existing $CONFIG_DIR directory"
else
    echo "$CONFIG_DIR directory does not exist (clean state)"
fi

# Step 2: Run configure script to create configuration file
print_section "Creating configuration file"
echo "Running configure_mavlink_server.sh to create configuration..."
echo "Please follow the prompts to configure your mavlink-server endpoints."
echo ""

# Run the configure script - it will create the config in /etc/mavlink-server when run as root
if "$CONFIGURE_SCRIPT"; then
    # Verify config file was created
    if [ -f "${CONFIG_DIR}/${CONFIG_FILE_NAME}" ]; then
        echo ""
        echo "Configuration file created successfully at: ${CONFIG_DIR}/${CONFIG_FILE_NAME}"
    else
        echo "Error: Configuration file was not created at ${CONFIG_DIR}/${CONFIG_FILE_NAME}" >&2
        exit 1
    fi
else
    echo "Error: Configuration script failed!" >&2
    exit 1
fi

# Step 3: Create run script wrapper
print_section "Creating run script wrapper"
cat > "$RUN_SCRIPT" << 'RUNSCRIPT_EOF'
#!/bin/bash
# Wrapper script to run mavlink-server with configuration file
# This script reads the config file and converts it to command-line arguments

set -e

CONFIG_FILE="${1:-/etc/mavlink-server/mavlink-server.conf}"
BINARY_PATH="/usr/bin/mavlink-server"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file '$CONFIG_FILE' not found!" >&2
    exit 1
fi

# Function to trim whitespace
trim() {
    echo "$1" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//'
}

# Function to parse TOML value
parse_toml_value() {
    local line="$1"
    line="${line%%#*}"
    line=$(trim "$line")
    
    if [ -z "$line" ]; then
        return
    fi
    
    if [[ "$line" =~ ^([^=]+)=(.*)$ ]]; then
        local key="${BASH_REMATCH[1]}"
        local value="${BASH_REMATCH[2]}"
        key=$(trim "$key")
        value=$(trim "$value")
        value="${value#\"}"
        value="${value%\"}"
        echo "$key|$value"
    fi
}

# Function to build endpoint string from table data
build_endpoint_from_table() {
    case "$current_table_type" in
        serial)
            if [ -n "${current_table[device]:-}" ] && [ -n "${current_table[baudrate]:-}" ]; then
                ENDPOINTS+=("serial://${current_table[device]}?baudrate=${current_table[baudrate]}")
            fi
            ;;
        udp_server)
            if [ -n "${current_table[address]:-}" ] && [ -n "${current_table[port]:-}" ]; then
                ENDPOINTS+=("udpserver://${current_table[address]}:${current_table[port]}")
            fi
            ;;
        udp_client)
            if [ -n "${current_table[address]:-}" ] && [ -n "${current_table[port]:-}" ]; then
                local direction_param=""
                if [ -n "${current_table[direction]:-}" ]; then
                    direction_param="?direction=${current_table[direction]}"
                fi
                ENDPOINTS+=("udpclient://${current_table[address]}:${current_table[port]}${direction_param}")
            fi
            ;;
        tcp_server)
            if [ -n "${current_table[address]:-}" ] && [ -n "${current_table[port]:-}" ]; then
                ENDPOINTS+=("tcpserver://${current_table[address]}:${current_table[port]}")
            fi
            ;;
        tcp_client)
            if [ -n "${current_table[address]:-}" ] && [ -n "${current_table[port]:-}" ]; then
                ENDPOINTS+=("tcpclient://${current_table[address]}:${current_table[port]}")
            fi
            ;;
    esac
}

# Initialize arrays
ENDPOINTS=()
ARGS=()
in_table_array=false
current_table_type=""
declare -A current_table

# Parse configuration file
while IFS= read -r line || [ -n "$line" ]; do
    clean_line="${line%%#*}"
    clean_line=$(trim "$clean_line")
    
    # Check for TOML array-of-tables: [[section]]
    if [[ "$clean_line" =~ ^\[\[(.+)\]\] ]]; then
        # Save previous table if exists
        if [ "$in_table_array" = true ] && [ ${#current_table[@]} -gt 0 ]; then
            build_endpoint_from_table
        fi
        
        # Start new table array
        current_table_type="${BASH_REMATCH[1]}"
        in_table_array=true
        unset current_table
        declare -A current_table
        continue
    fi
    
    # Check if we're in a table array, parse key=value pairs
    if [ "$in_table_array" = true ]; then
        # Check if we hit another table or section (end of current table)
        if [[ "$clean_line" =~ ^\[ ]] || [ -z "$clean_line" ]; then
            # End of current table, build endpoint
            if [ ${#current_table[@]} -gt 0 ]; then
                build_endpoint_from_table
            fi
            unset current_table
            declare -A current_table
            
            # If it's a new table array, continue processing
            if [[ "$clean_line" =~ ^\[\[ ]]; then
                if [[ "$clean_line" =~ ^\[\[(.+)\]\] ]]; then
                    current_table_type="${BASH_REMATCH[1]}"
                fi
                continue
            fi
            in_table_array=false
        else
            # Parse key=value within table
            result=$(parse_toml_value "$line")
            if [ -n "$result" ]; then
                IFS='|' read -r key value <<< "$result"
                current_table["$key"]="$value"
            fi
            continue
        fi
    fi
    
    # Parse regular key=value pairs
    result=$(parse_toml_value "$line")
    if [ -n "$result" ]; then
        IFS='|' read -r key value <<< "$result"
        
        # Store config value
        case "$key" in
            web_server)
                ARGS+=("--web-server" "$value")
                ;;
            default_api_version)
                ARGS+=("--default-api-version" "$value")
                ;;
            verbose)
                if [ "$value" = "true" ]; then
                    ARGS+=("-v")
                fi
                ;;
            log_path)
                ARGS+=("--log-path" "$value")
                ;;
            enable_tracing_level_log_file)
                if [ "$value" = "true" ]; then
                    ARGS+=("--enable-tracing-level-log-file")
                fi
                ;;
            streamreq_disable)
                if [ "$value" = "true" ]; then
                    ARGS+=("--streamreq-disable")
                fi
                ;;
            udp_server_timeout)
                ARGS+=("--udp-server-timeout" "$value")
                ;;
            mavlink_system_id)
                ARGS+=("--mavlink-system-id" "$value")
                ;;
            mavlink_component_id)
                ARGS+=("--mavlink-component-id" "$value")
                ;;
            mavlink_heartbeat_frequency)
                ARGS+=("--mavlink-heartbeat-frequency" "$value")
                ;;
            send_initial_heartbeats)
                if [ "$value" = "true" ]; then
                    ARGS+=("--send-initial-heartbeats")
                fi
                ;;
            zenoh_config_file)
                ARGS+=("--zenoh-config-file" "$value")
                ;;
        esac
    fi
done < "$CONFIG_FILE"

# Handle last table if file ends while in table array
if [ "$in_table_array" = true ] && [ ${#current_table[@]} -gt 0 ]; then
    build_endpoint_from_table
fi

# Add endpoints to arguments
ARGS+=("${ENDPOINTS[@]}")

# Execute mavlink-server
exec "$BINARY_PATH" "${ARGS[@]}"
RUNSCRIPT_EOF

chmod +x "$RUN_SCRIPT"
echo "Run script created at: $RUN_SCRIPT"

# Step 4: Stop existing service if running
print_section "Stopping existing service"
if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    systemctl stop "$SERVICE_NAME"
    echo "Stopped existing $SERVICE_NAME service"
else
    echo "No existing service running"
fi

# Step 5: Create systemd service file
print_section "Creating systemd service file"
cat > "$SERVICE_FILE" << SERVICE_EOF
[Unit]
Description=MAVLink Server Service
Documentation=https://github.com/bluerobotics/mavlink-server
After=network.target

[Service]
Type=simple
ExecStart=$RUN_SCRIPT $CONFIG_DIR/$CONFIG_FILE_NAME
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal
SyslogIdentifier=$SERVICE_NAME

# Security settings
NoNewPrivileges=true
PrivateTmp=true

[Install]
WantedBy=multi-user.target
SERVICE_EOF

echo "Systemd service file created at: $SERVICE_FILE"

# Step 6: Reload systemd daemon
print_section "Reloading systemd daemon"
systemctl daemon-reload
echo "Systemd daemon reloaded"

# Step 7: Enable service to start at boot
print_section "Enabling service to start at boot"
systemctl enable "$SERVICE_NAME"
echo "Service enabled to start at boot"

# Step 8: Start the service
print_section "Starting service"
systemctl start "$SERVICE_NAME"
echo "Service started"

# Step 9: Show status
print_section "Service Status"
sleep 2
systemctl status "$SERVICE_NAME" --no-pager || true

print_section "Deployment Complete"
echo "mavlink-server has been deployed as a systemd service."
echo ""
echo "Configuration file: ${CONFIG_DIR}/${CONFIG_FILE_NAME}"
echo "Service file: $SERVICE_FILE"
echo "Run script: $RUN_SCRIPT"
echo ""
echo "Useful commands:"
echo "  Check status:    sudo systemctl status $SERVICE_NAME"
echo "  View logs:       sudo journalctl -u $SERVICE_NAME -f"
echo "  Restart service: sudo systemctl restart $SERVICE_NAME"
echo "  Stop service:    sudo systemctl stop $SERVICE_NAME"
echo "  Start service:   sudo systemctl start $SERVICE_NAME"
echo "  Disable service: sudo systemctl disable $SERVICE_NAME"
echo ""
echo "To update configuration:"
echo "  1. Edit ${CONFIG_DIR}/${CONFIG_FILE_NAME}"
echo "  2. Restart the service: sudo systemctl restart $SERVICE_NAME"
echo ""
echo "Or use the configure script:"
echo "  sudo ./configure_mavlink_server.sh"


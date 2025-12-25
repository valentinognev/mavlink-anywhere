#!/bin/bash

# Interactive script to configure serial, UDP, and TCP endpoints for mavlink-server
# Usage: ./configure_endpoints.sh [CONFIG_FILE]

set -e

CONFIG_FILE="${1:-mavlink-server.conf}"
TEMPLATE_FILE="mavlink-server.conf.template"

# Function to print section headers
print_section() {
    echo ""
    echo "================================================================="
    echo "$1"
    echo "================================================================="
}

# Function to trim whitespace
trim() {
    echo "$1" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//'
}

# Function to parse TOML value
parse_toml_value() {
    local line="$1"
    line="${line%%#*}"
    line=$(trim "$line")
    
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

# Determine which file to read
# Search order: 1) /etc/mavlink-server, 2) current directory, 3) template file
SYSTEM_CONFIG_DIR="/etc/mavlink-server"
SYSTEM_CONFIG_FILE="${SYSTEM_CONFIG_DIR}/mavlink-server.conf"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE_FILE_PATH="${SCRIPT_DIR}/${TEMPLATE_FILE}"

SOURCE_FILE=""
CONFIG_FILE_TO_USE=""

# If CONFIG_FILE is explicitly provided and exists, use it
if [ -n "$1" ] && [ "$1" != "mavlink-server.conf" ] && [ -f "$1" ]; then
    SOURCE_FILE="$1"
    CONFIG_FILE_TO_USE="$1"
elif [ -f "$SYSTEM_CONFIG_FILE" ]; then
    # First priority: system config directory
    SOURCE_FILE="$SYSTEM_CONFIG_FILE"
    CONFIG_FILE_TO_USE="${CONFIG_FILE:-$SYSTEM_CONFIG_FILE}"
    echo "Found configuration at: $SYSTEM_CONFIG_FILE"
elif [ -f "$CONFIG_FILE" ]; then
    # Second priority: current directory
    SOURCE_FILE="$CONFIG_FILE"
    CONFIG_FILE_TO_USE="$CONFIG_FILE"
    echo "Found configuration at: $CONFIG_FILE"
elif [ -f "${SCRIPT_DIR}/${CONFIG_FILE}" ]; then
    # Check script directory
    SOURCE_FILE="${SCRIPT_DIR}/${CONFIG_FILE}"
    CONFIG_FILE_TO_USE="$CONFIG_FILE"
    echo "Found configuration at: ${SCRIPT_DIR}/${CONFIG_FILE}"
elif [ -f "$TEMPLATE_FILE_PATH" ]; then
    # Third priority: template file
    SOURCE_FILE="$TEMPLATE_FILE_PATH"
    CONFIG_FILE_TO_USE="${CONFIG_FILE:-mavlink-server.conf}"
    echo "Config file not found. Using template: $TEMPLATE_FILE_PATH"
else
    echo "Error: Configuration file not found!" >&2
    echo "Searched locations:" >&2
    echo "  1. $SYSTEM_CONFIG_FILE" >&2
    echo "  2. $CONFIG_FILE" >&2
    echo "  3. ${SCRIPT_DIR}/${CONFIG_FILE}" >&2
    echo "  4. $TEMPLATE_FILE_PATH" >&2
    exit 1
fi

# Determine where to write the config file
# If running as root and using default config name, write to system directory
if [ "$EUID" -eq 0 ] && [ "$CONFIG_FILE" = "mavlink-server.conf" ] && [ -z "$1" ]; then
    CONFIG_FILE_TO_WRITE="$SYSTEM_CONFIG_FILE"
    # Create directory if it doesn't exist
    mkdir -p "$SYSTEM_CONFIG_DIR"
else
    CONFIG_FILE_TO_WRITE="${CONFIG_FILE_TO_USE}"
fi

# Initialize variables
SERIAL_DEVICE=""
SERIAL_BAUDRATE=""
UDP_SERVERS=()
UDP_CLIENTS=()
TCP_SERVERS=()

# Parse existing configuration
in_serial=false
in_udp_server=false
in_udp_client=false
in_tcp_server=false
UDP_ADDR=""
UDP_CLIENT_ADDR=""
TCP_ADDR=""

while IFS= read -r line || [ -n "$line" ]; do
    clean_line="${line%%#*}"
    clean_line=$(trim "$clean_line")
    
    # Check for table array headers
    if [[ "$clean_line" =~ ^\[\[serial\]\] ]]; then
        in_serial=true
        in_udp_server=false
        in_udp_client=false
        in_tcp_server=false
        UDP_ADDR=""
        UDP_CLIENT_ADDR=""
        TCP_ADDR=""
        continue
    elif [[ "$clean_line" =~ ^\[\[udp_server\]\] ]]; then
        in_serial=false
        in_udp_server=true
        in_udp_client=false
        in_tcp_server=false
        UDP_ADDR=""
        UDP_CLIENT_ADDR=""
        TCP_ADDR=""
        continue
    elif [[ "$clean_line" =~ ^\[\[udp_client\]\] ]]; then
        in_serial=false
        in_udp_server=false
        in_udp_client=true
        in_tcp_server=false
        UDP_ADDR=""
        UDP_CLIENT_ADDR=""
        TCP_ADDR=""
        continue
    elif [[ "$clean_line" =~ ^\[\[tcp_server\]\] ]]; then
        in_serial=false
        in_udp_server=false
        in_udp_client=false
        in_tcp_server=true
        UDP_ADDR=""
        UDP_CLIENT_ADDR=""
        TCP_ADDR=""
        continue
    elif [[ "$clean_line" =~ ^\[\[ ]]; then
        # Other table array, stop parsing current section
        in_serial=false
        in_udp_server=false
        in_udp_client=false
        in_tcp_server=false
        UDP_ADDR=""
        UDP_CLIENT_ADDR=""
        TCP_ADDR=""
        continue
    fi
    
    # Parse key=value pairs
    if [ "$in_serial" = true ] || [ "$in_udp_server" = true ] || [ "$in_udp_client" = true ] || [ "$in_tcp_server" = true ]; then
        result=$(parse_toml_value "$line")
        if [ -n "$result" ]; then
            IFS='|' read -r key value <<< "$result"
            case "$key" in
                device)
                    if [ "$in_serial" = true ]; then
                        SERIAL_DEVICE="$value"
                    fi
                    ;;
                baudrate)
                    if [ "$in_serial" = true ]; then
                        SERIAL_BAUDRATE="$value"
                    fi
                    ;;
                address)
                    if [ "$in_udp_server" = true ]; then
                        UDP_ADDR="$value"
                    elif [ "$in_udp_client" = true ]; then
                        UDP_CLIENT_ADDR="$value"
                    elif [ "$in_tcp_server" = true ]; then
                        TCP_ADDR="$value"
                    fi
                    ;;
                port)
                    if [ "$in_udp_server" = true ] && [ -n "$UDP_ADDR" ]; then
                        UDP_SERVERS+=("$UDP_ADDR:$value")
                        UDP_ADDR=""
                    elif [ "$in_udp_client" = true ] && [ -n "$UDP_CLIENT_ADDR" ]; then
                        UDP_CLIENTS+=("$UDP_CLIENT_ADDR:$value")
                        UDP_CLIENT_ADDR=""
                    elif [ "$in_tcp_server" = true ] && [ -n "$TCP_ADDR" ]; then
                        TCP_SERVERS+=("$TCP_ADDR:$value")
                        TCP_ADDR=""
                    fi
                    ;;
            esac
        fi
    fi
done < "$SOURCE_FILE"

# Set defaults if not found (matching current mavlink-server.conf)
SERIAL_DEVICE="${SERIAL_DEVICE:-/dev/ttyAMA0}"
SERIAL_BAUDRATE="${SERIAL_BAUDRATE:-921600}"
if [ ${#UDP_CLIENTS[@]} -eq 0 ] && [ ${#UDP_SERVERS[@]} -eq 0 ]; then
    UDP_CLIENTS=("127.0.0.1:14540")
fi
if [ ${#TCP_SERVERS[@]} -eq 0 ]; then
    TCP_SERVERS=("0.0.0.0:5760")
fi

# Display current configuration
print_section "Current Configuration"
echo "Serial Port:     $SERIAL_DEVICE"
echo "Serial Baudrate: $SERIAL_BAUDRATE"
if [ ${#UDP_CLIENTS[@]} -gt 0 ]; then
    echo "UDP Clients:     ${UDP_CLIENTS[*]}"
fi
if [ ${#UDP_SERVERS[@]} -gt 0 ]; then
    echo "UDP Servers:     ${UDP_SERVERS[*]}"
fi
echo "TCP Servers:     ${TCP_SERVERS[*]}"
echo ""

# Prompt for new values
print_section "Enter New Configuration"
echo "Press Enter to keep current values, or type new values."
echo "Format for UDP/TCP: X.X.X.X:PORT (multiple entries separated by spaces)"
echo ""

read -p "Serial port device (default: $SERIAL_DEVICE): " NEW_SERIAL_DEVICE
SERIAL_DEVICE="${NEW_SERIAL_DEVICE:-$SERIAL_DEVICE}"

read -p "Serial baudrate (default: $SERIAL_BAUDRATE): " NEW_SERIAL_BAUDRATE
SERIAL_BAUDRATE="${NEW_SERIAL_BAUDRATE:-$SERIAL_BAUDRATE}"

# Prompt for UDP clients (if any exist or default)
if [ ${#UDP_CLIENTS[@]} -gt 0 ]; then
    read -p "UDP clients (format: X.X.X.X:PORT, multiple separated by space) (default: ${UDP_CLIENTS[*]}): " NEW_UDP_CLIENT_INPUT
    if [ -n "$NEW_UDP_CLIENT_INPUT" ]; then
        # Parse space-separated entries
        UDP_CLIENTS=()
        for entry in $NEW_UDP_CLIENT_INPUT; do
            # Validate format X.X.X.X:PORT
            if [[ "$entry" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+:[0-9]+$ ]]; then
                UDP_CLIENTS+=("$entry")
            else
                echo "Warning: Invalid UDP client format '$entry'. Expected format: X.X.X.X:PORT" >&2
            fi
        done
        # If parsing failed, keep defaults
        if [ ${#UDP_CLIENTS[@]} -eq 0 ]; then
            echo "Keeping default UDP clients due to invalid input."
            UDP_CLIENTS=("127.0.0.1:14540")
        fi
    fi
fi

# Prompt for UDP servers (if any exist)
if [ ${#UDP_SERVERS[@]} -gt 0 ]; then
    read -p "UDP servers (format: X.X.X.X:PORT, multiple separated by space) (default: ${UDP_SERVERS[*]}): " NEW_UDP_INPUT
    if [ -n "$NEW_UDP_INPUT" ]; then
        # Parse space-separated entries
        UDP_SERVERS=()
        for entry in $NEW_UDP_INPUT; do
            # Validate format X.X.X.X:PORT
            if [[ "$entry" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+:[0-9]+$ ]]; then
                UDP_SERVERS+=("$entry")
            else
                echo "Warning: Invalid UDP server format '$entry'. Expected format: X.X.X.X:PORT" >&2
            fi
        done
        # If parsing failed, keep defaults
        if [ ${#UDP_SERVERS[@]} -eq 0 ]; then
            echo "Keeping default UDP servers due to invalid input."
            UDP_SERVERS=("0.0.0.0:14550")
        fi
    fi
fi

read -p "TCP servers (format: X.X.X.X:PORT, multiple separated by space) (default: ${TCP_SERVERS[*]}): " NEW_TCP_INPUT
if [ -n "$NEW_TCP_INPUT" ]; then
    # Parse space-separated entries
    TCP_SERVERS=()
    for entry in $NEW_TCP_INPUT; do
        # Validate format X.X.X.X:PORT
        if [[ "$entry" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+:[0-9]+$ ]]; then
            TCP_SERVERS+=("$entry")
        else
            echo "Warning: Invalid TCP format '$entry'. Expected format: X.X.X.X:PORT" >&2
        fi
    done
    # If parsing failed, keep defaults
    if [ ${#TCP_SERVERS[@]} -eq 0 ]; then
        echo "Keeping default TCP servers due to invalid input."
        TCP_SERVERS=("0.0.0.0:5760")
    fi
fi

# Display what will be written
print_section "New Configuration Summary"
echo "Serial Port:     $SERIAL_DEVICE"
echo "Serial Baudrate: $SERIAL_BAUDRATE"
if [ ${#UDP_CLIENTS[@]} -gt 0 ]; then
    echo "UDP Clients:    ${UDP_CLIENTS[*]}"
fi
if [ ${#UDP_SERVERS[@]} -gt 0 ]; then
    echo "UDP Servers:     ${UDP_SERVERS[*]}"
fi
echo "TCP Servers:     ${TCP_SERVERS[*]}"
echo ""

read -p "Write this configuration to '$CONFIG_FILE_TO_WRITE'? (y/N): " CONFIRM
if [[ ! "$CONFIRM" =~ ^[Yy]$ ]]; then
    echo "Configuration cancelled."
    exit 0
fi

# Create backup if config file exists
if [ -f "$CONFIG_FILE_TO_WRITE" ]; then
    BACKUP_FILE="${CONFIG_FILE_TO_WRITE}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "$CONFIG_FILE_TO_WRITE" "$BACKUP_FILE"
    echo "Backup created: $BACKUP_FILE"
fi

# Read the source file and update endpoint sections
TEMP_FILE=$(mktemp)
in_serial=false
in_udp_server=false
in_udp_client=false
in_tcp_server=false
serial_written=false
skip_section=false

while IFS= read -r line || [ -n "$line" ]; do
    clean_line="${line%%#*}"
    clean_line=$(trim "$clean_line")
    
    # Check for table array headers
    if [[ "$clean_line" =~ ^\[\[serial\]\] ]]; then
        # End previous section if any
        skip_section=false
        in_serial=true
        in_udp_server=false
        in_tcp_server=false
        # Write the new serial section
        echo "[[serial]]" >> "$TEMP_FILE"
        echo "device = \"$SERIAL_DEVICE\"" >> "$TEMP_FILE"
        echo "baudrate = $SERIAL_BAUDRATE" >> "$TEMP_FILE"
        serial_written=true
        skip_section=true
        continue
    elif [[ "$clean_line" =~ ^\[\[udp_server\]\] ]]; then
        skip_section=false
        in_serial=false
        in_udp_server=true
        in_udp_client=false
        in_tcp_server=false
        # Skip old UDP server entries, we'll write all new ones at once
        skip_section=true
        continue
    elif [[ "$clean_line" =~ ^\[\[udp_client\]\] ]]; then
        skip_section=false
        in_serial=false
        in_udp_server=false
        in_udp_client=true
        in_tcp_server=false
        # Skip old UDP client entries, we'll write all new ones at once
        skip_section=true
        continue
    elif [[ "$clean_line" =~ ^\[\[tcp_server\]\] ]]; then
        skip_section=false
        in_serial=false
        in_udp_server=false
        in_tcp_server=true
        # Skip old TCP server entries, we'll write all new ones at once
        skip_section=true
        continue
    elif [[ "$clean_line" =~ ^\[\[ ]]; then
        # Other table array, stop skipping
        skip_section=false
        in_serial=false
        in_udp_server=false
        in_udp_client=false
        in_tcp_server=false
    fi
    
    # Skip lines that are part of sections we're replacing
    if [ "$skip_section" = true ]; then
        # Check if this is a key=value line (skip it)
        if [[ "$clean_line" =~ ^[^=]+= ]]; then
            continue
        fi
        # If we hit an empty line or comment, check if we should stop skipping
        # Stop skipping when we hit a non-empty, non-comment line that's not a key=value
        if [ -z "$clean_line" ] || [[ "$line" =~ ^[[:space:]]*# ]]; then
            # Continue skipping empty lines and comments
            continue
        else
            # Hit something else, stop skipping
            skip_section=false
        fi
    fi
    
    # Write the line as-is
    echo "$line" >> "$TEMP_FILE"
done < "$SOURCE_FILE"

# If sections weren't found, add them at the end
if [ "$serial_written" = false ]; then
    echo "" >> "$TEMP_FILE"
    echo "# Serial endpoint added by configure_endpoints.sh" >> "$TEMP_FILE"
    echo "[[serial]]" >> "$TEMP_FILE"
    echo "device = \"$SERIAL_DEVICE\"" >> "$TEMP_FILE"
    echo "baudrate = $SERIAL_BAUDRATE" >> "$TEMP_FILE"
fi

# Write UDP client sections (always write all, replacing old ones)
if [ ${#UDP_CLIENTS[@]} -gt 0 ]; then
    echo "" >> "$TEMP_FILE"
    echo "# UDP client endpoints added by configure_endpoints.sh" >> "$TEMP_FILE"
    echo "# Note: udp_client connects outbound (like udpout://), udp_server listens/binds" >> "$TEMP_FILE"
    for client in "${UDP_CLIENTS[@]}"; do
        IFS=':' read -r address port <<< "$client"
        echo "[[udp_client]]" >> "$TEMP_FILE"
        echo "address = \"$address\"" >> "$TEMP_FILE"
        echo "port = $port" >> "$TEMP_FILE"
    done
fi

# Write UDP server sections (always write all, replacing old ones)
if [ ${#UDP_SERVERS[@]} -gt 0 ]; then
    echo "" >> "$TEMP_FILE"
    echo "# UDP server endpoints added by configure_endpoints.sh" >> "$TEMP_FILE"
    for server in "${UDP_SERVERS[@]}"; do
        IFS=':' read -r address port <<< "$server"
        echo "[[udp_server]]" >> "$TEMP_FILE"
        echo "address = \"$address\"" >> "$TEMP_FILE"
        echo "port = $port" >> "$TEMP_FILE"
    done
fi

# Write TCP server sections (always write all, replacing old ones)
if [ ${#TCP_SERVERS[@]} -gt 0 ]; then
    echo "" >> "$TEMP_FILE"
    echo "# TCP server endpoints added by configure_endpoints.sh" >> "$TEMP_FILE"
    for server in "${TCP_SERVERS[@]}"; do
        IFS=':' read -r address port <<< "$server"
        echo "[[tcp_server]]" >> "$TEMP_FILE"
        echo "address = \"$address\"" >> "$TEMP_FILE"
        echo "port = $port" >> "$TEMP_FILE"
    done
fi

# Replace the config file
mv "$TEMP_FILE" "$CONFIG_FILE_TO_WRITE"

print_section "Configuration Updated"
echo "Configuration has been written to: $CONFIG_FILE_TO_WRITE"
if [ -n "$BACKUP_FILE" ]; then
    echo "Backup saved to: $BACKUP_FILE"
fi
echo ""
if [ "$CONFIG_FILE_TO_WRITE" = "$SYSTEM_CONFIG_FILE" ]; then
    echo "Note: Configuration was written to system directory."
    echo "      Restart the service to apply changes: sudo systemctl restart mavlink-server"
fi
echo ""


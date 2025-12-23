#!/bin/bash

# Script to read mavlink-server.conf and run mavlink-server with configuration values
# Usage: ./run_mavlink_server.sh [CONFIG_FILE] [ENDPOINTS...]

set -e

# Default configuration file path
CONFIG_FILE="${1:-mavlink-server.conf}"

# Shift to get endpoints if config file was provided as first argument
if [ "$1" = "$CONFIG_FILE" ] && [ -f "$CONFIG_FILE" ]; then
    shift
fi

# Remaining arguments are endpoints
ENDPOINTS=("$@")

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file '$CONFIG_FILE' not found!" >&2
    exit 1
fi

# Function to trim whitespace
trim() {
    echo "$1" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//'
}

# Function to parse TOML value (handles strings, numbers, booleans)
parse_toml_value() {
    local line="$1"
    # Remove comments
    line="${line%%#*}"
    # Trim whitespace
    line=$(trim "$line")
    
    if [ -z "$line" ]; then
        return
    fi
    
    # Extract key and value
    if [[ "$line" =~ ^([^=]+)=(.*)$ ]]; then
        local key="${BASH_REMATCH[1]}"
        local value="${BASH_REMATCH[2]}"
        
        # Trim key and value
        key=$(trim "$key")
        value=$(trim "$value")
        
        # Remove quotes if present
        if [[ "$value" =~ ^\"(.*)\"$ ]]; then
            value="${BASH_REMATCH[1]}"
        fi
        
        echo "$key|$value"
    fi
}

# Function to parse array value (handles single-line arrays)
parse_array_value() {
    local value="$1"
    
    # Remove brackets
    value="${value#\[}"
    value="${value%\]}"
    value=$(trim "$value")
    
    if [ -z "$value" ]; then
        return
    fi
    
    # Use awk to properly parse comma-separated values respecting quotes
    echo "$value" | awk -F',' '{
        for(i=1; i<=NF; i++) {
            gsub(/^[ \t"]+|[" \t]+$/, "", $i)
            if ($i != "") print $i
        }
    }'
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

# Parse configuration file
in_array=false
array_key=""
array_values=()
in_table_array=false
current_table_type=""
declare -A current_table

while IFS= read -r line || [ -n "$line" ]; do
    # Remove comments and get clean line
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
        # Clear the associative array
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
            # Clear the associative array
            unset current_table
            declare -A current_table
            
            # If it's a new table array, continue processing
            if [[ "$clean_line" =~ ^\[\[ ]]; then
                current_table_type="${BASH_REMATCH[1]}"
                continue
            fi
            # Otherwise, we're done with table arrays
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
    
    # Check if we're in a multi-line array (for generic endpoints)
    if [ "$in_array" = true ]; then
        # Check for array end
        if [[ "$clean_line" =~ ^\] ]]; then
            # End of array
            for endpoint in "${array_values[@]}"; do
                endpoint=$(trim "$endpoint")
                endpoint="${endpoint#\"}"
                endpoint="${endpoint%\"}"
                if [ -n "$endpoint" ]; then
                    ENDPOINTS+=("$endpoint")
                fi
            done
            in_array=false
            array_key=""
            array_values=()
            continue
        fi
        
        # Add to array values
        if [ -n "$clean_line" ]; then
            # Remove comma if present
            clean_line="${clean_line%,}"
            clean_line=$(trim "$clean_line")
            clean_line="${clean_line#\"}"
            clean_line="${clean_line%\"}"
            if [ -n "$clean_line" ]; then
                array_values+=("$clean_line")
            fi
        fi
        continue
    fi
    
    # Check for array start (generic endpoints)
    if [[ "$line" =~ ^([^=]+)=\[ ]]; then
        local key="${BASH_REMATCH[1]}"
        key=$(trim "$key")
        
        if [ "$key" = "endpoints" ]; then
            # Check if it's a single-line array
            if [[ "$line" =~ \[(.*)\] ]]; then
                # Single-line array
                local array_content="${BASH_REMATCH[1]}"
                while IFS= read -r endpoint; do
                    if [ -n "$endpoint" ]; then
                        ENDPOINTS+=("$endpoint")
                    fi
                done < <(parse_array_value "[$array_content]")
            else
                # Multi-line array
                in_array=true
                array_key="$key"
            fi
            continue
        fi
    fi
    
    # Parse regular key=value pairs
    result=$(parse_toml_value "$line")
    if [ -n "$result" ]; then
        IFS='|' read -r key value <<< "$result"
        
        # Store config value (convert key to uppercase with underscores)
        declare "CONFIG_${key^^}=$value"
    fi
done < "$CONFIG_FILE"

# Handle last table if file ends while in table array
if [ "$in_table_array" = true ] && [ ${#current_table[@]} -gt 0 ]; then
    build_endpoint_from_table
fi

# Build command arguments
ARGS=()

# Web server configuration
if [ -n "${CONFIG_WEB_SERVER:-}" ]; then
    ARGS+=("--web-server" "${CONFIG_WEB_SERVER}")
fi

# Default API version
if [ -n "${CONFIG_DEFAULT_API_VERSION:-}" ]; then
    ARGS+=("--default-api-version" "${CONFIG_DEFAULT_API_VERSION}")
fi

# Verbose flag
if [ "${CONFIG_VERBOSE:-false}" = "true" ]; then
    ARGS+=("-v")
fi

# Log path
if [ -n "${CONFIG_LOG_PATH:-}" ]; then
    ARGS+=("--log-path" "${CONFIG_LOG_PATH}")
fi

# Enable tracing level log file
if [ "${CONFIG_ENABLE_TRACING_LEVEL_LOG_FILE:-false}" = "true" ]; then
    ARGS+=("--enable-tracing-level-log-file")
fi

# Streamreq disable
if [ "${CONFIG_STREAMREQ_DISABLE:-false}" = "true" ]; then
    ARGS+=("--streamreq-disable")
fi

# UDP server timeout
if [ -n "${CONFIG_UDP_SERVER_TIMEOUT:-}" ]; then
    ARGS+=("--udp-server-timeout" "${CONFIG_UDP_SERVER_TIMEOUT}")
fi

# MAVLink system ID
if [ -n "${CONFIG_MAVLINK_SYSTEM_ID:-}" ]; then
    ARGS+=("--mavlink-system-id" "${CONFIG_MAVLINK_SYSTEM_ID}")
fi

# MAVLink component ID
if [ -n "${CONFIG_MAVLINK_COMPONENT_ID:-}" ]; then
    ARGS+=("--mavlink-component-id" "${CONFIG_MAVLINK_COMPONENT_ID}")
fi

# MAVLink heartbeat frequency
if [ -n "${CONFIG_MAVLINK_HEARTBEAT_FREQUENCY:-}" ]; then
    ARGS+=("--mavlink-heartbeat-frequency" "${CONFIG_MAVLINK_HEARTBEAT_FREQUENCY}")
fi

# Send initial heartbeats
if [ "${CONFIG_SEND_INITIAL_HEARTBEATS:-false}" = "true" ]; then
    ARGS+=("--send-initial-heartbeats")
fi

# MAVLink version - excluded from arguments
# if [ -n "${CONFIG_MAVLINK_VERSION:-}" ]; then
#     ARGS+=("--mavlink-version" "${CONFIG_MAVLINK_VERSION}")
# fi

# Zenoh config file
if [ -n "${CONFIG_ZENOH_CONFIG_FILE:-}" ]; then
    ARGS+=("--zenoh-config-file" "${CONFIG_ZENOH_CONFIG_FILE}")
fi

# Add endpoints
ARGS+=("${ENDPOINTS[@]}")

# Check if mavlink-server exists
# First check for local binary in current directory
if [ -f "./mavlink-server" ] && [ -x "./mavlink-server" ]; then
    MAVLINK_SERVER_CMD="./mavlink-server"
elif [ -f "mavlink-server" ] && [ -x "mavlink-server" ]; then
    MAVLINK_SERVER_CMD="mavlink-server"
else
    # Then check PATH
    MAVLINK_SERVER_CMD=$(command -v mavlink-server 2>/dev/null)
    if [ -z "$MAVLINK_SERVER_CMD" ]; then
        echo "Error: mavlink-server command not found!" >&2
        echo "Please ensure mavlink-server is installed and in your PATH," >&2
        echo "or place a mavlink-server binary in the current directory." >&2
        exit 1
    fi
fi

# Check if the binary is executable
if [ ! -x "$MAVLINK_SERVER_CMD" ]; then
    echo "Error: mavlink-server binary is not executable: $MAVLINK_SERVER_CMD" >&2
    exit 1
fi

# Display configuration summary
echo "Starting mavlink-server with configuration from: $CONFIG_FILE"
echo "Using binary: $MAVLINK_SERVER_CMD"
if [ ${#ENDPOINTS[@]} -gt 0 ]; then
    echo "Endpoints: ${ENDPOINTS[*]}"
else
    echo "Note: No endpoints specified in config file."
    echo "      Endpoints can be added to the config file or passed as command-line arguments."
    echo "      Example: $0 $CONFIG_FILE udpserver://0.0.0.0:14550"
fi
echo ""

# Display arguments being passed (for debugging)
if [ "${DEBUG:-}" = "1" ]; then
    echo "Debug: Command arguments:"
    for arg in "${ARGS[@]}"; do
        echo "  '$arg'"
    done
    echo ""
fi

# Check binary architecture before attempting to run
if command -v file &> /dev/null; then
    BINARY_INFO=$(file "$MAVLINK_SERVER_CMD" 2>/dev/null || echo "unknown")
    echo "Binary info: $BINARY_INFO"
    echo ""
fi

# Run mavlink-server
# Note: If you get "cannot execute binary file: Exec format error",
# it means the binary is for a different CPU architecture
echo "Running mavlink-server with arguments: ${ARGS[*]}"
echo ""
"$MAVLINK_SERVER_CMD" "${ARGS[@]}"


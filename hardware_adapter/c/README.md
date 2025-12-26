# Hardware Adapter C Implementation

This directory contains the C implementation of the hardware adapter, converted from the Python version.

## Overview

The hardware adapter acts as a bridge between the system manager and the MAVLink flight controller. It:
- Receives commands from the system manager via ZMQ
- Sends commands to the flight controller via MAVLink
- Receives flight data from the flight controller via MAVLink
- Publishes flight data to the system manager via ZMQ

## Architecture

The implementation uses two threads:
1. **Command Thread**: Handles incoming commands from system_manager and forwards them to MAVLink
2. **Data Thread**: Maintains the current flight data structure and publishes it to system_manager

## Dependencies

- **libzmq**: ZeroMQ library for inter-process communication
- **pthread**: POSIX threads for multi-threading
- **mavlink**: MAVLink C library (needs to be integrated)

## Building

### Install Dependencies

On Ubuntu/Debian:
```bash
make install-deps
```

Or manually:
```bash
sudo apt-get install libzmq3-dev build-essential
```

### Build

```bash
make
```

This will create the executable at `bin/hardware_adapter`.

### Clean

```bash
make clean
```

## Usage

```bash
./bin/hardware_adapter [log_directory]
```

If no log directory is specified, it defaults to `../logs/`.

## Implementation Notes

### MAVLink Integration

The current implementation includes placeholder functions for MAVLink communication. To complete the implementation:

1. Install the MAVLink C library or include the MAVLink headers
2. Replace the placeholder functions in `hardware_adapter.c`:
   - `mavlink_connect()`
   - `mavlink_disconnect()`
   - `mavlink_receive_message()`
   - `mavlink_send_heartbeat()`
   - `mavlink_send_set_position_target_local_ned()`
   - `mavlink_send_set_attitude_target()`
   - `mavlink_send_command_long()`
   - `mavlink_wait_heartbeat()`

3. Update `hardware_adapter_parse()` to properly parse MAVLink messages

### Serialization

The current implementation uses a simplified serialization (direct memory copy) which is not portable across different architectures. For production use, consider:

- **MessagePack**: Binary serialization format
- **Protocol Buffers**: Google's serialization format
- **JSON**: Text-based format (slower but more portable)

Update the serialization functions:
- `flight_data_serialize()`
- `flight_data_deserialize()`
- `deserialize_attitude_cmd()`
- `deserialize_vel_cmd()`

### ZMQ Topics

The ZMQ topics are defined in `zmq_topics.h` and match the Python implementation:
- Publisher port: 7790
- Subscriber port: 7793

## File Structure

```
c/
├── common.h              - Common data structures and utilities
├── common.c              - Implementation of common functions
├── low_pass_filter.h     - Low pass filter header
├── low_pass_filter.c     - Low pass filter implementation
├── zmq_topics.h          - ZMQ topic definitions
├── zmq_wrapper.h         - ZMQ wrapper header
├── zmq_wrapper.c         - ZMQ wrapper implementation
├── hardware_adapter.h    - Hardware adapter header
├── hardware_adapter.c    - Hardware adapter implementation
├── main.c                - Main entry point
├── Makefile              - Build configuration
└── README.md             - This file
```

## Differences from Python Version

1. **Static Typing**: All types are explicitly defined
2. **Manual Memory Management**: Memory must be explicitly allocated and freed
3. **Threading**: Uses pthreads instead of Python's threading module
4. **Serialization**: Uses simplified binary format (needs proper implementation)
5. **MAVLink**: Uses placeholder functions (needs MAVLink C library integration)

## TODO

- [ ] Integrate MAVLink C library
- [ ] Implement proper message parsing
- [ ] Implement proper serialization (MessagePack/Protobuf)
- [ ] Add error handling and logging
- [ ] Add unit tests
- [ ] Add configuration file support


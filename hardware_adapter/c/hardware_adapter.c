#include "hardware_adapter.h"
#include "zmq_topics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>

// Note: This implementation uses a simplified mavlink interface
// In a real implementation, you would use the mavlink C library:
// #include <mavlink/v2.0/common/mavlink.h>
// For now, we'll use placeholder types and functions

// Placeholder for mavlink connection (replace with actual mavlink types)
typedef struct {
    int target_system;
    int target_component;
    int fd;  // File descriptor for UDP connection
} mavlink_connection_t;

// Forward declarations
static void* mavlink_connect(const char* address);
static void mavlink_disconnect(void* conn);
static int mavlink_receive_message(void* conn, char* msg_type, size_t msg_type_size, void* msg_dict, size_t msg_dict_size);
static int mavlink_send_heartbeat(void* conn);
static int mavlink_send_set_position_target_local_ned(void* conn, uint32_t time_boot_ms, uint8_t coordinate_frame,
                                                       uint16_t type_mask, const vec3_t* pos, const vec3_t* vel,
                                                       const vec3_t* acc, double yaw, double yaw_rate);
static int mavlink_send_set_attitude_target(void* conn, uint32_t time_boot_ms, uint16_t type_mask,
                                            const quaternion_t* q, double body_roll_rate, double body_pitch_rate,
                                            double body_yaw_rate, double thrust);
static int mavlink_send_command_long(void* conn, uint16_t command, float param1, float param2, float param3,
                                     float param4, float param5, float param6, float param7);
static int mavlink_wait_heartbeat(void* conn, int timeout_ms);

// Serialization helper (simplified - in production use proper serialization like msgpack or protobuf)
static size_t flight_data_serialize(const flight_data_t* fd, void* buffer, size_t buffer_size);
static int flight_data_deserialize(flight_data_t* fd, const void* buffer, size_t buffer_size);

// Command deserialization helpers
static int deserialize_attitude_cmd(const void* data, size_t data_len, quaternion_t* quat, vec3_t* rpy_rate, 
                                     double* thrust, bool* is_rate);
static int deserialize_vel_cmd(const void* data, size_t data_len, vec3_t* vel, double* yaw, double* yaw_rate);

// Initialize hardware adapter
int hardware_adapter_init(hardware_adapter_t* adapter, const char* log_dir) {
    if (adapter == NULL) {
        return -1;
    }
    
    memset(adapter, 0, sizeof(hardware_adapter_t));
    
    // Initialize log directory
    if (log_dir != NULL) {
        adapter->log_dir = strdup(log_dir);
    } else {
        adapter->log_dir = strdup("./logs/");
    }
    
    // Initialize mutex
    if (pthread_mutex_init(&adapter->data_lock, NULL) != 0) {
        fprintf(stderr, "Failed to initialize data mutex\n");
        return -1;
    }
    
    // Initialize filters
    lpf_init(&adapter->vertical_speed_filter, 0.1, false, LPF_TYPE_FIRST_ORDER);
    lpf_init(&adapter->altitude_filter, 0.3, false, LPF_TYPE_FIRST_ORDER);
    adapter->prev_alt_m = -1.0;  // Invalid value to indicate not initialized
    adapter->alt_vel_count = 0;
    adapter->prev_vel_vertical = 0.0;
    adapter->prev_alt_ts = 0.0;
    
    // Initialize flight data
    flight_data_init(&adapter->current_data);
    
    // Initialize ZMQ
    zmq_wrapper_init();
    
    // Initialize mavlink
    adapter->mavlink_address = strdup("udp:127.0.0.1:14540");
    if (hardware_adapter_init_mavlink(adapter) != 0) {
        fprintf(stderr, "Failed to initialize mavlink connection\n");
        adapter->init_success = false;
        return -1;
    }
    
    // Create ZMQ publisher socket
    adapter->pub_socket = zmq_publisher_create(TOPIC_MAVLINK_PORT);
    if (adapter->pub_socket == NULL) {
        fprintf(stderr, "Failed to create ZMQ publisher socket\n");
        adapter->init_success = false;
        return -1;
    }
    
    // Wait a bit for publisher to bind
    usleep(200000);  // 200ms
    
    // Create ZMQ subscriber socket
    adapter->sub_socket = zmq_subscriber_create(TOPIC_GUIDANCE_CMD_PORT);
    if (adapter->sub_socket == NULL) {
        fprintf(stderr, "Failed to create ZMQ subscriber socket\n");
        adapter->init_success = false;
        return -1;
    }
    
    // Subscribe to all command topics
    zmq_subscriber_subscribe(adapter->sub_socket, TOPIC_GUIDANCE_CMD_ATTITUDE);
    zmq_subscriber_subscribe(adapter->sub_socket, TOPIC_GUIDANCE_CMD_VEL_NED);
    zmq_subscriber_subscribe(adapter->sub_socket, TOPIC_GUIDANCE_CMD_VEL_BODY);
    zmq_subscriber_subscribe(adapter->sub_socket, TOPIC_GUIDANCE_CMD_ACC);
    zmq_subscriber_subscribe(adapter->sub_socket, TOPIC_GUIDANCE_CMD_ARM);
    
    // Wait for subscriber to connect
    usleep(100000);  // 100ms
    
    // Initialize control flags
    adapter->mavlink_connected_to_usb = false;
    adapter->disable_offboard_control = false;
    adapter->offboard_control_enabled = false;
    adapter->running = true;
    adapter->init_success = true;
    
    // Start threads
    if (pthread_create(&adapter->command_thread, NULL, hardware_adapter_command_thread_func, adapter) != 0) {
        fprintf(stderr, "Failed to create command thread\n");
        adapter->init_success = false;
        return -1;
    }
    
    if (pthread_create(&adapter->data_thread, NULL, hardware_adapter_data_thread_func, adapter) != 0) {
        fprintf(stderr, "Failed to create data thread\n");
        adapter->running = false;
        pthread_join(adapter->command_thread, NULL);
        adapter->init_success = false;
        return -1;
    }
    
    printf("Hardware adapter threads started successfully\n");
    printf("Publishing to topic: %s on port: %d\n", TOPIC_MAVLINK_FLIGHT_DATA, TOPIC_MAVLINK_PORT);
    
    return 0;
}

void hardware_adapter_cleanup(hardware_adapter_t* adapter) {
    if (adapter == NULL) {
        return;
    }
    
    // Stop threads
    hardware_adapter_stop(adapter);
    
    // Close ZMQ sockets
    if (adapter->pub_socket != NULL) {
        zmq_publisher_destroy(adapter->pub_socket);
        adapter->pub_socket = NULL;
    }
    if (adapter->sub_socket != NULL) {
        zmq_subscriber_destroy(adapter->sub_socket);
        adapter->sub_socket = NULL;
    }
    
    // Disconnect mavlink
    if (adapter->mavlink_connection != NULL) {
        mavlink_disconnect(adapter->mavlink_connection);
        adapter->mavlink_connection = NULL;
    }
    
    // Cleanup flight data
    flight_data_cleanup(&adapter->current_data);
    
    // Destroy mutex
    pthread_mutex_destroy(&adapter->data_lock);
    
    // Free strings
    if (adapter->log_dir != NULL) {
        free(adapter->log_dir);
        adapter->log_dir = NULL;
    }
    if (adapter->mavlink_address != NULL) {
        free(adapter->mavlink_address);
        adapter->mavlink_address = NULL;
    }
    
    zmq_wrapper_cleanup();
}

bool hardware_adapter_init_succeeded(hardware_adapter_t* adapter) {
    return adapter != NULL && adapter->init_success;
}

void hardware_adapter_stop(hardware_adapter_t* adapter) {
    if (adapter == NULL) {
        return;
    }
    
    adapter->running = false;
    
    // Wait for threads to finish
    if (adapter->command_thread != 0) {
        pthread_join(adapter->command_thread, NULL);
    }
    if (adapter->data_thread != 0) {
        pthread_join(adapter->data_thread, NULL);
    }
}

// Initialize mavlink connection
int hardware_adapter_init_mavlink(hardware_adapter_t* adapter) {
    adapter->mavlink_connection = (void*)mavlink_connect(adapter->mavlink_address);
    if (adapter->mavlink_connection == NULL) {
        return -1;
    }
    
    // Send heartbeat
    mavlink_send_heartbeat(adapter->mavlink_connection);
    
    // Wait for heartbeat response
    int count = 0;
    while (count < 3) {
        count++;
        mavlink_send_heartbeat(adapter->mavlink_connection);
        if (mavlink_wait_heartbeat(adapter->mavlink_connection, 2000) == 0) {
            return 0;
        }
    }
    
    return -1;
}

// Listen to mavlink messages
void hardware_adapter_listener_to_mavlink(hardware_adapter_t* adapter, bool blocking, double timeout, 
                                          bool use_lock, bool apply_filter) {
    char msg_type[64];
    char msg_dict[1024];  // Simplified - in production use proper message structure
    
    int timeout_ms = blocking ? (int)(timeout * 1000) : 0;
    int result = mavlink_receive_message(adapter->mavlink_connection, msg_type, sizeof(msg_type),
                                        msg_dict, sizeof(msg_dict));
    
    if (result < 0) {
        return;  // No message or error
    }
    
    // Parse message
    if (use_lock) {
        pthread_mutex_lock(&adapter->data_lock);
        hardware_adapter_parse(adapter, msg_type, msg_dict);
        if (apply_filter) {
            hardware_adapter_filter_data(adapter, &adapter->current_data);
        }
        pthread_mutex_unlock(&adapter->data_lock);
    } else {
        hardware_adapter_parse(adapter, msg_type, msg_dict);
        if (apply_filter) {
            hardware_adapter_filter_data(adapter, &adapter->current_data);
        }
    }
}

// Filter data
void hardware_adapter_filter_data(hardware_adapter_t* adapter, flight_data_t* current_data) {
    double vel_vertical = 0.0;
    
    if (adapter->prev_alt_m >= 0.0) {  // Valid previous altitude
        double dt = current_data->altitude_m.timestamp - adapter->prev_alt_ts;
        if (dt != 0.0) {
            current_data->altitude_m.relative = lpf_step(&adapter->altitude_filter, 
                                                          current_data->altitude_m.relative);
            double d_alt = current_data->altitude_m.relative - adapter->prev_alt_m;
            vel_vertical = d_alt / dt;
            
            if (fabs(vel_vertical - adapter->prev_vel_vertical) > MAX_VERTICAL_VEL_JUMP_M_S) {
                if (vel_vertical - adapter->prev_vel_vertical > 0) {
                    vel_vertical = adapter->prev_vel_vertical + MAX_VERTICAL_VEL_JUMP_M_S;
                } else {
                    vel_vertical = adapter->prev_vel_vertical - MAX_VERTICAL_VEL_JUMP_M_S;
                }
            }
            
            current_data->altitude_m.vertical_speed_estimate = lpf_step(&adapter->vertical_speed_filter, 
                                                                        vel_vertical);
            adapter->prev_vel_vertical = current_data->altitude_m.vertical_speed_estimate;
            adapter->prev_alt_m = current_data->altitude_m.relative;
            adapter->prev_alt_ts = current_data->altitude_m.timestamp;
        }
    } else {
        adapter->prev_alt_m = current_data->altitude_m.relative;
        adapter->prev_alt_ts = current_data->altitude_m.timestamp;
    }
}

// Parse mavlink message (simplified - needs proper mavlink message parsing)
void hardware_adapter_parse(hardware_adapter_t* adapter, const char* msg_type, void* msg_dict) {
    // This is a placeholder - in production, parse actual mavlink messages
    // For now, we'll just update timestamp
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    adapter->current_data.local_ts = ts.tv_sec + ts.tv_nsec / 1e9;
    
    // In production, parse message based on msg_type:
    // - HEARTBEAT: update mode, custom_mode_id
    // - ATTITUDE_QUATERNION: update quat_ned_bodyfrd, rpy_rates
    // - LOCAL_POSITION_NED: update pos_ned_m
    // - GLOBAL_POSITION_INT: update filt_pos_lla_deg, vel_ned
    // - HIGHRES_IMU: update imu_ned
    // - ALTITUDE: update altitude_m
    // etc.
}

// Send setpoint
void hardware_adapter_send_setpoint(hardware_adapter_t* adapter, const vec3_t* pos, const vec3_t* vel,
                                    const vec3_t* acc, double yaw, double yaw_rate) {
    if (adapter->mavlink_connection == NULL) {
        return;
    }
    
    uint32_t time_boot_ms = 0;
    uint8_t coordinate_frame = 1;  // MAV_FRAME_LOCAL_NED
    uint16_t type_mask = 0;
    
    vec3_t pos_vec, vel_vec, acc_vec;
    if (pos == NULL) {
        type_mask |= 0x07;  // Ignore position
        vec3_zero(&pos_vec);
    } else {
        vec3_copy(&pos_vec, pos);
    }
    
    if (vel == NULL) {
        type_mask |= 0x38;  // Ignore velocity
        vec3_zero(&vel_vec);
    } else {
        vec3_copy(&vel_vec, vel);
    }
    
    if (acc == NULL) {
        type_mask |= 0x1C0;  // Ignore acceleration
        vec3_zero(&acc_vec);
    } else {
        vec3_copy(&acc_vec, acc);
    }
    
    if (isnan(yaw)) {
        type_mask |= 0x400;  // Ignore yaw
        yaw = 0.0;
    }
    
    if (isnan(yaw_rate)) {
        type_mask |= 0x800;  // Ignore yaw rate
        yaw_rate = 0.0;
    }
    
    mavlink_send_set_position_target_local_ned(adapter->mavlink_connection, time_boot_ms, coordinate_frame,
                                               type_mask, &pos_vec, &vel_vec, &acc_vec, yaw, yaw_rate);
}

// Send goal attitude
void hardware_adapter_send_goal_attitude(hardware_adapter_t* adapter, double goal_thrust,
                                         const quaternion_t* goal_attitude, const rate_cmd_t* rates) {
    if (!adapter->offboard_control_enabled || adapter->mavlink_connection == NULL) {
        return;
    }
    
    uint32_t time_boot_ms = (uint32_t)(time(NULL) * 1000);
    uint16_t type_mask = 0;
    
    quaternion_t q;
    double body_roll_rate = 0.0, body_pitch_rate = 0.0, body_yaw_rate = 0.0;
    
    if (goal_attitude == NULL) {
        type_mask |= 0x01;  // Ignore attitude
        quaternion_init(&q, 0.0, 0.0, 0.0, 1.0);
    } else {
        quaternion_copy(&q, goal_attitude);
    }
    
    if (rates == NULL) {
        type_mask |= 0x0E;  // Ignore rates
    } else {
        body_roll_rate = rates->rpydot.data[0];
        body_pitch_rate = rates->rpydot.data[1];
        body_yaw_rate = rates->rpydot.data[2];
    }
    
    mavlink_send_set_attitude_target(adapter->mavlink_connection, time_boot_ms, type_mask,
                                     &q, body_roll_rate, body_pitch_rate, body_yaw_rate, goal_thrust);
}

// Send offboard command
void hardware_adapter_send_offboard_cmd(hardware_adapter_t* adapter) {
    if (adapter->mavlink_connected_to_usb || adapter->disable_offboard_control) {
        return;
    }
    
    // Send command to set mode to OFFBOARD
    // Mode ID for OFFBOARD (PX4) is typically 6
    mavlink_send_command_long(adapter->mavlink_connection, 176, 1.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Arm vehicle
void hardware_adapter_arm(hardware_adapter_t* adapter) {
    if (adapter->mavlink_connected_to_usb || adapter->disable_offboard_control) {
        return;
    }
    
    // MAV_CMD_COMPONENT_ARM_DISARM with param1=1 to arm
    mavlink_send_command_long(adapter->mavlink_connection, 400, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Send takeoff command
void hardware_adapter_send_takeoff_cmd(hardware_adapter_t* adapter, double takeoff_altitude) {
    if (adapter->mavlink_connected_to_usb || adapter->disable_offboard_control) {
        return;
    }
    
    // This is a simplified version - in production, get current altitude first
    // and send proper takeoff command
    mavlink_send_command_long(adapter->mavlink_connection, 22, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, takeoff_altitude);
}

// Send land command
void hardware_adapter_send_land_cmd(hardware_adapter_t* adapter) {
    mavlink_send_command_long(adapter->mavlink_connection, 21, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Command thread function
void* hardware_adapter_command_thread_func(void* arg) {
    hardware_adapter_t* adapter = (hardware_adapter_t*)arg;
    char topic_buffer[256];
    char data_buffer[4096];
    
    printf("Hardware_adapter: Command thread started, waiting for commands...\n");
    
    while (adapter->running) {
        int data_len = zmq_subscriber_receive(adapter->sub_socket, topic_buffer, sizeof(topic_buffer),
                                             data_buffer, sizeof(data_buffer), 100);
        
        if (data_len < 0) {
            usleep(1000);  // 1ms
            continue;
        }
        
        // Process command based on topic
        if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ATTITUDE) == 0) {
            quaternion_t quat;
            vec3_t rpy_rate;
            double thrust;
            bool is_rate;
            
            if (deserialize_attitude_cmd(data_buffer, data_len, &quat, &rpy_rate, &thrust, &is_rate) == 0) {
                if (is_rate) {
                    rate_cmd_t rate_cmd;
                    rate_cmd_init(&rate_cmd, &rpy_rate);
                    hardware_adapter_send_goal_attitude(adapter, thrust, NULL, &rate_cmd);
                } else {
                    hardware_adapter_send_goal_attitude(adapter, thrust, &quat, NULL);
                }
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_VEL_NED) == 0) {
            vec3_t vel;
            double yaw, yaw_rate;
            
            if (deserialize_vel_cmd(data_buffer, data_len, &vel, &yaw, &yaw_rate) == 0) {
                double yaw_cmd = isnan(yaw) ? NAN : yaw;
                double yaw_rate_cmd = isnan(yaw_rate) ? NAN : yaw_rate;
                hardware_adapter_send_setpoint(adapter, NULL, &vel, NULL, yaw_cmd, yaw_rate_cmd);
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_VEL_BODY) == 0) {
            vec3_t vel_body, vel_ned;
            double yaw, yaw_rate;
            
            if (deserialize_vel_cmd(data_buffer, data_len, &vel_body, &yaw, &yaw_rate) == 0) {
                // Rotate body velocity to NED frame
                pthread_mutex_lock(&adapter->data_lock);
                if (adapter->current_data.gathered.quat_ned_bodyfrd) {
                    vel_ned = quaternion_rotate_vec(&adapter->current_data.quat_ned_bodyfrd, &vel_body);
                } else {
                    vec3_copy(&vel_ned, &vel_body);  // Fallback if quaternion not available
                }
                pthread_mutex_unlock(&adapter->data_lock);
                
                double yaw_cmd = isnan(yaw) ? NAN : yaw;
                double yaw_rate_cmd = isnan(yaw_rate) ? NAN : yaw_rate;
                hardware_adapter_send_setpoint(adapter, NULL, &vel_ned, NULL, yaw_cmd, yaw_rate_cmd);
            }
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ACC) == 0) {
            vec3_t acc;
            double yaw, yaw_rate;
            
            // Similar to vel command but for acceleration
            // Implementation would deserialize acc command and send setpoint
        } else if (strcmp(topic_buffer, TOPIC_GUIDANCE_CMD_ARM) == 0) {
            hardware_adapter_arm(adapter);
        }
    }
    
    return NULL;
}

// Data thread function
void* hardware_adapter_data_thread_func(void* arg) {
    hardware_adapter_t* adapter = (hardware_adapter_t*)arg;
    struct timespec ts;
    double out_time = 0.0;
    double print_time = 0.0;
    
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double start_time = ts.tv_sec + ts.tv_nsec / 1e9;
    out_time = start_time + PUBLISH_DT;
    print_time = start_time + 1.0;
    
    while (adapter->running) {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double current_time = ts.tv_sec + ts.tv_nsec / 1e9;
        
        // Process mavlink messages
        hardware_adapter_listener_to_mavlink(adapter, true, 0.0, true, true);
        
        // Publish data at specified frequency
        if (current_time >= out_time) {
            out_time = current_time + PUBLISH_DT;
            
            pthread_mutex_lock(&adapter->data_lock);
            adapter->current_data.local_ts = current_time;
            
            // Serialize flight data
            char serialized_data[8192];
            size_t data_len = flight_data_serialize(&adapter->current_data, serialized_data, sizeof(serialized_data));
            
            if (data_len > 0) {
                zmq_publisher_send(adapter->pub_socket, TOPIC_MAVLINK_FLIGHT_DATA, serialized_data, data_len);
                adapter->current_data.message_count++;
            }
            pthread_mutex_unlock(&adapter->data_lock);
        }
        
        // Print status at lower frequency
        if (current_time >= print_time) {
            print_time = current_time + 1.0;
            pthread_mutex_lock(&adapter->data_lock);
            printf("timestamp: %.3f\n", adapter->current_data.timestamp);
            pthread_mutex_unlock(&adapter->data_lock);
        }
        
        usleep(100);  // 0.1ms
    }
    
    return NULL;
}

// Placeholder mavlink functions (replace with actual mavlink C library calls)
static void* mavlink_connect(const char* address) {
    // In production, parse address (e.g., "udp:127.0.0.1:14540")
    // and create UDP socket connection
    // For now, return placeholder
    mavlink_connection_t* conn = (mavlink_connection_t*)malloc(sizeof(mavlink_connection_t));
    if (conn == NULL) {
        return NULL;
    }
    conn->target_system = 1;
    conn->target_component = 1;
    conn->fd = -1;  // Would be actual socket fd
    return (void*)conn;
}

static void mavlink_disconnect(void* conn) {
    if (conn != NULL) {
        free(conn);
    }
}

static int mavlink_receive_message(void* conn, char* msg_type, size_t msg_type_size,
                                   void* msg_dict, size_t msg_dict_size) {
    // Placeholder - in production, receive and parse mavlink message
    // Return message type and parsed data
    return -1;  // No message
}

static int mavlink_send_heartbeat(void* conn) {
    // Placeholder - send heartbeat message
    return 0;
}

static int mavlink_send_set_position_target_local_ned(void* conn, uint32_t time_boot_ms, uint8_t coordinate_frame,
                                                      uint16_t type_mask, const vec3_t* pos, const vec3_t* vel,
                                                      const vec3_t* acc, double yaw, double yaw_rate) {
    // Placeholder - send SET_POSITION_TARGET_LOCAL_NED message
    return 0;
}

static int mavlink_send_set_attitude_target(void* conn, uint32_t time_boot_ms, uint16_t type_mask,
                                            const quaternion_t* q, double body_roll_rate, double body_pitch_rate,
                                            double body_yaw_rate, double thrust) {
    // Placeholder - send SET_ATTITUDE_TARGET message
    return 0;
}

static int mavlink_send_command_long(void* conn, uint16_t command, float param1, float param2, float param3,
                                     float param4, float param5, float param6, float param7) {
    // Placeholder - send COMMAND_LONG message
    return 0;
}

static int mavlink_wait_heartbeat(void* conn, int timeout_ms) {
    // Placeholder - wait for heartbeat message
    return -1;
}

// Serialization helpers (simplified - use proper serialization in production)
static size_t flight_data_serialize(const flight_data_t* fd, void* buffer, size_t buffer_size) {
    // Placeholder - in production, use msgpack, protobuf, or similar
    // For now, just copy structure (not portable, but works for same architecture)
    if (buffer_size < sizeof(flight_data_t)) {
        return 0;
    }
    memcpy(buffer, fd, sizeof(flight_data_t));
    return sizeof(flight_data_t);
}

static int flight_data_deserialize(flight_data_t* fd, const void* buffer, size_t buffer_size) {
    if (buffer_size < sizeof(flight_data_t)) {
        return -1;
    }
    memcpy(fd, buffer, sizeof(flight_data_t));
    return 0;
}

static int deserialize_attitude_cmd(const void* data, size_t data_len, quaternion_t* quat, vec3_t* rpy_rate,
                                     double* thrust, bool* is_rate) {
    // Placeholder - deserialize attitude command
    // In production, parse from serialized format
    return -1;
}

static int deserialize_vel_cmd(const void* data, size_t data_len, vec3_t* vel, double* yaw, double* yaw_rate) {
    // Placeholder - deserialize velocity command
    // In production, parse from serialized format
    return -1;
}


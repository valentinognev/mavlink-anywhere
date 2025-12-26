CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c11 -pthread
LDFLAGS = -pthread -lzmq -lm

# Debug flags
DEBUG_CFLAGS = -Wall -Wextra -g3 -O0 -std=c11 -pthread -DDEBUG
DEBUG_LDFLAGS = -pthread -lzmq -lm -g3

# Directories
SRC_DIR = .
OBJ_DIR = obj
OBJ_DIR_DEBUG = obj_debug
BIN_DIR = bin

# Source files
SOURCES = $(wildcard $(SRC_DIR)/*.c)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
OBJECTS_DEBUG = $(SOURCES:$(SRC_DIR)/%.c=$(OBJ_DIR_DEBUG)/%.o)

# Target executables
TARGET = $(BIN_DIR)/hardware_adapter
TARGET_DEBUG = $(BIN_DIR)/hardware_adapter_debug

# Default target
all: $(TARGET)

# Create directories
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Build target
$(TARGET): $(OBJECTS) | $(BIN_DIR)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)
	@echo "Build complete: $@"

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Debug build target
debug: $(TARGET_DEBUG)

# Create debug directories
$(OBJ_DIR_DEBUG):
	mkdir -p $(OBJ_DIR_DEBUG)

# Build debug target
$(TARGET_DEBUG): $(OBJECTS_DEBUG) | $(BIN_DIR)
	$(CC) $(OBJECTS_DEBUG) -o $@ $(DEBUG_LDFLAGS)
	@echo "Debug build complete: $@"

# Compile debug source files
$(OBJ_DIR_DEBUG)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR_DEBUG)
	$(CC) $(DEBUG_CFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -rf $(OBJ_DIR) $(OBJ_DIR_DEBUG) $(BIN_DIR)
	@echo "Clean complete"

# Install dependencies (Ubuntu/Debian)
install-deps:
	sudo apt-get update
	sudo apt-get install -y libzmq3-dev build-essential

# Help target
help:
	@echo "Available targets:"
	@echo "  all          - Build the hardware adapter (optimized, default)"
	@echo "  debug        - Build the hardware adapter with debug symbols"
	@echo "  clean        - Remove build artifacts"
	@echo "  install-deps - Install required dependencies"
	@echo "  help         - Show this help message"

.PHONY: all debug clean install-deps help


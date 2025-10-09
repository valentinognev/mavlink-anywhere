#!/bin/bash

echo "================================================================="
echo "MavlinkAnywhere: Mavlink-router Installation Script"
echo "Author: Alireza Ghaderi"
echo "GitHub: https://github.com/alireza787b/mavlink-anywhere"
echo "Contact: p30planets@gmail.com"
echo "================================================================="

# Function to print progress messages
print_progress() {
    echo "================================================================="
    echo "$1"
    echo "================================================================="
}

# Function to clean up swap space
cleanup_swap() {
    sudo dphys-swapfile swapoff
    sudo sed -i 's/CONF_SWAPSIZE=2048/CONF_SWAPSIZE=100/' /etc/dphys-swapfile  # Assuming original size is 100
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
}

# Stop any existing mavlink-router service
print_progress "Stopping any existing mavlink-router service..."
sudo systemctl stop mavlink-router

sudo apt update
sudo apt upgrade -y
sudo apt install -y wget curl mc tmux dphys-swapfile
sudo apt install -y ninja-build gcc systemd systemd-dev  pkg-config

# Navigate to home directory
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
chmod +x Miniconda3-latest-Linux-aarch64.sh
./Miniconda3-latest-Linux-aarch64.sh -b -u -p ~/miniconda
~/miniconda/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
~/miniconda/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

~/miniconda/bin/conda create -n RL python==3.11.5 -y
~/miniconda/bin/conda init
source ~/miniconda/bin/activate && conda activate RL && pip install meson pymavlink
source ~/miniconda/bin/activate && conda activate RL && pip install matplotlib sample-factory plotly scipy pymap3d || { echo "sample factory Installation failed"; cleanup_swap; deactivate; exit 1; }
source ~/miniconda/bin/activate && conda activate RL && pip install torch==2.5 || { echo "torch Installation failed"; cleanup_swap; deactivate; exit 1; }
source ~/miniconda/bin/activate && conda activate RL && pip install opencv-python-headless || { echo "opencv-python-headless Installation failed"; cleanup_swap; deactivate; exit 1; }
source ~/miniconda/bin/activate && conda activate RL && pip install modular   || { echo "modular Installation failed"; cleanup_swap; deactivate; exit 1; }
source ~/miniconda/bin/activate && conda activate RL && pip uninstall sample-factory -y

# Check if mavlink-router is already installed
if command -v mavlink-routerd &> /dev/null; then
    print_progress "mavlink-router is already installed. You're good to go!"
    exit 0
fi

# If the mavlink-router directory exists, remove it
if [ -d "mavlink-router" ]; then
    print_progress "Removing existing mavlink-router directory..."
    rm -rf mavlink-router
fi

# Update and install packages
print_progress "Updating and installing necessary packages..."
sudo apt update && sudo apt install -y git meson ninja-build pkg-config gcc g++ systemd systemd-dev python3-venv || { echo "Installation of packages failed"; cleanup_swap; exit 1; }

# Increase swap space for low-memory systems
print_progress "Increasing swap space..."
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=[0-9]*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon

# Clone and navigate into the repository
print_progress "Cloning mavlink-router repository..."
git clone https://github.com/mavlink-router/mavlink-router.git || { echo "Cloning of repository failed"; cleanup_swap; exit 1; }
cd mavlink-router || { echo "Changing directory failed"; cleanup_swap; exit 1; }

# Fetch dependencies (submodules)
print_progress "Fetching submodules..."
git submodule update --init --recursive || { echo "Submodule update failed"; cleanup_swap; exit 1; }

# Build with Meson and Ninja
print_progress "Setting up the build with Meson..."
meson setup build . || { echo "Meson setup failed"; cleanup_swap; deactivate; exit 1; }
print_progress "Building with Ninja..."
ninja -C build || { echo "Ninja build failed"; cleanup_swap; deactivate; exit 1; }

# Install
print_progress "Installing mavlink-router..."
sudo ninja -C build install || { echo "Installation failed"; cleanup_swap; deactivate; exit 1; }

cd ~

# Print success message
print_progress "mavlink-router installed successfully."

# Reset swap space to original size
print_progress "Resetting swap space to original size..."
cleanup_swap

print_progress "Installation script completed."
echo "Next steps:"
echo "1. Configure mavlink-router using the provided configuration script."
echo "2. Check the status of the mavlink-router service with: sudo systemctl status mavlink-router"
echo "3. For detailed logs, use: sudo journalctl -u mavlink-router -f"
echo "4. Use QGroundControl to connect to the Raspberry Pi's IP address on port 14550."

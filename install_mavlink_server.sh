#!/bin/bash

echo "================================================================="
echo "MavlinkAnywhere: Mavlink-server Installation Script"
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

# Stop any existing mavlink-server service
print_progress "Stopping any existing mavlink-server service..."
sudo systemctl stop mavlink-server

sudo apt update
sudo apt upgrade -y
sudo apt install -y wget curl mc tmux dphys-swapfile
sudo apt install -y ninja-build gcc systemd systemd-dev pkg-config build-essential python3-dev

# Navigate to home directory
cd ~/Downloads
wget https://github.com/bluerobotics/mavlink-server/releases/download/0.7.0/mavlink-server-aarch64-unknown-linux-musl 
mv mavlink-server-aarch64-unknown-linux-musl mavlink-server
sudo mv mavlink-server /usr/bin/mavlink-server
sudo chmod +x /usr/bin/mavlink-server

wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
chmod +x Miniconda3-latest-Linux-aarch64.sh
./Miniconda3-latest-Linux-aarch64.sh -b -u -p ~/miniconda
~/miniconda/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
~/miniconda/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

~/miniconda/bin/conda create -n RL python==3.11.5 -y
~/miniconda/bin/conda init
source ~/miniconda/bin/activate && conda activate RL && pip install meson pymavlink zmq
source ~/miniconda/bin/activate && conda activate RL && pip install matplotlib sample-factory plotly scipy pymap3d || { echo "sample factory Installation failed"; cleanup_swap; deactivate; exit 1; }
source ~/miniconda/bin/activate && conda activate RL && pip install torch==2.5 || { echo "torch Installation failed"; cleanup_swap; deactivate; exit 1; }
source ~/miniconda/bin/activate && conda activate RL && pip install opencv-python-headless || { echo "opencv-python-headless Installation failed"; cleanup_swap; deactivate; exit 1; }
# source ~/miniconda/bin/activate && conda activate RL && pip install modular   || { echo "modular Installation failed"; cleanup_swap; deactivate; exit 1; }
source ~/miniconda/bin/activate && conda activate RL && pip uninstall sample-factory -y

# Update and install packages
print_progress "Updating and installing necessary packages..."
sudo apt update && sudo apt install -y git meson ninja-build pkg-config gcc g++ systemd systemd-dev python3-venv || { echo "Installation of packages failed"; cleanup_swap; exit 1; }

print_progress "Installation script completed."
echo "Next steps:"
echo "1. Configure mavlink-server using the configure_mavlink_server.sh script."
echo "2. Check the status of the mavlink-server service with: sudo systemctl status mavlink-server"
echo "3. For detailed logs, use: sudo journalctl -u mavlink-server -f"
echo "4. Use QGroundControl to connect to the Raspberry Pi's IP address on port 5760."

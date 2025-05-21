#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print status messages
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check command status
check_status() {
    if [ $? -eq 0 ]; then
        print_status "$1"
    else
        print_error "$2"
        exit 1
    fi
}

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    print_error "Please run as root (use sudo)"
    exit 1
fi

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/device-tree/model; then
    print_error "This script is designed for Raspberry Pi"
    exit 1
fi

print_status "Starting ROS 2 Iron installation on Raspberry Pi..."

# Set up locale
print_status "Setting up locale..."
apt update && apt install -y locales
check_status "Updated package list" "Failed to update package list"

# Generate and set locale
locale-gen en_GB.UTF-8
check_status "Generated locale" "Failed to generate locale"

# Set locale variables
export LANG=en_GB.UTF-8
export LANGUAGE=en_GB:en
export LC_ALL=en_GB.UTF-8

# Update locale
update-locale LANG=en_GB.UTF-8 LANGUAGE=en_GB:en LC_ALL=en_GB.UTF-8
check_status "Updated locale" "Failed to update locale"

# Clean up any existing ROS 2 repository
print_status "Cleaning up existing ROS 2 repository..."
rm -f /etc/apt/sources.list.d/ros2.list
check_status "Cleaned up existing repository" "Failed to clean up repository"

# Add ROS 2 apt repository
print_status "Adding ROS 2 repository..."
apt update && apt install -y software-properties-common curl gnupg2
check_status "Installed required packages" "Failed to install required packages"

# Add ROS 2 repository key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
check_status "Added ROS 2 repository key" "Failed to add repository key"

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/debian $(. /etc/os-release && echo $VERSION_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
check_status "Added ROS 2 repository" "Failed to add repository"

# Update package list
apt update
check_status "Updated package list" "Failed to update package list"

# Install ROS 2
print_status "Installing ROS 2 Iron..."
apt install -y ros-iron-ros-base
check_status "Installed ROS 2 base" "Failed to install ROS 2 base"

# Install development tools and ROS tools
print_status "Installing development tools and ROS tools..."
apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    build-essential \
    git \
    cmake \
    libpython3-dev \
    python3-dev \
    python3-full \
    python3-venv
check_status "Installed development tools" "Failed to install development tools"

# Create and activate virtual environment
print_status "Setting up Python virtual environment..."
# Remove existing venv if it exists
rm -rf ~/ros2_venv
python3 -m venv ~/ros2_venv
check_status "Created virtual environment" "Failed to create virtual environment"

# Activate virtual environment
source ~/ros2_venv/bin/activate
check_status "Activated virtual environment" "Failed to activate virtual environment"

# Initialize rosdep
print_status "Initializing rosdep..."
rosdep init || true
rosdep update
check_status "Initialized rosdep" "Failed to initialize rosdep"

# Set up environment
print_status "Setting up environment..."
# Remove existing ROS 2 entries from .bashrc
sed -i '/source \/opt\/ros\/iron\/setup.bash/d' ~/.bashrc
sed -i '/source \/usr\/share\/colcon_cd\/function\/colcon_cd.sh/d' ~/.bashrc
sed -i '/export _colcon_cd_root=\/opt\/ros\/iron\//d' ~/.bashrc
sed -i '/source ~\/ros2_venv\/bin\/activate/d' ~/.bashrc

# Add new entries
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/iron/" >> ~/.bashrc
echo "source ~/ros2_venv/bin/activate" >> ~/.bashrc
check_status "Updated .bashrc" "Failed to update .bashrc"

# Install additional Python packages in virtual environment
print_status "Installing additional Python packages..."
pip3 install --upgrade pip
pip3 install -r requirements.txt
check_status "Installed Python packages" "Failed to install Python packages"

# Install ROS 2 Python packages in virtual environment
print_status "Installing ROS 2 Python packages..."
pip3 install \
    rclpy \
    ros2pkg \
    ros2run \
    ros2topic \
    ros2node \
    ros2service \
    ros2param \
    ros2action
check_status "Installed ROS 2 Python packages" "Failed to install ROS 2 Python packages"

print_status "ROS 2 Iron installation completed!"
print_status "Please run 'source ~/.bashrc' to update your environment"
print_status "You can verify the installation by running: ros2 --version"

# Print next steps
echo -e "\n${YELLOW}Next steps:${NC}"
echo "1. Run: source ~/.bashrc"
echo "2. Test ROS 2: ros2 --version"
echo "3. Try a demo: ros2 run demo_nodes_py talker"
echo "4. In another terminal: ros2 run demo_nodes_py listener"

# Print system information
echo -e "\n${YELLOW}System Information:${NC}"
echo "ROS 2 Version: Iron"
echo "Architecture: $(uname -m)"
echo "OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
echo "Python Version: $(python3 --version)" 
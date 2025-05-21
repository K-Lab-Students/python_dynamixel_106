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

print_status "Starting ROS 2 Humble installation on Raspberry Pi..."

# Set up locale
print_status "Setting up locale..."
apt update && apt install -y locales
locale-gen en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
print_status "Adding ROS 2 repository..."
apt update && apt install -y software-properties-common curl gnupg2
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/debian $(. /etc/os-release && echo $VERSION_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
print_status "Installing ROS 2 Humble..."
apt update
apt install -y ros-humble-ros-base

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
    python3-full

# Create and activate virtual environment
print_status "Setting up Python virtual environment..."
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate

# Initialize rosdep
print_status "Initializing rosdep..."
rosdep init || true
rosdep update

# Set up environment
print_status "Setting up environment..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
echo "source ~/ros2_venv/bin/activate" >> ~/.bashrc

# Install additional Python packages in virtual environment
print_status "Installing additional Python packages..."
pip3 install --upgrade pip
pip3 install -r requirements.txt

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

print_status "ROS 2 Humble installation completed!"
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
echo "ROS 2 Version: Humble"
echo "Architecture: $(uname -m)"
echo "OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
echo "Python Version: $(python3 --version)" 
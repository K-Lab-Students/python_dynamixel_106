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

print_status "Starting ROS 2 Jazzy installation on Raspberry Pi..."

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

# Install development tools and dependencies
print_status "Installing development tools and dependencies..."
apt install -y \
    git \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-dev \
    python3-venv \
    python3-full \
    build-essential \
    cmake \
    libssl-dev \
    libffi-dev \
    libxml2-dev \
    libxslt1-dev \
    libjpeg-dev \
    libpng-dev \
    libfreetype6-dev \
    libopenblas-dev \
    libblas-dev \
    liblapack-dev \
    libatlas-base-dev \
    gfortran \
    libhdf5-dev \
    libhdf5-serial-dev \
    libopenmpi-dev \
    libomp-dev \
    libboost-all-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libprotoc-dev \
    libyaml-cpp-dev \
    libcurl4-openssl-dev \
    libtinyxml2-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libconsole-bridge-dev \
    libpoco-dev \
    libpocofoundation-dev \
    libpocoutil-dev \
    libpoconet-dev \
    libpoconetssl-dev \
    libpocoxml-dev \
    libpocojson-dev \
    libpocodata-dev \
    libpocodatasqlite-dev \
    libpocodataodbc-dev \
    libpocodatamysql-dev \
    libpocodataodbc-dev \
    libpocozip-dev \
    libpocopdf-dev \
    libpocoredis-dev \
    libpocodataredis-dev \
    libpocodatamongodb-dev
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

# Install Python dependencies
print_status "Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install -r requirements.txt
pip3 install \
    colcon-common-extensions \
    vcstool \
    rosdep \
    setuptools \
    wheel \
    pytest \
    pytest-cov \
    pytest-runner \
    pytest-mock \
    pytest-asyncio \
    pytest-timeout \
    pytest-repeat \
    pytest-rerunfailures \
    pytest-xdist \
    pytest-benchmark \
    pytest-env \
    pytest-sugar \
    pytest-html \
    pytest-metadata \
    pytest-ordering \
    pytest-randomly \
    pytest-xprocess \
    pytest-xvfb
check_status "Installed Python dependencies" "Failed to install Python dependencies"

# Create ROS 2 workspace
print_status "Creating ROS 2 workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
check_status "Created workspace" "Failed to create workspace"

# Clone ROS 2 source
print_status "Cloning ROS 2 source..."
git clone https://github.com/ros2/ros2.git src/ros2
cd src/ros2
git checkout jazzy
check_status "Cloned ROS 2 source" "Failed to clone ROS 2 source"

# Install dependencies
print_status "Installing ROS 2 dependencies..."
rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
check_status "Installed ROS 2 dependencies" "Failed to install ROS 2 dependencies"

# Build ROS 2
print_status "Building ROS 2..."
cd ~/ros2_ws
colcon build --symlink-install
check_status "Built ROS 2" "Failed to build ROS 2"

# Set up environment
print_status "Setting up environment..."
# Remove existing ROS 2 entries from .bashrc
sed -i '/source ~\/ros2_ws\/install\/setup.bash/d' ~/.bashrc
sed -i '/source ~\/ros2_venv\/bin\/activate/d' ~/.bashrc

# Add new entries
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/ros2_venv/bin/activate" >> ~/.bashrc
check_status "Updated .bashrc" "Failed to update .bashrc"

print_status "ROS 2 Jazzy installation completed!"
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
echo "ROS 2 Version: Jazzy (built from source)"
echo "Architecture: $(uname -m)"
echo "OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
echo "Python Version: $(python3 --version)" 
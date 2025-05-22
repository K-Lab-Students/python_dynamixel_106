FROM python:3.11-slim-bookworm

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN useradd -m -s /bin/bash rosuser

# Create and activate virtual environment
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install Python dependencies
RUN pip3 install --upgrade pip && \
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

# Create ROS 2 workspace and set permissions
RUN mkdir -p /ros2_ws && \
    chown -R rosuser:rosuser /ros2_ws

# Copy requirements file and build script
COPY requirements.txt /ros2_ws/
COPY build_ros2.sh /ros2_ws/
RUN chmod +x /ros2_ws/build_ros2.sh

# Install project dependencies
RUN pip3 install -r /ros2_ws/requirements.txt

# Initialize rosdep
RUN rosdep init || true

# Create entrypoint script
COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# Switch to non-root user
USER rosuser

# Set working directory
WORKDIR /ros2_ws

ENTRYPOINT ["docker-entrypoint.sh"]
CMD ["bash"] 
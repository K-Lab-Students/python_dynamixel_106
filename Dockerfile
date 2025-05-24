FROM ros:jazzy

# (Optional) Install Python 3.11 if you need it instead of 3.10
# RUN apt-get update && apt-get install -y python3.11 python3.11-venv python3.11-dev

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-full \
    libzbar0 \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Set up a virtual environment
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install Python dependencies
COPY requirements.txt /workspace/
RUN pip install --upgrade pip && pip install -r /workspace/requirements.txt

# Copy your code and scripts
COPY . /workspace/
WORKDIR /workspace

# (Optional) If you still need to build ROS 2 from source, keep your build_ros2.sh
# RUN bash build_ros2.sh

# Set entrypoint or CMD as needed
CMD ["bash"] 
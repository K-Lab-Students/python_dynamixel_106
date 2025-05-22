# Autonomous Medicine Delivery Robot

This system implements an autonomous medicine delivery robot for hospital environments. The robot is capable of:
- Reading QR codes to receive delivery tasks
- Navigating to medicine storage shelves using ArUco markers
- Precise positioning for medicine pickup
- Autonomous navigation to hospital wards
- Confirming delivery by entering wards

## System Architecture

### Camera System
- **Front Camera**: QR code detection for task reception and ward entry
- **Side Camera**: ArUco marker detection for shelf alignment
- **Opposite Side Camera**: Extended vision and safety monitoring

### Navigation System
- Precise positioning using ArUco markers
- Mecanum wheel-based movement
- ROS-based control system

### Components
1. **Camera Manager** (`src/camera/camera_manager.py`)
   - Handles all camera operations
   - QR code detection
   - ArUco marker detection
   - Position calculation

2. **Navigation Controller** (`src/navigation/navigation_controller.py`)
   - Precise positioning
   - Movement control
   - State management

3. **Main Controller** (`src/main_controller.py`)
   - System coordination
   - Task execution
   - ROS integration

## Setup

### Prerequisites
- Python 3.10+
- ROS Noetic
- OpenCV
- Dynamixel SDK

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/python_dynamixel_106.git
   cd python_dynamixel_106
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Configure camera devices in `src/main_controller.py`

### Usage
1. Start ROS core:
   ```bash
   roscore
   ```

2. Run the main controller:
   ```bash
   python src/main_controller.py
   ```

## Configuration

### Camera Setup
- Front camera: Device ID 0
- Side camera: Device ID 1
- Opposite side camera: Device ID 2

### Navigation Parameters
- Optimal distance to shelf: 60mm
- Distance tolerance: 5mm
- Angle tolerance: 0.05 radians

## Task Format
QR codes should be formatted as:
```
medicine_id:ward1,ward2,ward3
```

Example:
```
med123:101,102,103
```

## Safety Features
- Continuous position monitoring
- Error detection and reporting
- Resource cleanup on shutdown

## Contributing
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License
This project is licensed under the MIT License - see the LICENSE file for details.

# Python Dynamixel Control

## ROS 2 Docker Environment

### Starting the Container

1. Build and start the container:
   ```bash
   docker-compose up -d --build
   ```

2. Enter the container:
   ```bash
   docker-compose exec ros2 bash
   ```

### Inside the Container

Once inside the container, you'll be in the `/ros2_ws` directory. Here are some common ROS 2 commands:

1. **Source the ROS 2 environment**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. **Check ROS 2 installation**:
   ```bash
   ros2 --version
   ```

3. **List available nodes**:
   ```bash
   ros2 node list
   ```

4. **List available topics**:
   ```bash
   ros2 topic list
   ```

5. **View topic messages**:
   ```bash
   ros2 topic echo /topic_name
   ```

6. **Run your nodes**:
   ```bash
   # Run main controller
   python3 src/main_controller.py

   # Run navigation node
   python3 src/navigation/navigation_controller.py

   # Run sensor node
   python3 src/sensors/sensor_manager.py
   ```

7. **Run tests**:
   ```bash
   # Run all tests
   pytest tests/

   # Run specific test file
   pytest tests/test_ros_nodes.py

   # Run with verbose output
   pytest -v tests/
   ```

### Useful ROS 2 Commands

- **View node info**:
  ```bash
  ros2 node info /node_name
  ```

- **View topic info**:
  ```bash
  ros2 topic info /topic_name
  ```

- **View service list**:
  ```bash
  ros2 service list
  ```

- **Call a service**:
  ```bash
  ros2 service call /service_name service_type "{}"
  ```

- **View parameter list**:
  ```bash
  ros2 param list
  ```

- **Get parameter value**:
  ```bash
  ros2 param get /node_name parameter_name
  ```

### Development Tips

1. **Source your workspace**:
   ```bash
   source install/setup.bash
   ```

2. **Build your workspace**:
   ```bash
   colcon build
   ```

3. **View build logs**:
   ```bash
   colcon build --event-handlers console_direct+
   ```

4. **Clean build**:
   ```bash
   rm -rf build/ install/ log/
   colcon build
   ```

### Troubleshooting

1. **If ROS 2 commands are not found**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. **If your nodes are not found**:
   ```bash
   source install/setup.bash
   ```

3. **If you get permission errors**:
   ```bash
   # Check if you're running as rosuser
   whoami
   # Should show: rosuser
   ```

4. **To restart the container**:
   ```bash
   # From host machine
   docker-compose restart
   ``` 
# Logs Directory

This directory contains comprehensive logging for the robot system, including camera images, movement data, QR code detections, and system events.

## Directory Structure

```
logs/
├── images/              # Camera images with timestamps
├── movements/           # Movement and navigation logs
├── qr_detections/       # QR code detection logs
├── system_events/       # System events and errors
└── README.md           # This file
```

## Log Types

### 1. Images (`images/`)

Camera images are automatically captured and saved with the following naming convention:
```
{camera_name}_{event_type}_{timestamp}.jpg
```

**Event Types:**
- `routine` - Regular camera captures
- `qr_detected` - Images captured when QR codes are detected
- `aruco_detected` - Images captured when ArUco markers are detected
- `obstacle` - Images captured when obstacles are detected
- `error` - Images captured during error conditions

**Metadata Files:**
Each image may have an associated metadata file with the same name but `.json` extension containing:
```json
{
  "timestamp": "20241220_143052_123",
  "camera_name": "front",
  "event_type": "qr_detected",
  "qr_data": "WARD_A_01",
  "position": [[x1, y1], [x2, y2], [x3, y3], [x4, y4]],
  "action_taken": "navigate_to_ward"
}
```

### 2. Movements (`movements/`)

Movement logs are saved as JSON files with session timestamps:
```
movements_{session_id}.json
```

**Content Structure:**
```json
[
  {
    "timestamp": "2024-12-20T14:30:52.123456",
    "linear_velocity": 0.5,
    "angular_velocity": 0.0,
    "position_x": 1.25,
    "position_y": 0.75,
    "heading": 1.57,
    "battery_level": 85.5,
    "mode": "delivery"
  }
]
```

### 3. QR Detections (`qr_detections/`)

QR code detection logs with associated actions:
```
qr_detections_{session_id}.json
```

**Content Structure:**
```json
[
  {
    "timestamp": "2024-12-20T14:30:52.123456",
    "camera_name": "front",
    "qr_data": "WARD_A_01",
    "position": [[100, 100], [200, 100], [200, 200], [100, 200]],
    "image_filename": "front_qr_detected_20241220_143052_123.jpg",
    "action_taken": "navigate_to_ward"
  }
]
```

### 4. System Events (`system_events/`)

General system events, errors, and status changes:
```
system_events_{session_id}.json
```

**Content Structure:**
```json
[
  {
    "timestamp": "2024-12-20T14:30:52.123456",
    "event_type": "system_startup",
    "event_data": {
      "config": {...},
      "version": "1.0.0"
    },
    "severity": "info"
  }
]
```

**Event Types:**
- `system_startup` - System initialization
- `system_stop` - System shutdown
- `mode_change` - Operating mode changes
- `delivery_start` - Delivery task started
- `qr_handled` - QR code processed
- `low_battery` - Battery warning
- `error` - System errors
- `warning` - System warnings

**Severity Levels:**
- `debug` - Debug information
- `info` - General information
- `warning` - Warning conditions
- `error` - Error conditions
- `critical` - Critical errors

## Session Management

Each logging session is identified by a timestamp in the format `YYYYMMDD_HHMMSS`. All logs from a single robot operation session share the same session ID.

## Log Rotation

- Movement logs are written every 10 entries or 5 seconds (whichever comes first)
- QR detection logs are written immediately when QR codes are detected
- System event logs are written immediately for errors/warnings, or every 5 entries for info/debug
- All logs are force-saved during system cleanup

## Usage Examples

### Programmatic Access

```python
from src.logs.logger_manager import LoggerManager

# Initialize logger
logger = LoggerManager(base_log_dir="logs")

# Log camera image
logger.log_camera_image("front", camera_frame, "qr_detected", 
                       metadata={"qr_data": "WARD_A_01"})

# Log movement
logger.log_movement(0.5, 0.0, (1.25, 0.75), 1.57, 85.5, "delivery")

# Log QR detection
logger.log_qr_detection("front", "WARD_A_01", [[100,100], [200,200]], 
                       camera_frame, "navigate_to_ward")

# Log system event
logger.log_system_event("delivery_complete", 
                       {"ward": "A_01", "time_taken": 120}, "info")
```

### Analyzing Logs

```python
import json
import os

# Load movement logs
with open("logs/movements/movements_20241220_143052.json", "r") as f:
    movements = json.load(f)

# Find all QR detections
with open("logs/qr_detections/qr_detections_20241220_143052.json", "r") as f:
    qr_detections = json.load(f)

# Get session summary
summary = logger.get_session_summary()
print(f"Session: {summary['session_id']}")
print(f"Images: {summary['images_saved']}")
print(f"Movements: {summary['movement_logs_count']}")
```

## File Cleanup

Old log files should be periodically archived or deleted to manage disk space. Consider implementing a cleanup script that:

1. Compresses logs older than 30 days
2. Deletes logs older than 90 days
3. Maintains recent logs for debugging

## Integration

The logging system is automatically integrated into:
- `CameraManager` - Logs all camera captures and detections
- `MainController` - Logs system events and movements
- `SensorManager` - Can be extended to log sensor data
- `NavigationController` - Can log navigation decisions

All logging is non-blocking and designed to not interfere with real-time robot operations. 
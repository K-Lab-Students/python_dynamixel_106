import os
import json
import cv2
import logging
import asyncio
from datetime import datetime
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass, asdict
import numpy as np

@dataclass
class MovementLog:
    """Data class for movement logging."""
    timestamp: str
    linear_velocity: float
    angular_velocity: float
    position_x: float
    position_y: float
    heading: float
    battery_level: float
    mode: str

@dataclass
class QRDetectionLog:
    """Data class for QR code detection logging."""
    timestamp: str
    camera_name: str
    qr_data: str
    position: list
    image_filename: str
    action_taken: str

@dataclass
class SystemEventLog:
    """Data class for system events logging."""
    timestamp: str
    event_type: str
    event_data: Dict[str, Any]
    severity: str

class LoggerManager:
    """Manages all logging operations including images, movements, and events."""
    
    def __init__(self, base_log_dir: str = "logs"):
        """
        Initialize the logger manager.
        
        Args:
            base_log_dir: Base directory for all logs
        """
        self.base_log_dir = base_log_dir
        self.images_dir = os.path.join(base_log_dir, "images")
        self.movements_dir = os.path.join(base_log_dir, "movements")
        self.qr_detections_dir = os.path.join(base_log_dir, "qr_detections")
        self.system_events_dir = os.path.join(base_log_dir, "system_events")
        
        # Create directories if they don't exist
        self._create_directories()
        
        # Initialize file paths for current session
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.movement_log_file = os.path.join(self.movements_dir, f"movements_{self.session_id}.json")
        self.qr_log_file = os.path.join(self.qr_detections_dir, f"qr_detections_{self.session_id}.json")
        self.system_log_file = os.path.join(self.system_events_dir, f"system_events_{self.session_id}.json")
        
        # Initialize log lists
        self.movement_logs = []
        self.qr_logs = []
        self.system_logs = []
        
        # Setup logging
        self.logger = logging.getLogger(__name__)
        
    def _create_directories(self):
        """Create all necessary directories."""
        directories = [
            self.base_log_dir,
            self.images_dir,
            self.movements_dir,
            self.qr_detections_dir,
            self.system_events_dir
        ]
        
        for directory in directories:
            os.makedirs(directory, exist_ok=True)
            
    def log_camera_image(self, camera_name: str, frame: np.ndarray, 
                        event_type: str = "routine", metadata: Optional[Dict] = None) -> str:
        """
        Log a camera image with timestamp and metadata.
        
        Args:
            camera_name: Name of the camera ('front', 'side', 'opposite_side')
            frame: OpenCV image frame
            event_type: Type of event ('routine', 'qr_detected', 'obstacle', 'error')
            metadata: Additional metadata to store
            
        Returns:
            str: Filename of the saved image
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
        filename = f"{camera_name}_{event_type}_{timestamp}.jpg"
        filepath = os.path.join(self.images_dir, filename)
        
        # Save image
        cv2.imwrite(filepath, frame)
        
        # Save metadata if provided
        if metadata:
            metadata_file = filepath.replace('.jpg', '_metadata.json')
            metadata['timestamp'] = timestamp
            metadata['camera_name'] = camera_name
            metadata['event_type'] = event_type
            
            with open(metadata_file, 'w') as f:
                json.dump(metadata, f, indent=2)
                
        self.logger.info(f"Camera image logged: {filename}")
        return filename
    
    def log_movement(self, linear_vel: float, angular_vel: float, 
                    position: Tuple[float, float], heading: float,
                    battery_level: float, mode: str):
        """
        Log movement data.
        
        Args:
            linear_vel: Linear velocity in m/s
            angular_vel: Angular velocity in rad/s
            position: (x, y) position in meters
            heading: Heading angle in radians
            battery_level: Battery level percentage
            mode: Current robot mode
        """
        timestamp = datetime.now().isoformat()
        
        movement_log = MovementLog(
            timestamp=timestamp,
            linear_velocity=linear_vel,
            angular_velocity=angular_vel,
            position_x=position[0],
            position_y=position[1],
            heading=heading,
            battery_level=battery_level,
            mode=mode
        )
        
        self.movement_logs.append(movement_log)
        
        # Write to file every 10 entries or immediately for important events
        if len(self.movement_logs) % 10 == 0:
            self._save_movement_logs()
            
    def log_qr_detection(self, camera_name: str, qr_data: str, position: list,
                        frame: np.ndarray, action_taken: str = ""):
        """
        Log QR code detection with image and metadata.
        
        Args:
            camera_name: Name of the camera that detected the QR
            qr_data: Decoded QR data
            position: QR code position in image
            frame: Camera frame containing the QR code
            action_taken: Action taken based on QR detection
        """
        timestamp = datetime.now().isoformat()
        
        # Save the image
        image_filename = self.log_camera_image(
            camera_name, frame, "qr_detected",
            metadata={
                "qr_data": qr_data,
                "position": position,
                "action_taken": action_taken
            }
        )
        
        # Log QR detection
        qr_log = QRDetectionLog(
            timestamp=timestamp,
            camera_name=camera_name,
            qr_data=qr_data,
            position=position,
            image_filename=image_filename,
            action_taken=action_taken
        )
        
        self.qr_logs.append(qr_log)
        self._save_qr_logs()
        
        self.logger.info(f"QR detection logged: {qr_data} from {camera_name}")
        
    def log_system_event(self, event_type: str, event_data: Dict[str, Any], 
                        severity: str = "info"):
        """
        Log system events.
        
        Args:
            event_type: Type of event ('startup', 'shutdown', 'error', 'warning', 'task_start', etc.)
            event_data: Event-specific data
            severity: Event severity ('debug', 'info', 'warning', 'error', 'critical')
        """
        timestamp = datetime.now().isoformat()
        
        system_log = SystemEventLog(
            timestamp=timestamp,
            event_type=event_type,
            event_data=event_data,
            severity=severity
        )
        
        self.system_logs.append(system_log)
        
        # Save immediately for errors and warnings
        if severity in ['error', 'critical', 'warning']:
            self._save_system_logs()
        elif len(self.system_logs) % 5 == 0:
            self._save_system_logs()
            
        # Also log to standard logger
        log_level = getattr(logging, severity.upper(), logging.INFO)
        self.logger.log(log_level, f"System event: {event_type} - {event_data}")
        
    def _save_movement_logs(self):
        """Save movement logs to file."""
        try:
            with open(self.movement_log_file, 'w') as f:
                json.dump([asdict(log) for log in self.movement_logs], f, indent=2)
        except Exception as e:
            self.logger.error(f"Failed to save movement logs: {e}")
            
    def _save_qr_logs(self):
        """Save QR detection logs to file."""
        try:
            with open(self.qr_log_file, 'w') as f:
                json.dump([asdict(log) for log in self.qr_logs], f, indent=2)
        except Exception as e:
            self.logger.error(f"Failed to save QR logs: {e}")
            
    def _save_system_logs(self):
        """Save system event logs to file."""
        try:
            with open(self.system_log_file, 'w') as f:
                json.dump([asdict(log) for log in self.system_logs], f, indent=2)
        except Exception as e:
            self.logger.error(f"Failed to save system logs: {e}")
            
    def force_save_all(self):
        """Force save all logs to files."""
        self._save_movement_logs()
        self._save_qr_logs()
        self._save_system_logs()
        self.logger.info("All logs saved to files")
        
    def get_session_summary(self) -> Dict[str, Any]:
        """
        Get a summary of the current logging session.
        
        Returns:
            Dict containing session statistics
        """
        return {
            "session_id": self.session_id,
            "movement_logs_count": len(self.movement_logs),
            "qr_detections_count": len(self.qr_logs),
            "system_events_count": len(self.system_logs),
            "images_saved": len([f for f in os.listdir(self.images_dir) 
                               if f.startswith(f"{self.session_id.split('_')[0]}")])
        }
        
    async def cleanup(self):
        """Cleanup and save all remaining logs."""
        self.log_system_event("logger_cleanup", {"action": "saving_all_logs"})
        self.force_save_all()
        self.logger.info("Logger manager cleanup completed") 
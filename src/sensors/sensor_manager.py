from typing import Dict, List, Tuple, Optional
import numpy as np
import serial
import struct
from dataclasses import dataclass
from src.camera.camera_manager import CameraManager

@dataclass
class SensorData:
    """Container for all sensor data."""
    lidar_points: np.ndarray  # (N, 2) array of (x, y) points
    gyro_angle: float  # Current angle in radians
    camera_data: Dict[str, Optional[Tuple]]  # Camera detection results

class SensorManager:
    """Manages all sensors: LiDAR, gyroscope, and cameras."""
    
    def __init__(self, config: Dict):
        """
        Initialize the sensor manager.
        
        Args:
            config: Dictionary containing sensor configurations
        """
        # Initialize LiDAR
        self.lidar_port = config.get('lidar_port', '/dev/ttyUSB1')
        self.lidar_baud = config.get('lidar_baud', 115200)
        self.lidar_serial = None
        
        # Initialize cameras
        camera_configs = {
            'front': {'device_id': 0, 'resolution': (640, 480)},
            'side': {'device_id': 1, 'resolution': (640, 480)},
            'opposite_side': {'device_id': 2, 'resolution': (640, 480)}
        }
        self.camera_manager = CameraManager(camera_configs)
        
        # Initialize gyroscope (placeholder - implement actual gyro initialization)
        self.gyro_angle = 0.0
        
    def initialize_lidar(self) -> bool:
        """
        Initialize the LiDAR sensor.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            self.lidar_serial = serial.Serial(
                self.lidar_port,
                self.lidar_baud,
                timeout=1.0
            )
            self.lidar_serial.reset_input_buffer()
            return True
        except Exception as e:
            print(f"Failed to initialize LiDAR: {e}")
            return False
            
    def read_lidar_data(self) -> Optional[np.ndarray]:
        """
        Read and process LiDAR data.
        
        Returns:
            Optional[np.ndarray]: Array of (x, y) points if successful, None otherwise
        """
        if not self.lidar_serial:
            return None
            
        try:
            # Send scan command
            self.lidar_serial.write(b'\xA5\x20')
            
            # Read descriptor
            hdr = self.lidar_serial.read(7)
            if len(hdr) != 7 or hdr[0] != 0xA5 or hdr[1] != 0x5A:
                return None
                
            size = hdr[2]
            payload = self.lidar_serial.read(size)
            if len(payload) != size:
                return None
                
            # Read scan data
            nodes = []
            got_first = False
            
            while True:
                data = self.lidar_serial.read(5)
                if len(data) != 5:
                    break
                    
                b0, b1, b2, b3, b4 = struct.unpack('<BBBBB', data)
                if b0 & 0x1:  # New scan flag
                    if got_first:
                        break
                    got_first = True
                nodes.append((b0, b1, b2, b3, b4))
                
            # Convert to Cartesian coordinates
            xs, ys = [], []
            for b0, b1, b2, b3, b4 in nodes:
                quality = b0 >> 2
                raw_ang = ((b2 << 8) | b1) >> 1
                angle = raw_ang / 64.0  # degrees
                dist = ((b4 << 8) | b3) / 4.0  # mm
                
                if dist > 0:
                    rad = np.deg2rad(angle)
                    xs.append(dist * np.cos(rad))
                    ys.append(dist * np.sin(rad))
                    
            return np.column_stack((xs, ys))
            
        except Exception as e:
            print(f"Error reading LiDAR data: {e}")
            return None
            
    def update_gyro(self, angle: float):
        """
        Update gyroscope angle.
        
        Args:
            angle: New angle in radians
        """
        self.gyro_angle = angle
        
    def get_all_sensor_data(self) -> SensorData:
        """
        Get data from all sensors.
        
        Returns:
            SensorData object containing all sensor readings
        """
        # Get LiDAR data
        lidar_points = self.read_lidar_data()
        if lidar_points is None:
            lidar_points = np.array([])
            
        # Get camera data
        camera_data = {
            'front': self.camera_manager.detect_qr_code('front'),
            'side': self.camera_manager.detect_aruco('side'),
            'opposite_side': self.camera_manager.detect_aruco('opposite_side')
        }
        
        return SensorData(
            lidar_points=lidar_points,
            gyro_angle=self.gyro_angle,
            camera_data=camera_data
        )
        
    def cleanup(self):
        """Cleanup all sensor resources."""
        if self.lidar_serial:
            self.lidar_serial.close()
        self.camera_manager.release() 
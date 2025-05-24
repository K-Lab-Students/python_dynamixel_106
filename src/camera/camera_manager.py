from typing import Dict, Tuple, Optional
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from cv2 import aruco
from src.logs.logger_manager import LoggerManager

class CameraManager:
    """Manages all camera operations including QR and ArUco detection."""
    
    def __init__(self, camera_configs: Dict[str, Dict], logger_manager: Optional[LoggerManager] = None):
        """
        Initialize the camera manager with camera configurations.
        
        Args:
            camera_configs: Dictionary containing camera configurations
                {
                    'front': {'device_id': 0, 'resolution': (640, 480)},
                    'side': {'device_id': 1, 'resolution': (640, 480)},
                    'opposite_side': {'device_id': 2, 'resolution': (640, 480)}
                }
            logger_manager: Optional logger manager for logging images and events
        """
        self.cameras = {}
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        self.logger_manager = logger_manager
        
        for camera_name, config in camera_configs.items():
            self.cameras[camera_name] = cv2.VideoCapture(config['device_id'])
            self.cameras[camera_name].set(cv2.CAP_PROP_FRAME_WIDTH, config['resolution'][0])
            self.cameras[camera_name].set(cv2.CAP_PROP_FRAME_HEIGHT, config['resolution'][1])
    
    def detect_qr_code(self, camera_name: str, log_image: bool = True) -> Optional[Tuple[str, np.ndarray, np.ndarray]]:
        """
        Detect QR code from the specified camera.
        
        Args:
            camera_name: Name of the camera to use ('front', 'side', 'opposite_side')
            log_image: Whether to log the image when QR is detected
            
        Returns:
            Tuple of (decoded data, position matrix, image frame) if QR code found, None otherwise
        """
        if camera_name not in self.cameras:
            raise ValueError(f"Camera {camera_name} not found")
            
        ret, frame = self.cameras[camera_name].read()
        if not ret:
            return None
            
        decoded_objects = decode(frame)
        if not decoded_objects:
            return None
            
        # Get the first QR code found
        qr = decoded_objects[0]
        qr_data = qr.data.decode('utf-8')
        
        # Log QR detection if logger is available
        if self.logger_manager and log_image:
            self.logger_manager.log_qr_detection(
                camera_name=camera_name,
                qr_data=qr_data,
                position=qr.polygon.tolist(),
                frame=frame,
                action_taken="qr_detected"
            )
        
        return qr_data, qr.polygon, frame
    
    def detect_aruco(self, camera_name: str, log_image: bool = False) -> Optional[Tuple[int, np.ndarray, float, np.ndarray]]:
        """
        Detect ArUco marker from the specified camera.
        
        Args:
            camera_name: Name of the camera to use ('front', 'side', 'opposite_side')
            log_image: Whether to log the image when ArUco is detected
            
        Returns:
            Tuple of (marker ID, position matrix, distance, image frame) if marker found, None otherwise
        """
        if camera_name not in self.cameras:
            raise ValueError(f"Camera {camera_name} not found")
            
        ret, frame = self.cameras[camera_name].read()
        if not ret:
            return None
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        
        if ids is None or len(ids) == 0:
            return None
            
        # Calculate distance based on marker size (assuming 50mm marker)
        marker_size = 50  # mm
        camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]])  # Example values
        dist_coeffs = np.zeros(4)
        
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            corners[0], marker_size, camera_matrix, dist_coeffs
        )
        
        distance = np.linalg.norm(tvec[0][0])
        
        # Log ArUco detection if logger is available and requested
        if self.logger_manager and log_image:
            self.logger_manager.log_camera_image(
                camera_name=camera_name,
                frame=frame,
                event_type="aruco_detected",
                metadata={
                    "marker_id": int(ids[0][0]),
                    "position": tvec[0][0].tolist(),
                    "distance": float(distance)
                }
            )
        
        return ids[0][0], tvec[0][0], distance, frame

    def capture_routine_image(self, camera_name: str) -> Optional[np.ndarray]:
        """
        Capture a routine image from the specified camera and optionally log it.
        
        Args:
            camera_name: Name of the camera to use
            
        Returns:
            Image frame if successful, None otherwise
        """
        if camera_name not in self.cameras:
            raise ValueError(f"Camera {camera_name} not found")
            
        ret, frame = self.cameras[camera_name].read()
        if not ret:
            return None
            
        # Log routine image if logger is available
        if self.logger_manager:
            self.logger_manager.log_camera_image(
                camera_name=camera_name,
                frame=frame,
                event_type="routine"
            )
        
        return frame

    def calculate_alignment_correction(self, camera_name: str) -> Tuple[float, float]:
        """
        Calculate alignment correction values based on camera input.
        
        Args:
            camera_name: Name of the camera to use
            
        Returns:
            Tuple of (distance_correction, angle_correction)
        """
        marker_data = self.detect_aruco(camera_name)
        if marker_data is None:
            return 0.0, 0.0
            
        _, position, distance, _ = marker_data
        
        # Calculate corrections
        distance_correction = distance - 60  # Target distance is 60mm
        angle_correction = np.arctan2(position[0], position[2])  # Yaw angle
        
        return distance_correction, angle_correction
    
    def release(self):
        """Release all camera resources."""
        for camera in self.cameras.values():
            camera.release() 
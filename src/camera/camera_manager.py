from typing import Dict, Tuple, Optional
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from cv2 import aruco

class CameraManager:
    """Manages all camera operations including QR and ArUco detection."""
    
    def __init__(self, camera_configs: Dict[str, Dict]):
        """
        Initialize the camera manager with camera configurations.
        
        Args:
            camera_configs: Dictionary containing camera configurations
                {
                    'front': {'device_id': 0, 'resolution': (640, 480)},
                    'side': {'device_id': 1, 'resolution': (640, 480)},
                    'opposite_side': {'device_id': 2, 'resolution': (640, 480)}
                }
        """
        self.cameras = {}
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        
        for camera_name, config in camera_configs.items():
            self.cameras[camera_name] = cv2.VideoCapture(config['device_id'])
            self.cameras[camera_name].set(cv2.CAP_PROP_FRAME_WIDTH, config['resolution'][0])
            self.cameras[camera_name].set(cv2.CAP_PROP_FRAME_HEIGHT, config['resolution'][1])
    
    def detect_qr_code(self, camera_name: str) -> Optional[Tuple[str, np.ndarray]]:
        """
        Detect QR code from the specified camera.
        
        Args:
            camera_name: Name of the camera to use ('front', 'side', 'opposite_side')
            
        Returns:
            Tuple of (decoded data, position matrix) if QR code found, None otherwise
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
        return qr.data.decode('utf-8'), qr.polygon
    
    def detect_aruco(self, camera_name: str) -> Optional[Tuple[int, np.ndarray, float]]:
        """
        Detect ArUco marker from the specified camera.
        
        Args:
            camera_name: Name of the camera to use ('front', 'side', 'opposite_side')
            
        Returns:
            Tuple of (marker ID, position matrix, distance) if marker found, None otherwise
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
        
        return ids[0][0], tvec[0][0], np.linalg.norm(tvec[0][0])
    
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
            
        _, position, distance = marker_data
        
        # Calculate corrections
        distance_correction = distance - 60  # Target distance is 60mm
        angle_correction = np.arctan2(position[0], position[2])  # Yaw angle
        
        return distance_correction, angle_correction
    
    def release(self):
        """Release all camera resources."""
        for camera in self.cameras.values():
            camera.release() 
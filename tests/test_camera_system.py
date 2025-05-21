import pytest
import asyncio
from typing import Dict, Any, Optional, Tuple
import numpy as np
from src.sensors.camera_manager import CameraManager

@pytest.fixture
def camera_config() -> Dict[str, Any]:
    """Test configuration for camera system."""
    return {
        'front': {
            'device_id': 0,
            'resolution': [640, 480],
            'fps': 30
        },
        'side': {
            'device_id': 1,
            'resolution': [640, 480],
            'fps': 30
        },
        'opposite': {
            'device_id': 2,
            'resolution': [640, 480],
            'fps': 30
        }
    }

@pytest.fixture
async def camera_manager(camera_config):
    """Create and initialize a camera manager for testing."""
    manager = CameraManager(camera_config)
    await manager.initialize()
    yield manager
    await manager.cleanup()

@pytest.mark.asyncio
async def test_qr_code_detection_front(camera_manager):
    """Test QR code detection with front camera."""
    # Simulate QR code detection
    qr_data = await camera_manager.detect_qr_code('front')
    
    if qr_data is None:
        pytest.skip("No QR code detected in front camera")
        
    data, position = qr_data
    
    # Verify QR code data format
    assert isinstance(data, str), "QR code data should be a string"
    assert isinstance(position, np.ndarray), "Position should be a numpy array"
    assert position.shape == (4, 2), "Position should be a 4x2 array of points"
    
    # Verify QR code content format (medicine_id:ward1,ward2,ward3)
    try:
        medicine_id, wards = data.split(':')
        ward_list = wards.split(',')
        assert len(ward_list) > 0, "Should have at least one ward"
    except ValueError:
        pytest.fail("QR code data should be in format 'medicine_id:ward1,ward2,ward3'")

@pytest.mark.asyncio
async def test_aruco_detection_side(camera_manager):
    """Test ArUco marker detection with side camera."""
    # Simulate ArUco marker detection
    marker_data = await camera_manager.detect_aruco_marker('side')
    
    if marker_data is None:
        pytest.skip("No ArUco marker detected in side camera")
        
    marker_id, position, angle = marker_data
    
    # Verify marker data
    assert isinstance(marker_id, int), "Marker ID should be an integer"
    assert marker_id in [3, 5, 7], f"Unexpected marker ID: {marker_id}"
    assert isinstance(position, np.ndarray), "Position should be a numpy array"
    assert isinstance(angle, float), "Angle should be a float"
    assert -np.pi <= angle <= np.pi, "Angle should be in range [-pi, pi]"

@pytest.mark.asyncio
async def test_ward_qr_detection(camera_manager):
    """Test ward number QR code detection."""
    # Simulate ward QR code detection
    qr_data = await camera_manager.detect_qr_code('front')
    
    if qr_data is None:
        pytest.skip("No ward QR code detected")
        
    data, position = qr_data
    
    # Verify ward number format
    try:
        ward_number = int(data)
        assert 100 <= ward_number <= 999, f"Invalid ward number: {ward_number}"
    except ValueError:
        pytest.fail("Ward QR code should contain a number")

@pytest.mark.asyncio
async def test_camera_initialization(camera_manager):
    """Test camera initialization and status."""
    # Check if all cameras are initialized
    for camera_id in ['front', 'side', 'opposite']:
        assert camera_manager.is_camera_connected(camera_id), f"Camera {camera_id} not connected"
        
    # Check camera properties
    for camera_id in ['front', 'side', 'opposite']:
        frame = await camera_manager.get_frame(camera_id)
        assert frame is not None, f"Failed to get frame from {camera_id} camera"
        assert frame.shape[:2] == (480, 640), f"Invalid frame size for {camera_id} camera"

@pytest.mark.asyncio
async def test_camera_error_handling(camera_manager):
    """Test camera error handling."""
    # Test invalid camera ID
    with pytest.raises(ValueError):
        await camera_manager.get_frame('invalid_camera')
        
    # Test QR detection with invalid camera
    with pytest.raises(ValueError):
        await camera_manager.detect_qr_code('invalid_camera')
        
    # Test ArUco detection with invalid camera
    with pytest.raises(ValueError):
        await camera_manager.detect_aruco_marker('invalid_camera') 
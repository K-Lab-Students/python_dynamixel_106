import pytest
import asyncio
from typing import Dict, Any, List, Tuple
import numpy as np
from src.sensors.sensor_manager import SensorManager

@pytest.fixture
def sensor_config() -> Dict[str, Any]:
    """Test configuration for sensors."""
    return {
        'lidar': {
            'port': '/dev/ttyUSB1',
            'baud_rate': 115200,
            'scan_frequency': 10
        },
        'gyro': {
            'port': '/dev/ttyUSB0',
            'baud_rate': 9600,
            'update_rate': 100
        }
    }

@pytest.fixture
async def sensor_manager(sensor_config):
    """Create and initialize a sensor manager for testing."""
    manager = SensorManager(sensor_config)
    await manager.initialize()
    yield manager
    await manager.cleanup()

@pytest.mark.asyncio
async def test_lidar_initialization(sensor_manager):
    """Test LiDAR initialization and basic functionality."""
    # Check if LiDAR is connected
    assert sensor_manager.is_lidar_connected(), "LiDAR not connected"
    
    # Get point cloud
    point_cloud = await sensor_manager.get_lidar_data()
    
    # Verify point cloud format
    assert isinstance(point_cloud, np.ndarray), "Point cloud should be a numpy array"
    assert point_cloud.shape[1] == 2, "Point cloud should have 2D points"
    assert len(point_cloud) > 0, "Point cloud should not be empty"

@pytest.mark.asyncio
async def test_lidar_point_cloud_quality(sensor_manager):
    """Test LiDAR point cloud quality."""
    # Get multiple point clouds
    point_clouds = []
    for _ in range(5):
        cloud = await sensor_manager.get_lidar_data()
        point_clouds.append(cloud)
        await asyncio.sleep(0.1)
        
    # Check point cloud consistency
    for cloud in point_clouds:
        # Check point range
        distances = np.sqrt(np.sum(cloud**2, axis=1))
        assert np.all(distances > 0), "Points should have positive distance"
        assert np.all(distances < 10), "Points should be within 10 meters"
        
        # Check point distribution
        assert len(cloud) > 100, "Should have at least 100 points per scan"
        
        # Check for NaN values
        assert not np.any(np.isnan(cloud)), "Point cloud should not contain NaN values"

@pytest.mark.asyncio
async def test_imu_initialization(sensor_manager):
    """Test IMU initialization and basic functionality."""
    # Check if IMU is connected
    assert sensor_manager.is_gyro_connected(), "IMU not connected"
    
    # Get IMU data
    imu_data = await sensor_manager.get_gyro_data()
    
    # Verify IMU data format
    assert isinstance(imu_data, dict), "IMU data should be a dictionary"
    assert 'yaw' in imu_data, "IMU data should contain yaw angle"
    assert 'pitch' in imu_data, "IMU data should contain pitch angle"
    assert 'roll' in imu_data, "IMU data should contain roll angle"

@pytest.mark.asyncio
async def test_imu_angle_measurement(sensor_manager):
    """Test IMU angle measurements."""
    # Get multiple IMU readings
    angles = []
    for _ in range(10):
        imu_data = await sensor_manager.get_gyro_data()
        angles.append(imu_data['yaw'])
        await asyncio.sleep(0.1)
        
    # Check angle measurements
    for angle in angles:
        assert isinstance(angle, float), "Angle should be a float"
        assert -np.pi <= angle <= np.pi, "Angle should be in range [-pi, pi]"
        
    # Check for reasonable angle changes
    angle_changes = np.diff(angles)
    assert np.all(np.abs(angle_changes) < np.pi/2), "Angle changes should be reasonable"

@pytest.mark.asyncio
async def test_sensor_fusion(sensor_manager):
    """Test fusion of LiDAR and IMU data."""
    # Get combined sensor data
    sensor_data = await sensor_manager.get_all_sensor_data()
    
    # Verify data structure
    assert hasattr(sensor_data, 'lidar_data'), "Should have LiDAR data"
    assert hasattr(sensor_data, 'gyro_data'), "Should have IMU data"
    assert hasattr(sensor_data, 'camera_data'), "Should have camera data"
    
    # Check data consistency
    assert isinstance(sensor_data.lidar_data, np.ndarray), "LiDAR data should be numpy array"
    assert isinstance(sensor_data.gyro_data, dict), "IMU data should be dictionary"
    assert isinstance(sensor_data.camera_data, dict), "Camera data should be dictionary"

@pytest.mark.asyncio
async def test_sensor_error_handling(sensor_manager):
    """Test sensor error handling."""
    # Test invalid LiDAR port
    with pytest.raises(ValueError):
        await sensor_manager.initialize_lidar('/dev/invalid_port', 115200)
        
    # Test invalid IMU port
    with pytest.raises(ValueError):
        await sensor_manager.initialize_gyro('/dev/invalid_port', 9600)
        
    # Test sensor data with disconnected sensors
    sensor_manager.lidar_connected = False
    sensor_manager.gyro_connected = False
    
    sensor_data = await sensor_manager.get_all_sensor_data()
    assert sensor_data.lidar_data is None, "LiDAR data should be None when disconnected"
    assert sensor_data.gyro_data is None, "IMU data should be None when disconnected" 
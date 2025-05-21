import pytest
import asyncio
from typing import Dict, Any, List, Tuple
import numpy as np
from src.navigation.map_manager import MapManager, MapNode
from src.navigation.navigation_controller import NavigationController
from src.sensors.sensor_manager import SensorManager
from src.actuators.servo_controller import ServoController

@pytest.fixture
def test_map() -> Dict[str, Any]:
    """Create a test map configuration."""
    return {
        'nodes': [
            {
                'id': 'start',
                'position': [0.0, 0.0],
                'type': 'junction',
                'connections': ['shelf_1', 'ward_1']
            },
            {
                'id': 'shelf_1',
                'position': [1.0, 0.0],
                'type': 'shelf',
                'medicine_id': 'med_001',
                'connections': ['start', 'ward_1']
            },
            {
                'id': 'ward_1',
                'position': [0.0, 1.0],
                'type': 'ward',
                'qr_code': '101',
                'connections': ['start', 'shelf_1', 'ward_2']
            },
            {
                'id': 'ward_2',
                'position': [0.0, 2.0],
                'type': 'ward',
                'qr_code': '102',
                'connections': ['ward_1', 'ward_3']
            },
            {
                'id': 'ward_3',
                'position': [0.0, 3.0],
                'type': 'ward',
                'qr_code': '103',
                'connections': ['ward_2']
            }
        ]
    }

@pytest.fixture
async def navigation_system(test_map):
    """Create and initialize a navigation system for testing."""
    # Create components
    map_manager = MapManager(test_map)
    sensor_manager = SensorManager({
        'lidar': {'port': '/dev/ttyUSB1', 'baud_rate': 115200},
        'gyro': {'port': '/dev/ttyUSB0', 'baud_rate': 9600}
    })
    servo_controller = ServoController({
        'gripper': {'id': 1, 'min_angle': 0, 'max_angle': 180},
        'arm': {'id': 2, 'min_angle': 0, 'max_angle': 180}
    })
    
    # Initialize components
    await sensor_manager.initialize()
    await servo_controller.initialize()
    
    # Create navigation controller
    controller = NavigationController(map_manager, sensor_manager, servo_controller)
    
    yield controller
    
    # Cleanup
    await sensor_manager.cleanup()
    await servo_controller.cleanup()

@pytest.mark.asyncio
async def test_navigation_to_shelf(navigation_system):
    """Test navigation to medicine shelf."""
    # Start delivery task
    success = navigation_system.start_delivery_task('med_001', ['101', '102', '103'])
    assert success, "Failed to start delivery task"
    
    # Get initial state
    state = navigation_system.get_current_state()
    assert state.is_moving, "Robot should be moving"
    assert state.current_task == 'med_001', "Wrong medicine ID"
    assert state.current_ward == '101', "Wrong initial ward"
    
    # Simulate movement to shelf
    for _ in range(10):
        linear_vel, angular_vel = navigation_system.update()
        assert isinstance(linear_vel, float), "Linear velocity should be float"
        assert isinstance(angular_vel, float), "Angular velocity should be float"
        await asyncio.sleep(0.1)

@pytest.mark.asyncio
async def test_ward_navigation(navigation_system):
    """Test navigation between wards."""
    # Start with second ward
    success = navigation_system.start_delivery_task('med_001', ['102', '103'])
    assert success, "Failed to start delivery task"
    
    # Simulate ward navigation
    for _ in range(20):
        linear_vel, angular_vel = navigation_system.update()
        state = navigation_system.get_current_state()
        
        # Check velocity limits
        assert abs(linear_vel) <= 0.5, "Linear velocity exceeds limit"
        assert abs(angular_vel) <= 1.0, "Angular velocity exceeds limit"
        
        await asyncio.sleep(0.1)

@pytest.mark.asyncio
async def test_qr_code_handling(navigation_system):
    """Test QR code handling during navigation."""
    # Test ward QR code
    success = navigation_system.handle_qr_code('101')
    assert success, "Failed to handle ward QR code"
    
    state = navigation_system.get_current_state()
    assert state.current_ward == '101', "Wrong ward number"
    
    # Test medicine QR code
    success = navigation_system.handle_qr_code('med_001')
    assert success, "Failed to handle medicine QR code"
    
    state = navigation_system.get_current_state()
    assert state.current_task == 'med_001', "Wrong medicine ID"

@pytest.mark.asyncio
async def test_emergency_stop(navigation_system):
    """Test emergency stop functionality."""
    # Start a task
    navigation_system.start_delivery_task('med_001', ['101'])
    
    # Simulate some movement
    for _ in range(5):
        navigation_system.update()
        await asyncio.sleep(0.1)
    
    # Trigger emergency stop
    navigation_system.emergency_stop()
    
    # Check state
    state = navigation_system.get_current_state()
    assert not state.is_moving, "Robot should not be moving after emergency stop"
    assert state.current_task is None, "Task should be cleared after emergency stop"
    assert state.current_ward is None, "Ward should be cleared after emergency stop"

@pytest.mark.asyncio
async def test_error_handling(navigation_system):
    """Test navigation error handling."""
    # Test invalid medicine ID
    success = navigation_system.start_delivery_task('invalid_med', ['101'])
    assert not success, "Should fail with invalid medicine ID"
    
    # Test invalid ward sequence
    success = navigation_system.start_delivery_task('med_001', [])
    assert not success, "Should fail with empty ward sequence"
    
    # Test invalid QR code
    success = navigation_system.handle_qr_code('invalid_qr')
    assert not success, "Should fail with invalid QR code"
    
    # Check error state
    state = navigation_system.get_current_state()
    assert state.error_state is not None, "Should have error state" 
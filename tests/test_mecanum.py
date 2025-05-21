import pytest
import asyncio
from typing import Dict, Any, Tuple
import numpy as np
from src.actuators.mecanum_controller import MecanumController

@pytest.fixture
def mecanum_config() -> Dict[str, Any]:
    """Test configuration for Mecanum controller."""
    return {
        'wheel_radius': 0.05,  # meters
        'wheel_separation': 0.3,  # meters
        'max_linear_velocity': 0.5,  # m/s
        'max_angular_velocity': 1.0,  # rad/s
        'motor_ids': {
            'front_left': 1,
            'front_right': 2,
            'rear_left': 3,
            'rear_right': 4
        }
    }

@pytest.fixture
async def mecanum_controller(mecanum_config):
    """Create and initialize a Mecanum controller for testing."""
    controller = MecanumController(mecanum_config)
    await controller.initialize()
    yield controller
    await controller.cleanup()

@pytest.mark.asyncio
async def test_forward_movement(mecanum_controller):
    """Test forward movement."""
    # Set forward velocity
    await mecanum_controller.set_velocities(0.2, 0.0)  # 0.2 m/s forward
    
    # Get wheel velocities
    wheel_velocities = await mecanum_controller.get_wheel_velocities()
    
    # Verify all wheels move forward at same speed
    assert all(v > 0 for v in wheel_velocities.values()), "All wheels should move forward"
    assert abs(wheel_velocities['front_left'] - wheel_velocities['front_right']) < 0.01, "Front wheels should have same speed"
    assert abs(wheel_velocities['rear_left'] - wheel_velocities['rear_right']) < 0.01, "Rear wheels should have same speed"

@pytest.mark.asyncio
async def test_sideways_movement(mecanum_controller):
    """Test sideways movement."""
    # Set sideways velocity (using mecanum wheel properties)
    await mecanum_controller.set_velocities(0.0, 0.0, sideways_velocity=0.2)  # 0.2 m/s sideways
    
    # Get wheel velocities
    wheel_velocities = await mecanum_controller.get_wheel_velocities()
    
    # Verify wheels move in correct pattern for sideways movement
    assert wheel_velocities['front_left'] > 0, "Front left wheel should move forward"
    assert wheel_velocities['front_right'] < 0, "Front right wheel should move backward"
    assert wheel_velocities['rear_left'] < 0, "Rear left wheel should move backward"
    assert wheel_velocities['rear_right'] > 0, "Rear right wheel should move forward"

@pytest.mark.asyncio
async def test_rotation(mecanum_controller):
    """Test rotation movement."""
    # Set angular velocity
    await mecanum_controller.set_velocities(0.0, 0.5)  # 0.5 rad/s rotation
    
    # Get wheel velocities
    wheel_velocities = await mecanum_controller.get_wheel_velocities()
    
    # Verify wheels move in correct pattern for rotation
    assert wheel_velocities['front_left'] > 0, "Front left wheel should move forward"
    assert wheel_velocities['front_right'] < 0, "Front right wheel should move backward"
    assert wheel_velocities['rear_left'] > 0, "Rear left wheel should move forward"
    assert wheel_velocities['rear_right'] < 0, "Rear right wheel should move backward"

@pytest.mark.asyncio
async def test_combined_movement(mecanum_controller):
    """Test combined linear and angular movement."""
    # Set combined velocities
    await mecanum_controller.set_velocities(0.2, 0.3)  # 0.2 m/s forward, 0.3 rad/s rotation
    
    # Get wheel velocities
    wheel_velocities = await mecanum_controller.get_wheel_velocities()
    
    # Verify wheel velocities are within limits
    for velocity in wheel_velocities.values():
        assert abs(velocity) <= mecanum_controller.max_wheel_velocity, "Wheel velocity exceeds limit"

@pytest.mark.asyncio
async def test_velocity_limits(mecanum_controller):
    """Test velocity limit enforcement."""
    # Try to exceed linear velocity limit
    await mecanum_controller.set_velocities(1.0, 0.0)  # Try 1.0 m/s (limit is 0.5)
    
    # Get wheel velocities
    wheel_velocities = await mecanum_controller.get_wheel_velocities()
    
    # Verify velocities are limited
    for velocity in wheel_velocities.values():
        assert abs(velocity) <= mecanum_controller.max_wheel_velocity, "Wheel velocity exceeds limit"
        
    # Try to exceed angular velocity limit
    await mecanum_controller.set_velocities(0.0, 2.0)  # Try 2.0 rad/s (limit is 1.0)
    
    # Get wheel velocities
    wheel_velocities = await mecanum_controller.get_wheel_velocities()
    
    # Verify velocities are limited
    for velocity in wheel_velocities.values():
        assert abs(velocity) <= mecanum_controller.max_wheel_velocity, "Wheel velocity exceeds limit"

@pytest.mark.asyncio
async def test_emergency_stop(mecanum_controller):
    """Test emergency stop functionality."""
    # Set some velocity
    await mecanum_controller.set_velocities(0.2, 0.3)
    
    # Get initial wheel velocities
    initial_velocities = await mecanum_controller.get_wheel_velocities()
    
    # Trigger emergency stop
    await mecanum_controller.emergency_stop()
    
    # Get wheel velocities after stop
    final_velocities = await mecanum_controller.get_wheel_velocities()
    
    # Verify all wheels are stopped
    for velocity in final_velocities.values():
        assert abs(velocity) < 0.01, "Wheels should be stopped"

@pytest.mark.asyncio
async def test_odometry(mecanum_controller):
    """Test odometry calculation."""
    # Move forward for 1 second
    await mecanum_controller.set_velocities(0.2, 0.0)
    await asyncio.sleep(1.0)
    
    # Get odometry data
    odometry = await mecanum_controller.get_odometry()
    
    # Verify position and orientation
    assert abs(odometry['x'] - 0.2) < 0.05, "X position should be approximately 0.2m"
    assert abs(odometry['y']) < 0.05, "Y position should be approximately 0m"
    assert abs(odometry['theta']) < 0.05, "Orientation should be approximately 0rad" 
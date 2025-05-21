import pytest
import asyncio
from typing import Dict, Any
from src.actuators.servo_controller import ServoController

@pytest.fixture
def servo_config() -> Dict[str, Any]:
    """Test configuration for servo controller."""
    return {
        'gripper': {
            'id': 1,
            'min_angle': 0,
            'max_angle': 180,
            'default_position': 0
        },
        'arm': {
            'id': 2,
            'min_angle': 0,
            'max_angle': 180,
            'default_position': 90
        }
    }

@pytest.fixture
async def servo_controller(servo_config):
    """Create and initialize a servo controller for testing."""
    controller = ServoController(servo_config)
    await controller.initialize()
    yield controller
    await controller.cleanup()

@pytest.mark.asyncio
async def test_gripper_position_0(servo_controller):
    """Test setting gripper to position 0 (closed)."""
    # Set gripper to position 0
    await servo_controller.set_gripper_position(0)
    
    # Get current position
    current_pos = await servo_controller.get_gripper_position()
    
    # Verify position
    assert current_pos == 0, f"Expected gripper position 0, got {current_pos}"

@pytest.mark.asyncio
async def test_gripper_position_90(servo_controller):
    """Test setting gripper to position 90 (half open)."""
    # Set gripper to position 90
    await servo_controller.set_gripper_position(90)
    
    # Get current position
    current_pos = await servo_controller.get_gripper_position()
    
    # Verify position
    assert current_pos == 90, f"Expected gripper position 90, got {current_pos}"

@pytest.mark.asyncio
async def test_gripper_position_limits(servo_controller):
    """Test gripper position limits."""
    # Try to set position below minimum
    with pytest.raises(ValueError):
        await servo_controller.set_gripper_position(-10)
        
    # Try to set position above maximum
    with pytest.raises(ValueError):
        await servo_controller.set_gripper_position(190)
        
    # Verify current position hasn't changed
    current_pos = await servo_controller.get_gripper_position()
    assert 0 <= current_pos <= 180, f"Gripper position {current_pos} outside valid range"

@pytest.mark.asyncio
async def test_gripper_sequence(servo_controller):
    """Test a sequence of gripper movements."""
    # Test sequence: 0 -> 90 -> 180 -> 90 -> 0
    positions = [0, 90, 180, 90, 0]
    
    for pos in positions:
        await servo_controller.set_gripper_position(pos)
        current_pos = await servo_controller.get_gripper_position()
        assert current_pos == pos, f"Expected position {pos}, got {current_pos}"
        
        # Add small delay between movements
        await asyncio.sleep(0.5) 
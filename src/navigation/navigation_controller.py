from typing import Optional, Tuple, Dict
import numpy as np
from dataclasses import dataclass
from src.navigation.map_manager import MapManager
from src.navigation.task_executor import TaskExecutor
from src.sensors.sensor_manager import SensorManager
from src.actuators.servo_controller import ServoController

@dataclass
class NavigationState:
    """Represents the current state of navigation."""
    is_moving: bool = False
    current_task: Optional[str] = None
    current_ward: Optional[str] = None
    medicine_picked: bool = False
    error_state: Optional[str] = None

class NavigationController:
    """Controls high-level navigation and coordinates between components."""
    
    def __init__(
        self,
        map_manager: MapManager,
        sensor_manager: SensorManager,
        servo_controller: ServoController
    ):
        """
        Initialize the navigation controller.
        
        Args:
            map_manager: MapManager instance for map operations
            sensor_manager: SensorManager instance for sensor data
            servo_controller: ServoController instance for actuator control
        """
        self.map_manager = map_manager
        self.sensor_manager = sensor_manager
        self.servo_controller = servo_controller
        self.task_executor = TaskExecutor(map_manager, sensor_manager, servo_controller)
        self.state = NavigationState()
        
    def start_delivery_task(self, medicine_id: str, ward_sequence: list[str]) -> bool:
        """
        Start a new delivery task.
        
        Args:
            medicine_id: ID of the medicine to deliver
            ward_sequence: List of ward numbers to visit
            
        Returns:
            bool: True if task started successfully
        """
        try:
            self.state = NavigationState(
                is_moving=True,
                current_task=medicine_id,
                current_ward=ward_sequence[0] if ward_sequence else None
            )
            self.task_executor.start_task(medicine_id, ward_sequence)
            return True
        except Exception as e:
            self.state.error_state = f"Failed to start task: {str(e)}"
            return False
            
    def update(self) -> Tuple[float, float]:
        """
        Update navigation state and get movement commands.
        
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        try:
            # Get sensor data
            sensor_data = self.sensor_manager.get_all_sensor_data()
            
            # Execute one step of the current task
            linear_vel, angular_vel, task_complete = self.task_executor.execute_step()
            
            # Update state
            if task_complete:
                self.state.is_moving = False
                self.state.current_task = None
                self.state.current_ward = None
                self.state.medicine_picked = False
                
            return linear_vel, angular_vel
            
        except Exception as e:
            self.state.error_state = f"Navigation error: {str(e)}"
            return 0.0, 0.0
            
    def get_current_state(self) -> NavigationState:
        """
        Get the current navigation state.
        
        Returns:
            NavigationState object
        """
        return self.state
        
    def handle_qr_code(self, qr_data: str) -> bool:
        """
        Handle QR code detection.
        
        Args:
            qr_data: Content of the detected QR code
            
        Returns:
            bool: True if QR code was handled successfully
        """
        try:
            # Check if QR code is a ward number
            ward_node = self.map_manager.get_ward_by_qr(qr_data)
            if ward_node:
                self.state.current_ward = qr_data
                return True
                
            # Check if QR code is a medicine ID
            shelf_node = self.map_manager.get_shelf_by_medicine(qr_data)
            if shelf_node:
                self.state.current_task = qr_data
                return True
                
            return False
            
        except Exception as e:
            self.state.error_state = f"QR code handling error: {str(e)}"
            return False
            
    def emergency_stop(self):
        """Stop all movement and reset state."""
        self.state = NavigationState()
        self.servo_controller.set_gripper_position(0)  # Close gripper
        self.task_executor.current_task = None
        self.task_executor.current_path = [] 
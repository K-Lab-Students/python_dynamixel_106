import asyncio
from typing import Optional, Dict, Any
import logging
from dataclasses import dataclass
from src.navigation.map_manager import MapManager
from src.navigation.navigation_controller import NavigationController
from src.sensors.sensor_manager import SensorManager
from src.actuators.servo_controller import ServoController

@dataclass
class SystemState:
    """Represents the current state of the entire system."""
    is_initialized: bool = False
    is_running: bool = False
    current_mode: str = "idle"  # idle, mapping, delivery
    error_state: Optional[str] = None
    battery_level: float = 100.0
    sensor_status: Dict[str, bool] = None

class MainController:
    """Main controller coordinating all system components."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the main controller.
        
        Args:
            config: Configuration dictionary containing system parameters
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.state = SystemState()
        self.state.sensor_status = {
            'lidar': False,
            'gyro': False,
            'front_camera': False,
            'side_camera': False,
            'opposite_camera': False
        }
        
        # Initialize components
        self.sensor_manager = None
        self.servo_controller = None
        self.map_manager = None
        self.navigation_controller = None
        
    async def initialize(self) -> bool:
        """
        Initialize all system components.
        
        Returns:
            bool: True if initialization was successful
        """
        try:
            # Initialize sensor manager
            self.sensor_manager = SensorManager(self.config['sensors'])
            await self.sensor_manager.initialize()
            
            # Initialize servo controller
            self.servo_controller = ServoController(self.config['servos'])
            await self.servo_controller.initialize()
            
            # Initialize map manager
            self.map_manager = MapManager(self.config['map_file'])
            
            # Initialize navigation controller
            self.navigation_controller = NavigationController(
                self.map_manager,
                self.sensor_manager,
                self.servo_controller
            )
            
            # Update sensor status
            self._update_sensor_status()
            
            self.state.is_initialized = True
            self.logger.info("System initialized successfully")
            return True
            
        except Exception as e:
            self.state.error_state = f"Initialization error: {str(e)}"
            self.logger.error(f"Failed to initialize system: {str(e)}")
            return False
            
    async def start(self):
        """Start the main control loop."""
        if not self.state.is_initialized:
            self.logger.error("System not initialized")
            return
            
        self.state.is_running = True
        self.logger.info("Starting main control loop")
        
        try:
            while self.state.is_running:
                # Get sensor data
                sensor_data = await self.sensor_manager.get_all_sensor_data()
                
                # Update battery level (simulated)
                self.state.battery_level = max(0.0, self.state.battery_level - 0.01)
                
                # Handle QR codes if detected
                if sensor_data.camera_data['front']:
                    qr_data = sensor_data.camera_data['front'][0]
                    self.navigation_controller.handle_qr_code(qr_data)
                    
                # Update navigation
                linear_vel, angular_vel = self.navigation_controller.update()
                
                # Apply movement commands (to be implemented)
                # self.motor_controller.set_velocities(linear_vel, angular_vel)
                
                # Check for low battery
                if self.state.battery_level < 20.0:
                    self.logger.warning("Low battery, returning to charging station")
                    await self._return_to_charging()
                    
                await asyncio.sleep(0.1)  # 10Hz control loop
                
        except Exception as e:
            self.state.error_state = f"Runtime error: {str(e)}"
            self.logger.error(f"Error in main control loop: {str(e)}")
            
        finally:
            await self.cleanup()
            
    async def start_mapping(self):
        """Start the mapping mode."""
        if not self.state.is_initialized:
            return
            
        self.state.current_mode = "mapping"
        self.logger.info("Starting mapping mode")
        
        # TODO: Implement mapping logic
        
    async def start_delivery(self, medicine_id: str, ward_sequence: list[str]):
        """
        Start a delivery task.
        
        Args:
            medicine_id: ID of the medicine to deliver
            ward_sequence: List of ward numbers to visit
        """
        if not self.state.is_initialized:
            return
            
        self.state.current_mode = "delivery"
        self.logger.info(f"Starting delivery task: {medicine_id} -> {ward_sequence}")
        
        success = self.navigation_controller.start_delivery_task(medicine_id, ward_sequence)
        if not success:
            self.state.error_state = "Failed to start delivery task"
            
    async def stop(self):
        """Stop the system."""
        self.state.is_running = False
        self.logger.info("Stopping system")
        
    async def cleanup(self):
        """Clean up resources."""
        self.logger.info("Cleaning up resources")
        
        if self.sensor_manager:
            await self.sensor_manager.cleanup()
            
        if self.servo_controller:
            await self.servo_controller.cleanup()
            
        self.state.is_initialized = False
        self.state.is_running = False
        
    def _update_sensor_status(self):
        """Update the status of all sensors."""
        if not self.sensor_manager:
            return
            
        self.state.sensor_status['lidar'] = self.sensor_manager.is_lidar_connected()
        self.state.sensor_status['gyro'] = self.sensor_manager.is_gyro_connected()
        self.state.sensor_status['front_camera'] = self.sensor_manager.is_camera_connected('front')
        self.state.sensor_status['side_camera'] = self.sensor_manager.is_camera_connected('side')
        self.state.sensor_status['opposite_camera'] = self.sensor_manager.is_camera_connected('opposite')
        
    async def _return_to_charging(self):
        """Return to charging station."""
        self.logger.info("Returning to charging station")
        # TODO: Implement return to charging logic 
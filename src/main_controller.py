import asyncio
from typing import Optional, Dict, Any
import logging
from dataclasses import dataclass
from src.navigation.map_manager import MapManager
from src.navigation.navigation_controller import NavigationController
from src.sensors.sensor_manager import SensorManager
from src.actuators.servo_controller import ServoController
from src.logs.logger_manager import LoggerManager

@dataclass
class SystemState:
    """Represents the current state of the entire system."""
    is_initialized: bool = False
    is_running: bool = False
    current_mode: str = "idle"  # idle, mapping, delivery
    error_state: Optional[str] = None
    battery_level: float = 100.0
    sensor_status: Dict[str, bool] = None
    position: tuple = (0.0, 0.0)  # x, y position
    heading: float = 0.0  # heading in radians

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
        
        # Initialize logging system
        self.logger_manager = LoggerManager()
        
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
            # Log system startup
            self.logger_manager.log_system_event(
                "system_startup", 
                {"config": self.config}, 
                "info"
            )
            
            # Initialize sensor manager
            self.sensor_manager = SensorManager(self.config['sensors'], self.logger_manager)
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
            
            # Log successful initialization
            self.logger_manager.log_system_event(
                "initialization_complete",
                {"sensor_status": self.state.sensor_status},
                "info"
            )
            
            return True
            
        except Exception as e:
            self.state.error_state = f"Initialization error: {str(e)}"
            self.logger.error(f"Failed to initialize system: {str(e)}")
            
            # Log initialization error
            self.logger_manager.log_system_event(
                "initialization_error",
                {"error": str(e)},
                "error"
            )
            
            return False
            
    async def start(self):
        """Start the main control loop."""
        if not self.state.is_initialized:
            self.logger.error("System not initialized")
            return
            
        self.state.is_running = True
        self.logger.info("Starting main control loop")
        
        # Log system start
        self.logger_manager.log_system_event(
            "main_loop_start",
            {"mode": self.state.current_mode},
            "info"
        )
        
        try:
            loop_count = 0
            while self.state.is_running:
                # Get sensor data
                sensor_data = await self.sensor_manager.get_all_sensor_data()
                
                # Update battery level (simulated)
                self.state.battery_level = max(0.0, self.state.battery_level - 0.01)
                
                # Handle QR codes if detected
                if sensor_data.camera_data['front']:
                    qr_data = sensor_data.camera_data['front'][0]
                    self.navigation_controller.handle_qr_code(qr_data)
                    
                    # Log QR handling
                    self.logger_manager.log_system_event(
                        "qr_handled",
                        {"qr_data": qr_data, "camera": "front"},
                        "info"
                    )
                    
                # Update navigation
                linear_vel, angular_vel = self.navigation_controller.update()
                
                # Update position and heading (simplified - would come from actual sensors)
                self.state.position = (
                    self.state.position[0] + linear_vel * 0.1,  # 0.1s update rate
                    self.state.position[1]
                )
                self.state.heading += angular_vel * 0.1
                
                # Log movement every 50 iterations (5 seconds at 10Hz)
                if loop_count % 50 == 0:
                    self.logger_manager.log_movement(
                        linear_vel=linear_vel,
                        angular_vel=angular_vel,
                        position=self.state.position,
                        heading=self.state.heading,
                        battery_level=self.state.battery_level,
                        mode=self.state.current_mode
                    )
                
                # Apply movement commands (to be implemented)
                # self.motor_controller.set_velocities(linear_vel, angular_vel)
                
                # Check for low battery
                if self.state.battery_level < 20.0:
                    self.logger.warning("Low battery, returning to charging station")
                    
                    # Log low battery event
                    self.logger_manager.log_system_event(
                        "low_battery",
                        {"battery_level": self.state.battery_level},
                        "warning"
                    )
                    
                    await self._return_to_charging()
                    
                loop_count += 1
                await asyncio.sleep(0.1)  # 10Hz control loop
                
        except Exception as e:
            self.state.error_state = f"Runtime error: {str(e)}"
            self.logger.error(f"Error in main control loop: {str(e)}")
            
            # Log runtime error
            self.logger_manager.log_system_event(
                "runtime_error",
                {"error": str(e), "loop_count": loop_count},
                "error"
            )
            
        finally:
            await self.cleanup()
            
    async def start_mapping(self):
        """Start the mapping mode."""
        if not self.state.is_initialized:
            return
            
        self.state.current_mode = "mapping"
        self.logger.info("Starting mapping mode")
        
        # Log mode change
        self.logger_manager.log_system_event(
            "mode_change",
            {"new_mode": "mapping", "previous_mode": "idle"},
            "info"
        )
        
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
        
        # Log delivery start
        self.logger_manager.log_system_event(
            "delivery_start",
            {
                "medicine_id": medicine_id,
                "ward_sequence": ward_sequence,
                "start_position": self.state.position
            },
            "info"
        )
        
        success = self.navigation_controller.start_delivery_task(medicine_id, ward_sequence)
        if not success:
            self.state.error_state = "Failed to start delivery task"
            
            # Log delivery failure
            self.logger_manager.log_system_event(
                "delivery_start_failed",
                {"medicine_id": medicine_id, "ward_sequence": ward_sequence},
                "error"
            )
            
    async def stop(self):
        """Stop the system."""
        self.state.is_running = False
        self.logger.info("Stopping system")
        
        # Log system stop
        self.logger_manager.log_system_event(
            "system_stop",
            {"final_position": self.state.position, "battery_level": self.state.battery_level},
            "info"
        )
        
    async def cleanup(self):
        """Clean up resources."""
        self.logger.info("Cleaning up resources")
        
        # Log cleanup start
        self.logger_manager.log_system_event(
            "cleanup_start",
            {"final_state": self.state.__dict__},
            "info"
        )
        
        if self.sensor_manager:
            await self.sensor_manager.cleanup()
            
        if self.servo_controller:
            await self.servo_controller.cleanup()
            
        # Cleanup logger (save all remaining logs)
        await self.logger_manager.cleanup()
        
        self.state.is_initialized = False
        self.state.is_running = False
        
        # Final log
        self.logger_manager.log_system_event(
            "cleanup_complete",
            {"session_summary": self.logger_manager.get_session_summary()},
            "info"
        )
        
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
        
        # Log return to charging
        self.logger_manager.log_system_event(
            "return_to_charging",
            {"current_position": self.state.position, "battery_level": self.state.battery_level},
            "warning"
        )
        
        # TODO: Implement return to charging logic 
from typing import Dict, List, Optional
import os
from dotenv import load_dotenv
from dynamixel_sdk import *

class ServoController:
    """Controls Dynamixel servos for robotic arm movement."""
    
    def __init__(self):
        """Initialize the servo controller with configuration from .env file."""
        # Load configuration
        load_dotenv()
        
        self.config = {
            "device": os.getenv("DX_DEVICE", "/dev/ttyUSB1"),
            "baud": int(os.getenv("DX_BAUD", "57600")),
            "protocol": float(os.getenv("DX_PROTOCOL", "1.0")),
            "ids": list(map(int, os.getenv("DX_IDS", "2,7,8,9").split(","))),
            "invert": list(map(int, os.getenv("DX_INVERT", "1,-1,-1,1").split(","))),
            "speed_scale": float(os.getenv("DX_SPEED_SCALE", "500")),
        }
        
        # Initialize port handler and packet handler
        self.port_handler = PortHandler(self.config["device"])
        self.packet_handler = PacketHandler(self.config["protocol"])
        
        # Open port
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port {self.config['device']}")
            
        # Set port baudrate
        if not self.port_handler.setBaudRate(self.config["baud"]):
            raise RuntimeError(f"Failed to set baudrate {self.config['baud']}")
            
        # Initialize servos
        self.initialize_servos()
        
    def initialize_servos(self):
        """Initialize all servos with default settings."""
        for servo_id in self.config["ids"]:
            # Enable torque
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_TORQUE_ENABLE,
                1
            )
            
            # Set operating mode to position control
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_OPERATING_MODE,
                3  # Position control mode
            )
            
    def set_position(self, servo_id: int, position: float, speed: Optional[float] = None):
        """
        Set position for a specific servo.
        
        Args:
            servo_id: ID of the servo
            position: Target position in degrees (0-360)
            speed: Optional speed setting
        """
        if servo_id not in self.config["ids"]:
            raise ValueError(f"Invalid servo ID: {servo_id}")
            
        # Convert position to steps (assuming 4096 steps per revolution)
        position_steps = int(position * 4096 / 360)
        
        # Apply inversion if configured
        idx = self.config["ids"].index(servo_id)
        if self.config["invert"][idx] == -1:
            position_steps = 4096 - position_steps
            
        # Set speed if provided
        if speed is not None:
            speed_steps = int(speed * self.config["speed_scale"])
            self.packet_handler.write4ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_GOAL_VELOCITY,
                speed_steps
            )
            
        # Set position
        self.packet_handler.write4ByteTxRx(
            self.port_handler,
            servo_id,
            ADDR_GOAL_POSITION,
            position_steps
        )
        
    def get_position(self, servo_id: int) -> float:
        """
        Get current position of a servo.
        
        Args:
            servo_id: ID of the servo
            
        Returns:
            Current position in degrees
        """
        if servo_id not in self.config["ids"]:
            raise ValueError(f"Invalid servo ID: {servo_id}")
            
        # Read current position
        position_steps, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler,
            servo_id,
            ADDR_PRESENT_POSITION
        )
        
        # Convert steps to degrees
        position = position_steps * 360 / 4096
        
        # Apply inversion if configured
        idx = self.config["ids"].index(servo_id)
        if self.config["invert"][idx] == -1:
            position = 360 - position
            
        return position
        
    def set_gripper_position(self, position: float):
        """
        Set position for the gripper servo.
        
        Args:
            position: Target position in degrees (0-180)
        """
        # Assuming the last servo in the list is the gripper
        gripper_id = self.config["ids"][-1]
        self.set_position(gripper_id, position)
        
    def cleanup(self):
        """Cleanup servo resources."""
        for servo_id in self.config["ids"]:
            # Disable torque
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_TORQUE_ENABLE,
                0
            )
            
        # Close port
        self.port_handler.closePort() 
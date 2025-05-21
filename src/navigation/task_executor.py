from typing import List, Optional, Tuple
from dataclasses import dataclass
from src.navigation.map_manager import MapManager, MapNode
from src.sensors.sensor_manager import SensorManager
from src.actuators.servo_controller import ServoController

@dataclass
class DeliveryTask:
    """Represents a delivery task."""
    medicine_id: str
    ward_sequence: List[str]
    current_ward_index: int = 0
    medicine_picked: bool = False

class TaskExecutor:
    """Executes delivery tasks using the navigation system."""
    
    def __init__(self, map_manager: MapManager, sensor_manager: SensorManager, servo_controller: ServoController):
        """
        Initialize the task executor.
        
        Args:
            map_manager: MapManager instance
            sensor_manager: SensorManager instance
            servo_controller: ServoController instance
        """
        self.map_manager = map_manager
        self.sensor_manager = sensor_manager
        self.servo_controller = servo_controller
        self.current_task: Optional[DeliveryTask] = None
        self.current_path: List[str] = []
        self.current_path_index: int = 0
        
    def start_task(self, medicine_id: str, ward_sequence: List[str]):
        """
        Start a new delivery task.
        
        Args:
            medicine_id: ID of the medicine to deliver
            ward_sequence: List of ward numbers to visit
        """
        self.current_task = DeliveryTask(
            medicine_id=medicine_id,
            ward_sequence=ward_sequence
        )
        self._plan_next_movement()
        
    def execute_step(self) -> Tuple[float, float, bool]:
        """
        Execute one step of the current task.
        
        Returns:
            Tuple of (linear_velocity, angular_velocity, task_complete)
        """
        if not self.current_task:
            return 0.0, 0.0, True
            
        # Get current sensor data
        sensor_data = self.sensor_manager.get_all_sensor_data()
        
        # Update robot position based on LiDAR and gyro data
        self._update_position(sensor_data)
        
        # If we have a path to follow
        if self.current_path:
            return self._follow_path(sensor_data)
            
        # If we need to pick up medicine
        if not self.current_task.medicine_picked:
            return self._pickup_medicine(sensor_data)
            
        # If we need to deliver to a ward
        return self._deliver_to_ward(sensor_data)
        
    def _plan_next_movement(self):
        """Plan the next movement based on current task state."""
        if not self.current_task:
            return
            
        current_node = self.map_manager.find_nearest_node(
            self.map_manager.current_position
        )
        
        if not current_node:
            return
            
        if not self.current_task.medicine_picked:
            # Plan path to medicine shelf
            shelf_node = self.map_manager.get_shelf_by_medicine(
                self.current_task.medicine_id
            )
            if shelf_node:
                self.current_path = self.map_manager.find_path(
                    current_node.id,
                    shelf_node.id
                )
                self.current_path_index = 0
        else:
            # Plan path to next ward
            ward_number = self.current_task.ward_sequence[self.current_task.current_ward_index]
            ward_node = self.map_manager.get_ward_by_qr(ward_number)
            if ward_node:
                self.current_path = self.map_manager.find_path(
                    current_node.id,
                    ward_node.id
                )
                self.current_path_index = 0
                
    def _follow_path(self, sensor_data) -> Tuple[float, float, bool]:
        """
        Follow the current path.
        
        Returns:
            Tuple of (linear_velocity, angular_velocity, task_complete)
        """
        if self.current_path_index >= len(self.current_path):
            self.current_path = []
            return 0.0, 0.0, False
            
        target_node = self.map_manager.nodes[self.current_path[self.current_path_index]]
        
        # Calculate distance and angle to target
        dx = target_node.position[0] - self.map_manager.current_position[0]
        dy = target_node.position[1] - self.map_manager.current_position[1]
        target_angle = np.arctan2(dy, dx)
        angle_diff = target_angle - self.map_manager.current_heading
        
        # Normalize angle difference
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
            
        # Calculate velocities
        distance = np.sqrt(dx*dx + dy*dy)
        if distance < 50:  # 50mm threshold
            self.current_path_index += 1
            return 0.0, 0.0, False
            
        linear_vel = min(0.2, distance * 0.001)  # Scale down distance
        angular_vel = np.clip(angle_diff * 0.5, -0.5, 0.5)
        
        return linear_vel, angular_vel, False
        
    def _pickup_medicine(self, sensor_data) -> Tuple[float, float, bool]:
        """
        Handle medicine pickup.
        
        Returns:
            Tuple of (linear_velocity, angular_velocity, task_complete)
        """
        # Check if we're at the shelf
        shelf_node = self.map_manager.get_shelf_by_medicine(self.current_task.medicine_id)
        if not shelf_node:
            return 0.0, 0.0, True  # Error: shelf not found
            
        distance = np.sqrt(
            (shelf_node.position[0] - self.map_manager.current_position[0])**2 +
            (shelf_node.position[1] - self.map_manager.current_position[1])**2
        )
        
        if distance < 100:  # 100mm threshold
            # Align with shelf using side camera
            aruco_data = sensor_data.camera_data['side']
            if aruco_data:
                _, position, _ = aruco_data
                # Calculate alignment corrections
                linear_vel = np.clip(position[2] - 60, -0.1, 0.1)  # 60mm target distance
                angular_vel = np.clip(position[0] * 0.5, -0.5, 0.5)
                
                if abs(linear_vel) < 0.01 and abs(angular_vel) < 0.01:
                    # Aligned, pick up medicine
                    self.servo_controller.set_gripper_position(90)  # Open gripper
                    self.current_task.medicine_picked = True
                    self._plan_next_movement()
                    return 0.0, 0.0, False
                    
                return linear_vel, angular_vel, False
                
        # Not at shelf, plan path
        self._plan_next_movement()
        return 0.0, 0.0, False
        
    def _deliver_to_ward(self, sensor_data) -> Tuple[float, float, bool]:
        """
        Handle ward delivery.
        
        Returns:
            Tuple of (linear_velocity, angular_velocity, task_complete)
        """
        if self.current_task.current_ward_index >= len(self.current_task.ward_sequence):
            return 0.0, 0.0, True  # Task complete
            
        # Check if we're at the ward
        ward_number = self.current_task.ward_sequence[self.current_task.current_ward_index]
        ward_node = self.map_manager.get_ward_by_qr(ward_number)
        if not ward_node:
            return 0.0, 0.0, True  # Error: ward not found
            
        distance = np.sqrt(
            (ward_node.position[0] - self.map_manager.current_position[0])**2 +
            (ward_node.position[1] - self.map_manager.current_position[1])**2
        )
        
        if distance < 100:  # 100mm threshold
            # Check QR code with front camera
            qr_data = sensor_data.camera_data['front']
            if qr_data and qr_data[0] == ward_number:
                # At correct ward, mark as delivered
                self.current_task.current_ward_index += 1
                self._plan_next_movement()
                return 0.0, 0.0, False
                
        # Not at ward, plan path
        self._plan_next_movement()
        return 0.0, 0.0, False
        
    def _update_position(self, sensor_data):
        """Update robot position using sensor data."""
        # Use LiDAR data to update position
        if len(sensor_data.lidar_points) > 0:
            # Simple position update using nearest point
            nearest_point = sensor_data.lidar_points[0]
            self.map_manager.update_position(
                (nearest_point[0], nearest_point[1]),
                sensor_data.gyro_angle
            ) 
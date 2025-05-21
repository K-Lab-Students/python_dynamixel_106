from typing import Dict, List, Tuple, Optional
import numpy as np
from dataclasses import dataclass
import json
import os

@dataclass
class MapNode:
    """Represents a node in the navigation map."""
    id: str  # Unique identifier (e.g., "ward_101", "shelf_3")
    position: Tuple[float, float]  # (x, y) in mm
    type: str  # "ward", "shelf", "junction"
    qr_code: Optional[str] = None  # QR code content if present
    connections: List[str] = None  # List of connected node IDs

class MapManager:
    """Manages the labyrinth map and navigation."""
    
    def __init__(self, map_file: str = "labyrinth_map.json"):
        """
        Initialize the map manager.
        
        Args:
            map_file: Path to the map file
        """
        self.map_file = map_file
        self.nodes: Dict[str, MapNode] = {}
        self.current_position = (0.0, 0.0)
        self.current_heading = 0.0  # radians
        
        # Load existing map if available
        if os.path.exists(map_file):
            self.load_map()
            
    def add_node(self, node: MapNode):
        """
        Add a node to the map.
        
        Args:
            node: MapNode object to add
        """
        self.nodes[node.id] = node
        
    def update_node(self, node_id: str, **kwargs):
        """
        Update node properties.
        
        Args:
            node_id: ID of the node to update
            **kwargs: Properties to update
        """
        if node_id not in self.nodes:
            raise ValueError(f"Node {node_id} not found")
            
        node = self.nodes[node_id]
        for key, value in kwargs.items():
            setattr(node, key, value)
            
    def find_nearest_node(self, position: Tuple[float, float], node_type: Optional[str] = None) -> Optional[MapNode]:
        """
        Find the nearest node of a specific type.
        
        Args:
            position: Current (x, y) position
            node_type: Optional node type filter
            
        Returns:
            Nearest MapNode or None if not found
        """
        nearest_node = None
        min_distance = float('inf')
        
        for node in self.nodes.values():
            if node_type and node.type != node_type:
                continue
                
            distance = np.sqrt(
                (node.position[0] - position[0])**2 +
                (node.position[1] - position[1])**2
            )
            
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
                
        return nearest_node
        
    def find_path(self, start_id: str, goal_id: str) -> List[str]:
        """
        Find optimal path between two nodes using A* algorithm.
        
        Args:
            start_id: Starting node ID
            goal_id: Goal node ID
            
        Returns:
            List of node IDs forming the path
        """
        if start_id not in self.nodes or goal_id not in self.nodes:
            raise ValueError("Start or goal node not found")
            
        # A* implementation
        open_set = {start_id}
        closed_set = set()
        came_from = {}
        
        g_score = {start_id: 0}
        f_score = {start_id: self._heuristic(start_id, goal_id)}
        
        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            
            if current == goal_id:
                return self._reconstruct_path(came_from, current)
                
            open_set.remove(current)
            closed_set.add(current)
            
            for neighbor in self.nodes[current].connections:
                if neighbor in closed_set:
                    continue
                    
                tentative_g_score = g_score[current] + self._distance(current, neighbor)
                
                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif tentative_g_score >= g_score.get(neighbor, float('inf')):
                    continue
                    
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self._heuristic(neighbor, goal_id)
                
        return []  # No path found
        
    def _heuristic(self, node_id: str, goal_id: str) -> float:
        """Calculate heuristic (Euclidean distance) between nodes."""
        node = self.nodes[node_id]
        goal = self.nodes[goal_id]
        return np.sqrt(
            (node.position[0] - goal.position[0])**2 +
            (node.position[1] - goal.position[1])**2
        )
        
    def _distance(self, node1_id: str, node2_id: str) -> float:
        """Calculate actual distance between connected nodes."""
        node1 = self.nodes[node1_id]
        node2 = self.nodes[node2_id]
        return np.sqrt(
            (node1.position[0] - node2.position[0])**2 +
            (node1.position[1] - node2.position[1])**2
        )
        
    def _reconstruct_path(self, came_from: Dict[str, str], current: str) -> List[str]:
        """Reconstruct path from A* results."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
        
    def save_map(self):
        """Save the current map to file."""
        map_data = {
            node_id: {
                'position': node.position,
                'type': node.type,
                'qr_code': node.qr_code,
                'connections': node.connections
            }
            for node_id, node in self.nodes.items()
        }
        
        with open(self.map_file, 'w') as f:
            json.dump(map_data, f, indent=2)
            
    def load_map(self):
        """Load map from file."""
        with open(self.map_file, 'r') as f:
            map_data = json.load(f)
            
        for node_id, data in map_data.items():
            self.nodes[node_id] = MapNode(
                id=node_id,
                position=tuple(data['position']),
                type=data['type'],
                qr_code=data['qr_code'],
                connections=data['connections']
            )
            
    def update_position(self, position: Tuple[float, float], heading: float):
        """
        Update current robot position and heading.
        
        Args:
            position: New (x, y) position
            heading: New heading in radians
        """
        self.current_position = position
        self.current_heading = heading
        
    def get_ward_by_qr(self, qr_code: str) -> Optional[MapNode]:
        """
        Find ward node by QR code content.
        
        Args:
            qr_code: QR code content to search for
            
        Returns:
            MapNode of the ward or None if not found
        """
        for node in self.nodes.values():
            if node.type == 'ward' and node.qr_code == qr_code:
                return node
        return None
        
    def get_shelf_by_medicine(self, medicine_id: str) -> Optional[MapNode]:
        """
        Find shelf node by medicine ID.
        
        Args:
            medicine_id: Medicine ID to search for
            
        Returns:
            MapNode of the shelf or None if not found
        """
        for node in self.nodes.values():
            if node.type == 'shelf' and node.qr_code == medicine_id:
                return node
        return None 
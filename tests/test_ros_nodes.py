#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import time
import os
import sys
import asyncio
from typing import Dict, Any

# Add src directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

# Import your nodes
from main_controller import MainController, SystemState
# Import other nodes as needed
# from navigation import NavigationNode
# from actuators import ActuatorNode
# from sensors import SensorNode
# from camera import CameraNode

class TestROSNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.executor = SingleThreadedExecutor()
        cls.node = Node('test_node')
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        # Create test configuration
        self.test_config: Dict[str, Any] = {
            'sensors': {
                'lidar': {'port': '/dev/ttyUSB0', 'baudrate': 115200},
                'gyro': {'port': '/dev/ttyUSB1', 'baudrate': 9600},
                'cameras': {
                    'front': {'device_id': 0},
                    'side': {'device_id': 1},
                    'opposite': {'device_id': 2}
                }
            },
            'servos': {
                'port': '/dev/ttyUSB2',
                'baudrate': 1000000,
                'ids': [1, 2, 3]
            },
            'map_file': 'test_map.yaml'
        }

    def test_main_controller_initialization(self):
        """Test MainController initialization"""
        try:
            # Create MainController node
            main_controller = MainController(self.test_config)
            
            # Test initial state
            self.assertFalse(main_controller.state.is_initialized)
            self.assertFalse(main_controller.state.is_running)
            self.assertEqual(main_controller.state.current_mode, "idle")
            self.assertIsNone(main_controller.state.error_state)
            self.assertEqual(main_controller.state.battery_level, 100.0)
            
            # Test sensor status initialization
            self.assertIsNotNone(main_controller.state.sensor_status)
            self.assertFalse(main_controller.state.sensor_status['lidar'])
            self.assertFalse(main_controller.state.sensor_status['gyro'])
            self.assertFalse(main_controller.state.sensor_status['front_camera'])
            self.assertFalse(main_controller.state.sensor_status['side_camera'])
            self.assertFalse(main_controller.state.sensor_status['opposite_camera'])

        except Exception as e:
            self.fail(f"MainController initialization test failed: {str(e)}")

    def test_main_controller_async_operations(self):
        """Test MainController async operations"""
        async def run_async_tests():
            main_controller = MainController(self.test_config)
            
            # Test initialization
            init_success = await main_controller.initialize()
            self.assertTrue(init_success)
            self.assertTrue(main_controller.state.is_initialized)
            
            # Test start operation
            await main_controller.start()
            self.assertTrue(main_controller.state.is_running)
            
            # Test mapping mode
            await main_controller.start_mapping()
            self.assertEqual(main_controller.state.current_mode, "mapping")
            
            # Test delivery mode
            await main_controller.start_delivery("MED001", ["WARD1", "WARD2"])
            self.assertEqual(main_controller.state.current_mode, "delivery")
            
            # Test stop operation
            await main_controller.stop()
            self.assertFalse(main_controller.state.is_running)
            
            # Test cleanup
            await main_controller.cleanup()
            self.assertFalse(main_controller.state.is_initialized)
            self.assertFalse(main_controller.state.is_running)

        # Run async tests
        asyncio.run(run_async_tests())

    def test_main_controller_error_handling(self):
        """Test MainController error handling"""
        async def run_error_tests():
            # Test with invalid config
            invalid_config = {'sensors': None}
            main_controller = MainController(invalid_config)
            
            # Test initialization with invalid config
            init_success = await main_controller.initialize()
            self.assertFalse(init_success)
            self.assertIsNotNone(main_controller.state.error_state)
            
            # Test starting without initialization
            await main_controller.start()
            self.assertFalse(main_controller.state.is_running)

        # Run error tests
        asyncio.run(run_error_tests())

    def test_navigation_node(self):
        """Test Navigation node"""
        # Add navigation node tests
        pass

    def test_actuator_node(self):
        """Test Actuator node"""
        # Add actuator node tests
        pass

    def test_sensor_node(self):
        """Test Sensor node"""
        # Add sensor node tests
        pass

    def test_camera_node(self):
        """Test Camera node"""
        # Add camera node tests
        pass

def main():
    unittest.main()

if __name__ == '__main__':
    main() 
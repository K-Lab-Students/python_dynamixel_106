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
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String

# Add src directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

# Import your components
from actuators.servo_controller import ServoController
from camera.camera_manager import CameraManager
from sensors.sensor_manager import SensorManager
from navigation.navigation_controller import NavigationController

class TestRobotUnits(unittest.TestCase):
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
        self.test_config = {
            'sensors': {
                'lidar': {'port': '/dev/ttyUSB0', 'baudrate': 115200},
                'imu': {'port': '/dev/ttyUSB1', 'baudrate': 9600},
                'cameras': {
                    'front': {'device_id': 0},  # QR code at post
                    'side': {'device_id': 1},   # ArUco markers
                    'opposite': {'device_id': 2} # Room numbers
                }
            },
            'servos': {
                'port': '/dev/ttyUSB2',
                'baudrate': 1000000,
                'ids': [1, 2, 3]  # Servo IDs for gripper
            }
        }

    def test_servo_gripper(self):
        """Test servo gripper functionality"""
        async def run_gripper_test():
            servo = ServoController(self.test_config['servos'])
            await servo.initialize()

            # Test gripper open
            await servo.set_position(1, 0)  # Open position
            time.sleep(1)
            pos = await servo.get_position(1)
            self.assertAlmostEqual(pos, 0, delta=5)

            # Test gripper close
            await servo.set_position(1, 90)  # Close position
            time.sleep(1)
            pos = await servo.get_position(1)
            self.assertAlmostEqual(pos, 90, delta=5)

            await servo.cleanup()

        asyncio.run(run_gripper_test())

    def test_qr_code_detection(self):
        """Test QR code detection at post"""
        async def run_qr_test():
            camera = CameraManager(self.test_config['cameras'])
            await camera.initialize()

            # Test QR code detection
            qr_data = await camera.detect_qr_code('front')
            self.assertIsNotNone(qr_data)
            self.assertTrue('medicine_id' in qr_data)
            self.assertTrue('ward_sequence' in qr_data)

            await camera.cleanup()

        asyncio.run(run_qr_test())

    def test_room_number_detection(self):
        """Test room number detection"""
        async def run_room_test():
            camera = CameraManager(self.test_config['cameras'])
            await camera.initialize()

            # Test room number detection
            room_number = await camera.detect_room_number('opposite')
            self.assertIsNotNone(room_number)
            self.assertTrue(isinstance(room_number, str))
            self.assertTrue(room_number.isdigit())

            await camera.cleanup()

        asyncio.run(run_room_test())

    def test_aruco_marker_detection(self):
        """Test ArUco marker detection for sections 3/5/7"""
        async def run_aruco_test():
            camera = CameraManager(self.test_config['cameras'])
            await camera.initialize()

            # Test ArUco marker detection
            marker_data = await camera.detect_aruco_marker('side')
            self.assertIsNotNone(marker_data)
            self.assertTrue('id' in marker_data)
            self.assertTrue('position' in marker_data)
            self.assertTrue(marker_data['id'] in [3, 5, 7])

            await camera.cleanup()

        asyncio.run(run_aruco_test())

    def test_lidar_point_cloud(self):
        """Test LiDAR point cloud collection"""
        async def run_lidar_test():
            sensor = SensorManager(self.test_config['sensors'])
            await sensor.initialize()

            # Test LiDAR data collection
            point_cloud = await sensor.get_lidar_data()
            self.assertIsNotNone(point_cloud)
            self.assertTrue(len(point_cloud) > 0)
            self.assertTrue(isinstance(point_cloud, list))

            await sensor.cleanup()

        asyncio.run(run_lidar_test())

    def test_imu_data(self):
        """Test IMU data collection"""
        async def run_imu_test():
            sensor = SensorManager(self.test_config['sensors'])
            await sensor.initialize()

            # Test IMU data collection
            imu_data = await sensor.get_imu_data()
            self.assertIsNotNone(imu_data)
            self.assertTrue('orientation' in imu_data)
            self.assertTrue('angular_velocity' in imu_data)
            self.assertTrue('linear_acceleration' in imu_data)

            await sensor.cleanup()

        asyncio.run(run_imu_test())

    def test_mecanum_movement(self):
        """Test Mecanum wheel movement"""
        async def run_movement_test():
            nav = NavigationController(None, None, None)  # Dependencies not needed for basic movement
            await nav.initialize()

            # Test forward movement
            await nav.move_forward(0.5)  # 0.5 m/s
            time.sleep(2)
            await nav.stop()
            
            # Test sideways movement
            await nav.move_sideways(0.5)  # 0.5 m/s
            time.sleep(2)
            await nav.stop()

            # Test rotation
            await nav.rotate(0.5)  # 0.5 rad/s
            time.sleep(2)
            await nav.stop()

            await nav.cleanup()

        asyncio.run(run_movement_test())

def main():
    unittest.main()

if __name__ == '__main__':
    main() 
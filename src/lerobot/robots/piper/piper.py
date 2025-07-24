#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
import threading
from functools import cached_property
from typing import Any, Optional
import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_piper import PiperConfig

logger = logging.getLogger(__name__)

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    ROS2_AVAILABLE = True
except ImportError:
    logger.warning("ROS2 (rclpy) not available. Piper robot will use mock interface.")
    ROS2_AVAILABLE = False


class PiperVirtualMotorBus:
    """
    Virtual Motor Bus for Piper robot to maintain compatibility with LeRobot framework.
    
    This class provides the standard MotorsBus interface while using CAN communication internally.
    """
    
    def __init__(self, sdk_interface: 'PiperROS2Interface', joint_names: list[str], use_degrees: bool = False, calibration: dict = None):
        self.sdk = sdk_interface
        self.joint_names = joint_names
        self.calibration = calibration or {}
        
        # Create virtual motor objects for LeRobot compatibility
        norm_mode = MotorNormMode.DEGREES if use_degrees else MotorNormMode.RANGE_M100_100
        self.motors = {}
        
        for i, joint_name in enumerate(joint_names):
            # Create virtual motors (joint_7 is gripper with different normalization)
            if joint_name == "joint_7":  # gripper
                self.motors[joint_name] = Motor(i + 1, "virtual_piper", MotorNormMode.RANGE_0_100)
            else:
                self.motors[joint_name] = Motor(i + 1, "virtual_piper", norm_mode)
    
    @property
    def is_calibrated(self) -> bool:
        """Check if the virtual motor bus is calibrated. Always True for Piper."""
        return True  # Piper uses factory calibration
    
    @property
    def is_connected(self) -> bool:
        return self.sdk.is_connected
    
    def connect(self):
        """Connect through SDK interface."""
        return self.sdk.connect()
    
    def disconnect(self, disable_torque: bool = True):
        """Disconnect through SDK interface."""
        return self.sdk.disconnect()
    
    def sync_read(self, data_name: str, motor_names: list[str] = None) -> dict[str, float]:
        """Read data from all motors via SDK."""
        if not self.is_connected:
            raise DeviceNotConnectedError("Virtual motor bus not connected")
        
        motor_names = motor_names or list(self.motors.keys())
        joint_states = self.sdk.get_joint_states()
        
        if data_name == "Present_Position":
            positions = joint_states['positions']
            return {motor: float(positions[i]) for i, motor in enumerate(motor_names) if i < len(positions)}
        elif data_name == "Present_Velocity":
            velocities = joint_states['velocities']
            return {motor: float(velocities[i]) for i, motor in enumerate(motor_names) if i < len(velocities)}
        elif data_name == "Present_Load" or data_name == "Present_Current":
            torques = joint_states['torques']
            return {motor: float(torques[i]) for i, motor in enumerate(motor_names) if i < len(torques)}
        elif data_name == "Present_Temperature":
            # Mock temperature data (CAN interface doesn't provide this)
            return {motor: 35.0 for motor in motor_names}  # Safe temperature
        else:
            # Return zeros for unknown data types
            return {motor: 0.0 for motor in motor_names}
    
    def sync_write(self, data_name: str, name_value_dict: dict[str, float]):
        """Write data to motors via SDK."""
        if not self.is_connected:
            raise DeviceNotConnectedError("Virtual motor bus not connected")
        
        if data_name == "Goal_Position":
            # Convert to numpy array in joint order
            goal_positions = np.zeros(len(self.joint_names))
            for motor_name, value in name_value_dict.items():
                if motor_name in self.joint_names:
                    idx = self.joint_names.index(motor_name)
                    goal_positions[idx] = value
            
            self.sdk.send_joint_commands(goal_positions)
    
    def read(self, data_name: str, motor_name: str) -> float:
        """Read single motor data."""
        result = self.sync_read(data_name, [motor_name])
        return result.get(motor_name, 0.0)
    
    def write(self, data_name: str, motor_name: str, value: float):
        """Write single motor data."""
        self.sync_write(data_name, {motor_name: value})


class PiperROS2Interface:
    """
    Piper ROS2 interface for real-time joint state reading and action sending.
    
    This class supports two modes:
    1. RViz mode: subscribes to ROS2 topics from start_single_piper_rviz.launch.py
    2. Direct mode: connects to joint_state_loop_pub.py TCP server + ROS2 topics
    """
    
    def __init__(self, can_interface: str, sdk_config_path: Optional[str] = None, use_direct_mode: bool = True):
        self.can_interface = can_interface
        self.sdk_config_path = sdk_config_path
        self.use_direct_mode = use_direct_mode  # True: use joint_state_loop_pub.py, False: use RViz launch
        self._is_connected = False
        self._ros2_node = None
        
        # Joint state cache (7 joints for LeRobot: joint1-joint7)
        self._joint_positions = np.zeros(7)
        self._joint_velocities = np.zeros(7)
        self._joint_torques = np.zeros(7)
        
        # ROS2 related
        self._joint_states_subscriber = None
        self._joint_ctrl_publisher = None
        
        # Threading for ROS2 spinning
        self._stop_spinning = threading.Event()
        self._spin_thread = None
        
        # Safety monitoring
        self._last_communication_time = 0.0
        # Only use first 7 joints for LeRobot (joint8 is ignored)
        self._joint_names_ros2 = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
    def connect(self) -> None:
        """Connect to piper robot via ROS2 and optionally TCP."""
        try:
            if not ROS2_AVAILABLE:
                logger.warning("ROS2 not available, using mock interface")
                self._create_mock_interface()
                self._is_connected = True
                return
            
            # Initialize ROS2 if not already done
            if not rclpy.ok():
                rclpy.init()
            
            # Create ROS2 node
            self._ros2_node = rclpy.create_node('piper_lerobot_interface')
            logger.info("Created ROS2 node: piper_lerobot_interface")
            
            if self.use_direct_mode:
                # NEW: Direct mode - connect to joint_state_loop_pub.py
                logger.info("Using DIRECT mode: connecting to joint_state_loop_pub.py")
                self._connect_direct_mode()
            else:
                # OLD: RViz mode - subscribe to topics from start_single_piper_rviz.launch.py
                logger.info("Using RVIZ mode: subscribing to ROS2 topics from launch file")
                self._connect_rviz_mode()
            
            # Start ROS2 spinning in separate thread
            self._start_ros2_spinning()
            
            self._is_connected = True
            logger.info(f"Connected to Piper robot via ROS2 on {self.can_interface} ({'DIRECT' if self.use_direct_mode else 'RVIZ'} mode)")
            
        except Exception as e:
            logger.error(f"Failed to connect to Piper robot via ROS2: {e}")
            # Fallback to mock interface
            logger.warning("Falling back to mock interface")
            self._create_mock_interface()
            self._is_connected = True

    def _connect_direct_mode(self) -> None:
        """Connect in direct mode: receive data from joint_state_loop_pub.py via ROS2 topics."""
        # Subscribe to joint_states topic (published by joint_state_loop_pub.py)
        self._joint_states_subscriber = self._ros2_node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10  # QoS depth
        )
        
        # Create publisher for joint commands
        self._joint_ctrl_publisher = self._ros2_node.create_publisher(
            Float64MultiArray,
            '/joint_ctrl',
            10
        )
        
        # Note: TCP connection to joint_state_loop_pub.py is not needed
        # because we receive data via ROS2 topics that joint_state_loop_pub.py publishes
        logger.info("Direct mode: subscribing to /joint_states from joint_state_loop_pub.py")
        logger.info("Make sure joint_state_loop_pub.py is running to publish joint data")

    def _connect_rviz_mode(self) -> None:
        """Connect in RViz mode: subscribe to topics from start_single_piper_rviz.launch.py."""
        # LEGACY: Subscribe to joint_states topic from RViz launch file
        self._joint_states_subscriber = self._ros2_node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10  # QoS depth
        )
        
        # LEGACY: Create publisher for joint commands
        self._joint_ctrl_publisher = self._ros2_node.create_publisher(
            Float64MultiArray,
            '/joint_ctrl',
            10
        )
        
        logger.info("RViz mode connection established (requires start_single_piper_rviz.launch.py)")
    
    def _joint_states_callback(self, msg: 'JointState') -> None:
        """Callback for joint_states topic."""
        try:
            # Update joint positions from ROS2 message
            # Only process joint1~joint7 (ignore joint8)
            for i, joint_name in enumerate(msg.name):
                if joint_name in self._joint_names_ros2:
                    ros2_idx = self._joint_names_ros2.index(joint_name)
                    # Position
                    if i < len(msg.position):
                        self._joint_positions[ros2_idx] = msg.position[i]
                    # Velocity
                    if i < len(msg.velocity):
                        self._joint_velocities[ros2_idx] = msg.velocity[i]
                    # Effort (torque)
                    if i < len(msg.effort):
                        self._joint_torques[ros2_idx] = msg.effort[i]
            
            self._last_communication_time = time.time()
            
        except Exception as e:
            logger.error(f"Error in joint_states callback: {e}")
    
    def _start_ros2_spinning(self) -> None:
        """Start ROS2 spinning in a separate thread."""
        self._stop_spinning.clear()
        self._spin_thread = threading.Thread(target=self._ros2_spin_loop, daemon=True)
        self._spin_thread.start()
        logger.info("Started ROS2 spinning thread")
    
    def _ros2_spin_loop(self) -> None:
        """ROS2 spinning loop."""
        while not self._stop_spinning.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(self._ros2_node, timeout_sec=0.01)
            except Exception as e:
                logger.error(f"Error in ROS2 spin loop: {e}")
                time.sleep(0.1)
    
    def _create_mock_interface(self):
        """Create a mock interface for development/testing."""
        logger.info("Using mock interface for Piper robot")
        # Start a simple mock data generator
        self._start_mock_data_generation()
    
    def _start_mock_data_generation(self):
        """Start mock data generation in separate thread."""
        def mock_loop():
            while not self._stop_spinning.is_set():
                # Generate mock joint positions (slowly changing)
                t = time.time() * 0.1  # Slow oscillation
                self._joint_positions = np.array([
                    0.1 * np.sin(t),        # joint1
                    0.2 * np.sin(t + 1),    # joint2  
                    0.15 * np.sin(t + 2),   # joint3
                    0.1 * np.sin(t + 3),    # joint4
                    0.05 * np.sin(t + 4),   # joint5
                    0.1 * np.sin(t + 5),    # joint6
                    0.02 * np.sin(t + 6),   # joint7 (gripper)
                ])
                self._joint_velocities = np.random.uniform(-0.01, 0.01, 7)
                self._joint_torques = np.random.uniform(-0.1, 0.1, 7)
                self._last_communication_time = time.time()
                time.sleep(0.01)  # 100Hz
        
        self._stop_spinning.clear()
        self._spin_thread = threading.Thread(target=mock_loop, daemon=True)
        self._spin_thread.start()
    
    def get_joint_states(self) -> dict[str, np.ndarray]:
        """Get current joint states from ROS2 or mock interface."""
        if not self._is_connected:
            raise DeviceNotConnectedError("Piper ROS2 interface not connected")
        
        # Return 7 joints (joint1-joint7) for LeRobot
        return {
            'positions': self._joint_positions.copy(),
            'velocities': self._joint_velocities.copy(), 
            'torques': self._joint_torques.copy()
        }
    
    def send_joint_commands(self, target_positions: np.ndarray) -> None:
        """Send joint position commands via ROS2."""
        if not self._is_connected:
            raise DeviceNotConnectedError("Piper ROS2 interface not connected")
        
        try:
            if self._joint_ctrl_publisher is not None:
                # Create Float64MultiArray message
                msg = Float64MultiArray()
                msg.data = target_positions.tolist()
                
                # Publish joint commands
                self._joint_ctrl_publisher.publish(msg)
                logger.debug(f"Sent joint commands: {target_positions}")
            else:
                logger.warning("Joint control publisher not available")
                
        except Exception as e:
            logger.error(f"Error sending joint commands via ROS2: {e}")
            raise
    
    def emergency_stop(self) -> None:
        """Emergency stop the robot by sending zero velocities."""
        try:
            # Send current positions to stop movement
            current_positions = self._joint_positions.copy()  # Use all 7 joints
            self.send_joint_commands(current_positions)
            logger.warning("Emergency stop activated - sent current positions")
        except Exception as e:
            logger.error(f"Error in emergency stop: {e}")
    
    def disconnect(self) -> None:
        """Disconnect from the robot and cleanup ROS2 resources."""
        # Stop ROS2 spinning thread
        if self._spin_thread and self._spin_thread.is_alive():
            self._stop_spinning.set()
            self._spin_thread.join(timeout=2.0)
        
        # Cleanup ROS2 resources
        if self._ros2_node is not None:
            try:
                self._ros2_node.destroy_node()
            except Exception as e:
                logger.error(f"Error destroying ROS2 node: {e}")
        
        self._is_connected = False
        logger.info("Disconnected from Piper robot ROS2 interface")
    
    @property
    def is_connected(self) -> bool:
        return self._is_connected
    
    def check_communication_timeout(self, timeout: float) -> bool:
        """Check if communication has timed out."""
        return (time.time() - self._last_communication_time) > timeout


class Piper(Robot):
    """
    Piper robot arm implementation with CAN communication.
    
    This class provides control interface for the Piper robotic arm,
    featuring 7 joints controlled via CAN bus and piper_sdk.
    """

    config_class = PiperConfig
    name = "piper"

    def __init__(self, config: PiperConfig):
        super().__init__(config)
        self.config = config
        
        # Initialize Piper ROS2 interface
        self.sdk = PiperROS2Interface(
            can_interface=config.can_interface,
            sdk_config_path=config.sdk_config_path,
            use_direct_mode=config.use_direct_mode
        )
        
        # Initialize Virtual Motor Bus for LeRobot compatibility
        self.bus = PiperVirtualMotorBus(
            sdk_interface=self.sdk,
            joint_names=config.joint_names,
            use_degrees=config.use_degrees,
            calibration=self.calibration
        )
        
        # Initialize cameras
        self.cameras = make_cameras_from_configs(config.cameras)
        
        # Joint names mapping
        self.joint_names = config.joint_names
        
        # Safety monitoring
        self._last_observation_time = 0.0
        self._last_action_time = 0.0
        self._emergency_stop_triggered = False

    @property
    def _motors_ft(self) -> dict[str, type]:
        """Return motor/joint feature types for observation and action spaces."""
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Return camera feature types for observation space."""
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) 
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Return the structure of observations from this robot."""
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Return the structure of actions for this robot."""
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        """Check if robot and all cameras are connected."""
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        """Check if the robot is calibrated."""
        return self.bus.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        """
        Connect to the robot.
        
        Args:
            calibrate: Ignored for Piper robot (uses factory calibration).
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        # Connect to Piper SDK via Virtual Motor Bus
        self.bus.connect()
        
        # Connect cameras
        for cam in self.cameras.values():
            cam.connect()

        # Apply configuration
        self.configure()
        
        logger.info(f"{self} connected successfully.")

    def calibrate(self, limb_name: str = None) -> None:
        """
        Calibrate the robot. No-op for Piper robot as it uses factory calibration.
        
        Args:
            limb_name: Ignored.
        """
        logger.info(f"{self} uses factory calibration, no manual calibration needed.")

    def configure(self) -> None:
        """Configure robot parameters."""
        logger.info(f"Configuring {self}")
        
        # Log configuration
        logger.info(f"Joint read frequency: {self.config.joint_read_frequency} Hz")
        logger.info(f"Action frequency: {self.config.action_frequency} Hz")
        logger.info(f"Max joint velocity: {self.config.max_joint_velocity} rad/s")
        logger.info(f"Safety monitoring enabled: {self.config.enable_collision_detection}")
        
        logger.info(f"{self} configuration completed")

    def get_observation(self) -> dict[str, Any]:
        """
        Get current robot observation including joint positions and camera images.
        
        Returns:
            Dictionary containing joint positions and camera images.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # Check communication timeout
        if self.sdk.check_communication_timeout(self.config.communication_timeout):
            logger.error("Communication timeout detected!")
            self._trigger_emergency_stop()
            raise DeviceNotConnectedError("Communication timeout with robot")

        start_time = time.perf_counter()
        obs_dict = {}

        # Read motor positions via Virtual Motor Bus
        motor_start = time.perf_counter()
        positions = self.bus.sync_read("Present_Position")
        obs_dict.update({f"{motor}.pos": pos for motor, pos in positions.items()})
        motor_dt = (time.perf_counter() - motor_start) * 1000
        logger.debug(f"{self} read motor positions: {motor_dt:.1f}ms")

        # Read camera images
        for cam_key, cam in self.cameras.items():
            cam_start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            cam_dt = (time.perf_counter() - cam_start) * 1000
            logger.debug(f"{self} read {cam_key}: {cam_dt:.1f}ms")

        total_dt = (time.perf_counter() - start_time) * 1000
        logger.debug(f"{self} total observation time: {total_dt:.1f}ms")
        
        self._last_observation_time = time.perf_counter()
        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        Send action to the robot.
        
        Args:
            action: Dictionary containing joint position targets.
            
        Returns:
            Dictionary containing the actual joint targets sent (potentially clipped).
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        if self._emergency_stop_triggered:
            logger.warning("Emergency stop active, ignoring action")
            return {}

        start_time = time.perf_counter()
        
        # Handle keyboard teleoperator (empty action) - maintain current position
        if not action or all(v is None for v in action.values()):
            current_pos = self.bus.sync_read("Present_Position")
            return {f"{motor}.pos": pos for motor, pos in current_pos.items()}
        
        # Extract goal positions from action
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Apply safety constraints if configured
        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send commands to motors via Virtual Motor Bus
        self.bus.sync_write("Goal_Position", goal_pos)
        
        action_dt = (time.perf_counter() - start_time) * 1000
        logger.debug(f"{self} send action: {action_dt:.1f}ms")
        
        self._last_action_time = time.perf_counter()
        
        # Return the actual targets sent
        return {f"{motor}.pos": pos for motor, pos in goal_pos.items()}



    def _trigger_emergency_stop(self) -> None:
        """Trigger emergency stop."""
        logger.error("EMERGENCY STOP TRIGGERED")
        self._emergency_stop_triggered = True
        self.sdk.emergency_stop()

    def disconnect(self) -> None:
        """Disconnect from the robot and cameras."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # Disconnect Virtual Motor Bus (which disconnects SDK)
        self.bus.disconnect()
        
        # Disconnect cameras
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected")

    def get_status(self) -> dict[str, Any]:
        """
        Get detailed status information about the robot.
        
        Returns:
            Dictionary containing robot status information.
        """
        if not self.is_connected:
            return {"connected": False}

        try:
            # Read motor status via Virtual Motor Bus
            positions = self.bus.sync_read("Present_Position")
            velocities = self.bus.sync_read("Present_Velocity")
            loads = self.bus.sync_read("Present_Load")
            temperatures = self.bus.sync_read("Present_Temperature")
            
            status = {
                "connected": True,
                "calibrated": self.is_calibrated,
                "emergency_stop": self._emergency_stop_triggered,
                "motors": {
                    motor: {
                        "position": positions.get(motor, 0.0),
                        "velocity": velocities.get(motor, 0.0),
                        "load": loads.get(motor, 0.0),
                        "temperature": temperatures.get(motor, 0.0),
                    }
                    for motor in self.bus.motors
                },
                "cameras": {
                    cam_name: {"connected": cam.is_connected}
                    for cam_name, cam in self.cameras.items()
                },
                "communication_timeout": self.sdk.check_communication_timeout(self.config.communication_timeout),
                "last_observation_time": self._last_observation_time,
                "last_action_time": self._last_action_time,
            }
            
            return status
            
        except Exception as e:
            logger.error(f"Error getting status: {e}")
            return {"connected": True, "error": str(e)}

    def reset_emergency_stop(self) -> None:
        """Reset emergency stop state."""
        logger.info("Resetting emergency stop")
        self._emergency_stop_triggered = False 
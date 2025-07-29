#!/usr/bin/env python3

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

"""
Inference for Piper robot using ROS2 communication.

Example:
```shell
python -m lerobot.infer_piper \
    --policy_path=outputs/train/act_piper_xr_test/checkpoints/100000/pretrained_model \
    --dataset_repo_id=ujin/piper-direct-mode-test_xr_2 \
    --episode_time_s=30 \
    --fps=30
```
"""

import logging
import time
import subprocess
import signal
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from pprint import pformat
from typing import Dict, Any
import threading

import numpy as np
import torch
import cv2
from PIL import Image
import draccus

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("ROS2 (rclpy) not available. Please install ROS2 and rclpy.")

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.factory import make_policy
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.configs.policies import PreTrainedConfig
from lerobot.robots.piper.piper import Piper
from lerobot.robots.piper.config_piper import PiperConfig
from lerobot.utils.utils import (
    init_logging,
    log_say,
    get_safe_torch_device,
)
from lerobot.utils.control_utils import predict_action


@dataclass
class InferConfig:
    # Path to trained policy model
    policy_path: str | Path
    # Dataset repo_id to get features structure
    dataset_repo_id: str
    # Dataset root directory (optional)
    dataset_root: str | Path | None = None
    # Control frequency
    fps: int = 30
    # Total inference time in seconds
    episode_time_s: int | float = 30
    # Device
    device: str = "cuda"
    # Use amp
    use_amp: bool = False
    # Task description
    single_task: str = "Direct mode demonstration"
    # Display inference info
    verbose: bool = True


class PiperInferenceNode(Node):
    def __init__(self):
        super().__init__('piper_inference_node')
        
        # Joint command publisher (to /joint_states which is remapped from joint_ctrl_single)
        self.joint_publisher = self.create_publisher(
            JointState, 
            '/joint_states',  # This is where Piper expects commands
            10
        )
        
        # Joint state subscriber (from /joint_states_single where Piper publishes current state)
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states_single',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = np.zeros(7, dtype=np.float32)
        self.joint_state_lock = threading.Lock()
        self.last_joint_update = time.time()
        
        self.get_logger().info('Piper inference node initialized')
    
    def joint_state_callback(self, msg: JointState):
        """Callback for joint state updates from /joint_states_single"""
        with self.joint_state_lock:
            try:
                positions = np.zeros(7, dtype=np.float32)
                
                # Map joint positions: joint1-joint6 + gripper
                for i in range(6):  # First 6 joints
                    joint_name = f'joint{i+1}'
                    if joint_name in msg.name:
                        idx = msg.name.index(joint_name)
                        if idx < len(msg.position):
                            positions[i] = float(msg.position[idx])
                
                # Handle gripper as 7th joint
                if 'gripper' in msg.name:
                    idx = msg.name.index('gripper')
                    if idx < len(msg.position):
                        positions[6] = float(msg.position[idx])
                
                self.current_joint_positions = positions
                self.last_joint_update = time.time()
                
            except Exception as e:
                self.get_logger().warn(f'Error processing joint state: {e}')
    
    def get_current_joint_positions(self) -> np.ndarray:
        """Get current joint positions"""
        with self.joint_state_lock:
            return self.current_joint_positions.copy()
    
    def is_joint_state_fresh(self, timeout_s: float = 1.0) -> bool:
        """Check if joint state is recent"""
        return (time.time() - self.last_joint_update) < timeout_s
    
    def publish_joint_commands(self, joint_positions: list[float]):
        """Publish joint commands to /joint_states (remapped to joint_ctrl_single)"""
        if len(joint_positions) != 7:
            self.get_logger().error(f'Expected 7 joint positions, got {len(joint_positions)}')
            return
        
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        msg.position = joint_positions
        
        self.joint_publisher.publish(msg)
        
        if self.get_logger().get_effective_level() <= 10:  # DEBUG level
            self.get_logger().debug(f'Published joint commands: {joint_positions}')


class PiperInferenceSystem:
    def __init__(self, cfg: InferConfig):
        self.cfg = cfg
        self.device = get_safe_torch_device(cfg.device)
        
        # Load dataset to get features
        self.dataset = LeRobotDataset(
            cfg.dataset_repo_id, 
            root=cfg.dataset_root,
            episodes=[0]  # Load just one episode to get features
        )
        
        # Load policy
        policy_config = PreTrainedConfig.from_pretrained(cfg.policy_path)
        self.policy = make_policy(policy_config, ds_meta=self.dataset.meta)
        self.policy.eval()
        
        # Initialize robot with cameras (required)
        robot_config = PiperConfig()
        # Use default camera configuration from PiperConfig
        from lerobot.robots.piper.config_piper import default_piper_cameras
        robot_config.cameras = default_piper_cameras()
        
        self.robot = Piper(robot_config)
        
        # ROS2 node for direct communication with Piper
        if ROS2_AVAILABLE:
            rclpy.init()
            self.ros_node = PiperInferenceNode()
            self.ros_thread = None
        else:
            logging.error("ROS2 not available! This is required for Piper inference.")
            raise RuntimeError("ROS2 is required for Piper inference")
        
        # Action names from dataset
        self.action_names = self.dataset.features["action"]["names"]
        logging.info(f"Action names: {self.action_names}")
        
        # Observation cache for action chunking
        self.action_queue = []
        self.action_chunk_idx = 0
        
    def start_ros_spinning(self):
        """Start ROS2 spinning in a separate thread"""
        if self.ros_node:
            self.ros_thread = threading.Thread(target=self._ros_spin_worker, daemon=True)
            self.ros_thread.start()
            logging.info("Started ROS2 spinning thread")
    
    def _ros_spin_worker(self):
        """ROS2 spinning worker function"""
        try:
            while rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.01)
        except Exception as e:
            logging.error(f"ROS2 spinning error: {e}")
    
    def connect(self):
        """Connect to robot cameras and start ROS2"""
        # Connect to cameras only (avoid ROS2 conflicts)
        if hasattr(self.robot, 'cameras') and self.robot.cameras:
            for camera in self.robot.cameras.values():
                camera.connect()
            logging.info("Connected to robot cameras")
        
        # Start our own ROS2 node
        if self.ros_node:
            self.start_ros_spinning()
            # Wait a bit for joint state messages to start coming in
            logging.info("Waiting for joint state messages...")
            time.sleep(3.0)
    
    def disconnect(self):
        """Disconnect from robot and cleanup"""
        # Disconnect cameras
        if hasattr(self.robot, 'cameras') and self.robot.cameras:
            for camera in self.robot.cameras.values():
                camera.disconnect()
        
        if ROS2_AVAILABLE and self.ros_node:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        
        logging.info("Disconnected and cleaned up")
    
    def get_observation(self) -> Dict[str, Any]:
        """Get current observation from robot"""
        observation = {}
        
        # Get camera images directly
        if hasattr(self.robot, 'cameras') and self.robot.cameras:
            observation["images"] = {}
            for camera_name, camera in self.robot.cameras.items():
                try:
                    image = camera.read()
                    observation["images"][camera_name] = image
                except Exception as e:
                    logging.warning(f"Failed to read from camera {camera_name}: {e}")
                    # Use a dummy image if camera fails
                    observation["images"][camera_name] = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            raise RuntimeError("No cameras available")
        
        # Get current joint positions from ROS2
        if self.ros_node and self.ros_node.is_joint_state_fresh():
            current_joints = self.ros_node.get_current_joint_positions()
            observation["state"] = current_joints
        else:
            # Fallback to zeros if no recent joint state
            observation["state"] = np.zeros(7, dtype=np.float32)
            if self.ros_node:
                logging.warning("No recent joint state received, using zeros")
        
        return observation
    
    def process_observation(self, observation: Dict[str, Any]) -> Dict[str, torch.Tensor]:
        """Process observation into model input format"""
        processed = {}
        
        # Process images
        if "images" in observation:
            for camera_name, image in observation["images"].items():
                # Convert to tensor and normalize
                if isinstance(image, np.ndarray):
                    # Convert BGR to RGB if needed and ensure correct format
                    if image.shape[-1] == 3:
                        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    
                    # Convert to PIL Image then to tensor
                    pil_image = Image.fromarray(image)
                    # Resize to expected size
                    pil_image = pil_image.resize((640, 480))
                    
                    # Convert to tensor and normalize to [0, 1]
                    image_tensor = torch.from_numpy(np.array(pil_image)).float() / 255.0
                    # Change from HWC to CHW
                    image_tensor = image_tensor.permute(2, 0, 1)
                    
                    processed[f"observation.images.{camera_name}"] = image_tensor.unsqueeze(0).to(self.device)
        
        # Process state
        if "state" in observation:
            state_tensor = torch.from_numpy(observation["state"]).float()
            processed["observation.state"] = state_tensor.unsqueeze(0).to(self.device)
        
        return processed
    
    def predict_action(self, observation_batch: Dict[str, torch.Tensor]) -> np.ndarray:
        """Predict action using the loaded policy"""
        with torch.no_grad():
            if self.cfg.use_amp:
                with torch.autocast(device_type=self.device.type):
                    action_tensor = self.policy.select_action(observation_batch)
            else:
                action_tensor = self.policy.select_action(observation_batch)
        
        # Convert to numpy and get first batch element
        if isinstance(action_tensor, torch.Tensor):
            action = action_tensor.cpu().numpy()
            if action.ndim > 1:
                action = action[0]  # Get first batch element
        else:
            action = np.array(action_tensor)
        
        return action
    
    def send_joint_commands(self, joint_positions: np.ndarray):
        """Send joint commands via ROS2"""
        joint_list = joint_positions.tolist()
        
        if self.ros_node:
            # Send via ROS2
            self.ros_node.publish_joint_commands(joint_list)
        else:
            logging.error("ROS2 node not available, cannot send joint commands")
    
    def run_inference(self):
        """Main inference loop"""
        logging.info("Starting Piper inference...")
        
        # Check if we're receiving joint states
        if not self.ros_node.is_joint_state_fresh(timeout_s=5.0):
            logging.error("No joint state messages received! Check if Piper ROS node is running.")
            return
        
        start_time = time.time()
        frame_count = 0
        
        try:
            while time.time() - start_time < self.cfg.episode_time_s:
                loop_start = time.time()
                
                # Get observation
                observation = self.get_observation()
                
                # Process observation
                observation_batch = self.process_observation(observation)
                
                # Predict action
                action = self.predict_action(observation_batch)
                
                # Send joint commands
                self.send_joint_commands(action)
                
                if self.cfg.verbose:
                    current_joints = observation["state"]
                    current_str = ', '.join([f'{pos:.4f}' for pos in current_joints])
                    action_str = ', '.join([f'{pos:.4f}' for pos in action])
                    logging.info(f"Frame {frame_count}: Current=[{current_str}] → Action=[{action_str}]")
                
                frame_count += 1
                
                # Maintain control frequency
                loop_time = time.time() - loop_start
                sleep_time = (1.0 / self.cfg.fps) - loop_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            logging.info("Inference interrupted by user")
        
        finally:
            total_time = time.time() - start_time
            avg_fps = frame_count / total_time if total_time > 0 else 0
            logging.info(f"Inference completed. Frames: {frame_count}, "
                        f"Time: {total_time:.2f}s, Avg FPS: {avg_fps:.2f}")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    logging.info("Inference interrupted by signal")
    sys.exit(0)


@draccus.wrap()
def infer_piper(cfg: InferConfig):
    """Main inference function"""
    init_logging()
    logging.info("Piper Inference Configuration:")
    logging.info(pformat(asdict(cfg)))
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize inference system
    inference_system = PiperInferenceSystem(cfg)
    
    try:
        # Connect to robot
        inference_system.connect()
        
        # Run inference
        inference_system.run_inference()
        
    finally:
        # Clean up
        inference_system.disconnect()


if __name__ == "__main__":
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 not available. Please install ROS2 and rclpy.")
        print("Run: source /opt/ros/humble/setup.bash")
        sys.exit(1)
    
    infer_piper() 
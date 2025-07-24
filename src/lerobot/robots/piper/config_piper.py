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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig

from ..config import RobotConfig


def default_piper_cameras() -> dict[str, CameraConfig]:
    """Default camera configuration for Piper robot."""
    return {
        # Wrist Intel RealSense camera
        "wrist": RealSenseCameraConfig(
            serial_number_or_name="243522071785",  # 실제 감지된 시리얼 번호 사용
            fps=30,
            width=640,
            height=480,
            # Wrist camera는 보통 180도 회전이 필요할 수 있음
            rotation=0,
        ),
        # Phone camera (external)
        "front": RealSenseCameraConfig(
            serial_number_or_name="112322075866",  # 실제 감지된 시리얼 번호 사용
            fps=30,
            width=640,
            height=480,
            rotation=0,
        ),
    }


@RobotConfig.register_subclass("piper")
@dataclass
class PiperConfig(RobotConfig):
    """Configuration for the Piper robot arm."""
    
    # CAN interface to connect to the arm (e.g., "can0")
    can_interface: str = "can0"

    # Piper SDK configuration
    sdk_config_path: str | None = None  # Path to piper SDK configuration file
    
    # Connection mode settings
    use_direct_mode: bool = True  # True: use joint_state_loop_pub.py, False: use start_single_piper_rviz.launch.py
    
    # Real-time joint reading frequency (Hz)
    joint_read_frequency: float = 100.0
    
    # Action sending frequency (Hz) 
    action_frequency: float = 50.0
    
    # Joint limits and safety parameters
    joint_limits: dict[str, list[float]] = field(default_factory=lambda: {
        "joint_1": [-2.618, 2.168],  # radians
        "joint_2": [0.000, 3.140],
        "joint_3": [-2.967, 0.000], 
        "joint_4": [-1.745, 1.745],
        "joint_5": [-1.220, 1.220],
        "joint_6": [-2.094, 2.094],
        "joint_7": [0.000, 0.035],  # gripper
    })
    
    # Maximum joint velocity (rad/s)
    max_joint_velocity: float = 2.0
    
    # Maximum joint acceleration (rad/s²)
    max_joint_acceleration: float = 5.0
    
    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all joints, or a list that is the same length as
    # the number of joints in your robot arm.
    max_relative_target: float | None = 0.5  # radians
    
    # Safety monitoring
    enable_collision_detection: bool = True
    max_joint_torque: float = 10.0  # Nm
    emergency_stop_acceleration: float = 20.0  # rad/s²
    
    # Communication timeout (seconds)
    communication_timeout: float = 1.0
    
    # Camera configuration
    cameras: dict[str, CameraConfig] = field(default_factory=default_piper_cameras)
    
    # Joint name mapping for the 7 joints
    joint_names: list[str] = field(default_factory=lambda: [
        "joint_1",  # Base rotation
        "joint_2",  # Shoulder pitch  
        "joint_3",  # Shoulder roll
        "joint_4",  # Elbow pitch
        "joint_5",  # Wrist pitch
        "joint_6",  # Wrist roll
        "joint_7",  # Gripper
    ])
    
    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False

    def __post_init__(self):
        super().__post_init__()
        
        # Validate joint configuration
        if len(self.joint_names) != 7:
            raise ValueError(f"Piper robot must have exactly 7 joints, got {len(self.joint_names)}")
        
        # Validate joint limits
        for joint_name in self.joint_names:
            if joint_name not in self.joint_limits:
                raise ValueError(f"Joint limits not specified for joint: {joint_name}")
            
            limits = self.joint_limits[joint_name]
            if len(limits) != 2 or limits[0] >= limits[1]:
                raise ValueError(f"Invalid joint limits for {joint_name}: {limits}")
        
        # Validate safety parameters
        if self.max_joint_velocity <= 0:
            raise ValueError(f"max_joint_velocity must be positive, got {self.max_joint_velocity}")
        
        if self.max_joint_acceleration <= 0:
            raise ValueError(f"max_joint_acceleration must be positive, got {self.max_joint_acceleration}")
        
        if self.max_joint_torque <= 0:
            raise ValueError(f"max_joint_torque must be positive, got {self.max_joint_torque}")
        
        if self.communication_timeout <= 0:
            raise ValueError(f"communication_timeout must be positive, got {self.communication_timeout}")
        
        # Validate frequencies
        if self.joint_read_frequency <= 0:
            raise ValueError(f"joint_read_frequency must be positive, got {self.joint_read_frequency}")
        
        if self.action_frequency <= 0:
            raise ValueError(f"action_frequency must be positive, got {self.action_frequency}")
        
        if self.action_frequency > self.joint_read_frequency:
            raise ValueError(f"action_frequency ({self.action_frequency}) cannot be higher than joint_read_frequency ({self.joint_read_frequency})") 
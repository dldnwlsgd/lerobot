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

import time
from dataclasses import dataclass
from typing import Any

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from .piper import Piper


@TeleoperatorConfig.register_subclass("piper_demonstration")
@dataclass
class PiperDemonstrationTeleopConfig(TeleoperatorConfig):
    """Configuration for Piper demonstration teleoperator."""
    pass


class PiperDemonstrationTeleop(Teleoperator):
    """
    Demonstration teleoperator for Piper robot.
    
    This teleoperator reads the current joint positions from the robot
    and returns them as actions, allowing for demonstration data collection
    where the robot is moved manually and those movements are recorded.
    """
    
    config_class = PiperDemonstrationTeleopConfig
    name = "piper_demonstration"
    
    def __init__(self, config: PiperDemonstrationTeleopConfig):
        super().__init__(config)
        self._robot: Piper | None = None
        self._is_connected = False
        
    @property
    def action_features(self) -> dict[str, type]:
        """Return action features matching Piper robot's joint structure."""
        return {
            "joint_1.pos": float,
            "joint_2.pos": float, 
            "joint_3.pos": float,
            "joint_4.pos": float,
            "joint_5.pos": float,
            "joint_6.pos": float,
            "joint_7.pos": float,
        }
    
    @property
    def feedback_features(self) -> dict[str, type]:
        """No feedback features needed for demonstration mode."""
        return {}
    
    @property
    def is_connected(self) -> bool:
        """Check if teleoperator is connected."""
        return self._is_connected
    
    @property
    def is_calibrated(self) -> bool:
        """Always calibrated for demonstration mode."""
        return True
    
    def connect(self, calibrate: bool = True) -> None:
        """Connect to demonstration mode."""
        self._is_connected = True
        
    def calibrate(self) -> None:
        """No calibration needed for demonstration mode."""
        pass
        
    def configure(self) -> None:
        """No configuration needed for demonstration mode."""
        pass
    
    def set_robot(self, robot: Piper) -> None:
        """Set the robot instance to read joint positions from."""
        self._robot = robot
        
    def get_action(self) -> dict[str, Any]:
        """
        Get current joint positions as actions.
        
        Returns:
            Dictionary with current joint positions as actions.
        """
        if not self._is_connected:
            raise RuntimeError("PiperDemonstrationTeleop is not connected")
            
        if self._robot is None:
            raise RuntimeError("Robot not set. Call set_robot() first.")
            
        if not self._robot.is_connected:
            raise RuntimeError("Robot is not connected")
        
        # Read current joint positions from robot
        try:
            current_positions = self._robot.bus.sync_read("Present_Position")
            return {f"{joint}.pos": pos for joint, pos in current_positions.items()}
        except Exception as e:
            # If reading fails, return zero positions as fallback
            return {f"joint_{i+1}.pos": 0.0 for i in range(7)}
    
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """No feedback needed for demonstration mode."""
        pass
        
    def disconnect(self) -> None:
        """Disconnect from demonstration mode."""
        self._is_connected = False
        self._robot = None 
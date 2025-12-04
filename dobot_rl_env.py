#!/usr/bin/env python3
"""
Simplified Gym Environment for Dobot Pick-and-Place RL Training in RViz
No ROS dependencies - kinematic simulation only
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np

class DobotPickPlaceEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        # State variables (simulated)
        self.current_joints = np.zeros(4)
        self.detected_objects = []
        self.step_count = 0
        self.max_steps = 100
        
        # Target information for current task
        self.target_object_idx = 0  # Which object to pick
        self.target_box = 0  # Which box to place in (0-3 for red/green/blue/yellow)
        self.target_stack_level = 0  # Stack level (0-2 for bottom/middle/top)
        
        # Action space: [pick_x, pick_y, place_x, place_y]
        self.action_space = spaces.Box(low=-0.5, high=0.5, shape=(4,), dtype=np.float32)
        
        # Observation space: flattened [joint_positions, object_positions, object_sizes, object_colors, target_box, stack_level]
        # Max 5 objects: joints(4) + objects(5*6) + target_info(2) = 36 dimensions
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(36,), dtype=np.float32)
        
        # Reward parameters
        self.pick_reward = 1.0
        self.place_reward = 10.0
        self.step_penalty = -0.1
        
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        
        # Reset joints to home position
        self.current_joints = np.zeros(4)
        
        # Simulate object detection (fixed objects for training)
        self.detected_objects = [
            {'position': [0.1, 0.0, 0.05], 'size': 0, 'color': 0},  # Small red
            {'position': [0.0, 0.1, 0.15], 'size': 1, 'color': 1},  # Medium green
            {'position': [-0.1, 0.0, 0.25], 'size': 2, 'color': 2},  # Large blue
            {'position': [0.15, 0.05, 0.10], 'size': 0, 'color': 3},  # Small yellow
            {'position': [-0.05, -0.1, 0.20], 'size': 1, 'color': 0}   # Medium red
        ]
        
        # Randomly select target for this episode
        self.target_object_idx = np.random.randint(len(self.detected_objects))
        self.target_box = np.random.randint(4)  # 4 boxes
        self.target_stack_level = np.random.randint(3)  # 3 levels
        
        obs = self._get_obs()
        return obs, {}
    
    def step(self, action):
        self.step_count += 1
        
        # Parse action: [pick_x, pick_y, place_x, place_y]
        pick_x, pick_y, place_x, place_y = action
        
        # Get target object
        target_obj = self.detected_objects[self.target_object_idx]
        
        # Find closest object to pick position
        picked_obj = None
        min_pick_dist = float('inf')
        for obj in self.detected_objects:
            dist = np.sqrt((obj['position'][0] - pick_x)**2 + (obj['position'][1] - pick_y)**2)
            if dist < min_pick_dist:
                min_pick_dist = dist
                picked_obj = obj
        
        # Calculate reward
        reward = 0.0
        
        if picked_obj is None or min_pick_dist > 0.1:
            # No object picked
            reward = -2.0
        else:
            # Object picked - check if it's the target
            if picked_obj['size'] == target_obj['size'] and picked_obj['color'] == target_obj['color']:
                reward += 5.0  # Correct object picked
            else:
                reward += 1.0  # Wrong object but picked something
            
            # Check placement
            # Box positions (simplified - in real scenario these would be calibrated)
            box_positions = {
                0: [0.2, 0.2],   # Red box
                1: [0.2, -0.2],  # Green box  
                2: [-0.2, -0.2], # Blue box
                3: [-0.2, 0.2]   # Yellow box
            }
            
            target_box_pos = box_positions[self.target_box]
            place_dist = np.sqrt((place_x - target_box_pos[0])**2 + (place_y - target_box_pos[1])**2)
            
            if place_dist < 0.1:  # Close to correct box
                reward += 5.0
                
                # Bonus for correct color matching
                if picked_obj['color'] == self.target_box:
                    reward += 3.0
                
                # Check stack level (higher levels get more reward)
                if self.target_stack_level == 2:  # Top level
                    reward += 2.0
                elif self.target_stack_level == 1:  # Middle level
                    reward += 1.0
                # Bottom level = 0 bonus
            else:
                reward -= place_dist  # Penalty for distance from correct box
        
        reward += self.step_penalty  # Step penalty
        terminated = self.step_count >= self.max_steps
        
        # Simulate joint movement
        self.current_joints = np.random.uniform(-0.5, 0.5, 4)
        
        obs = self._get_obs()
        return obs, reward, terminated, False, {}
    
    def _get_obs(self):
        # Flatten observation: [joints(4), object_data(5 objects * 6 features each), target_info(2)]
        obs = np.zeros(36, dtype=np.float32)
        obs[:4] = self.current_joints
        
        # Fill object data (up to 5 objects, pad with zeros)
        for i, obj in enumerate(self.detected_objects[:5]):
            start_idx = 4 + i * 6
            obs[start_idx:start_idx+3] = obj['position']  # x, y, z
            obs[start_idx+3] = obj['size']  # 0, 1, 2
            obs[start_idx+4] = obj['color']  # 0-3
            obs[start_idx+5] = 1.0  # existence flag
        
        # Add target information
        obs[34] = self.target_box  # Target box (0-3)
        obs[35] = self.target_stack_level  # Stack level (0-2)
        
        return obs
    
    def close(self):
        pass
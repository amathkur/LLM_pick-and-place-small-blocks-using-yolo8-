#!/usr/bin/env python3

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from dobot_rl_env import DobotPickPlaceEnv
import os

def main():
    # Create environment
    env = DobotPickPlaceEnv()
    
    # Check environment
    print("Checking environment...")
    check_env(env)
    print("Environment check passed!")
    
    # Create PPO model
    model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.001, n_steps=2048)
    
    # Train the model
    print("Starting training...")
    model.learn(total_timesteps=10000)
    
    # Save the model
    model_path = "dobot_rl_model"
    model.save(model_path)
    print(f"Model saved to {model_path}")
    
    # Test the trained model
    print("Testing trained model...")
    obs, _ = env.reset()
    for _ in range(10):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"Action: {action}, Reward: {reward}")
        if terminated or truncated:
            obs, _ = env.reset()

if __name__ == "__main__":
    main()
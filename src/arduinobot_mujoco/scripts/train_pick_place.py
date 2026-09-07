#!/usr/bin/env python3
from pathlib import Path
from stable_baselines3 import PPO
from arduinobot_mujoco.pick_place_env import ArduinobotPickPlaceEnv


def main():
    env = ArduinobotPickPlaceEnv()
    PPO("MlpPolicy", env, verbose=1).learn(total_timesteps=100_000).save(Path("pick_place_ppo"))
    env.close()


if __name__ == "__main__":
    main()

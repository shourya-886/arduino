#!/usr/bin/env python3
from pathlib import Path
from stable_baselines3 import PPO
from arduinobot_mujoco.pick_place_env import ArduinobotPickPlaceEnv


def main():
    env = ArduinobotPickPlaceEnv()
    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=100_000)
    output = Path("pick_place_ppo")
    model.save(output)
    print(f"Training complete. Saved model to {output.resolve()}.zip")
    env.close()


if __name__ == "__main__":
    main()

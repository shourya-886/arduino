#!/usr/bin/env python3
"""Evaluate a trained pick-and-place policy in the MuJoCo viewer."""

import argparse
import time
from pathlib import Path

from stable_baselines3 import PPO

from arduinobot_mujoco.pick_place_env import ArduinobotPickPlaceEnv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", nargs="?", default="pick_place_ppo.zip")
    parser.add_argument("--episodes", type=int, default=5)
    args = parser.parse_args()

    env = ArduinobotPickPlaceEnv(render_mode="human")
    model = PPO.load(Path(args.model), env=env)
    successes = 0

    for episode in range(args.episodes):
        observation, _ = env.reset(seed=episode)
        done = False
        total_reward = 0.0
        while not done:
            action, _ = model.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            env.render()
            time.sleep(0.002)
            done = terminated or truncated
        successes += int(info.get("success", False))
        print(f"Episode {episode + 1}: reward={total_reward:.2f}, success={info.get('success', False)}")

    print(f"Success rate: {successes}/{args.episodes}")
    env.close()


if __name__ == "__main__":
    main()

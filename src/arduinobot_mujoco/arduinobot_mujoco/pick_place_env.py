"""Gymnasium environment for Arduinobot cube pick-and-place."""

from pathlib import Path
import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces
from ament_index_python.packages import get_package_share_directory


class ArduinobotPickPlaceEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self, model_path=None, render_mode=None, episode_length=500):
        super().__init__()
        if model_path is None:
            try:
                model_path = Path(get_package_share_directory("arduinobot_mujoco")) / "urdf" / "my_robot_mujoco.xml"
            except Exception:
                model_path = Path(__file__).parents[2] / "urdf" / "my_robot_mujoco.xml"
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.render_mode, self.episode_length, self.step_count = render_mode, episode_length, 0
        self.viewer = None
        obj = mujoco.mjtObj
        self.gripper_site = mujoco.mj_name2id(self.model, obj.mjOBJ_SITE, "gripper_site")
        self.target_site = mujoco.mj_name2id(self.model, obj.mjOBJ_SITE, "place_target")
        self.cube_body = mujoco.mj_name2id(self.model, obj.mjOBJ_BODY, "cube")
        self.action_space = spaces.Box(-1.0, 1.0, shape=(self.model.nu,), dtype=np.float32)
        # 5 joint positions + 5 velocities + cube, target, and gripper xyz.
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(19,), dtype=np.float32)

    def _observation(self):
        mujoco.mj_forward(self.model, self.data)
        return np.concatenate((self.data.qpos[:5], self.data.qvel[:5], self.data.xpos[self.cube_body], self.data.site_xpos[self.target_site], self.data.site_xpos[self.gripper_site])).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:5] = self.np_random.uniform(-0.08, 0.08, 5)
        self.data.qpos[5:12] = [0.15, 0.15, 0.04, 1, 0, 0, 0]
        self.data.ctrl[:] = self.data.qpos[:self.model.nu]
        self.step_count = 0
        return self._observation(), {}

    def step(self, action):
        self.data.ctrl[:] = np.clip(action, -1, 1) * np.pi / 2
        for _ in range(5):
            mujoco.mj_step(self.model, self.data)
        self.step_count += 1
        obs = self._observation()
        cube, target = self.data.xpos[self.cube_body], self.data.site_xpos[self.target_site]
        grip = self.data.site_xpos[self.gripper_site]
        cube_target, grip_cube = np.linalg.norm(cube - target), np.linalg.norm(grip - cube)
        success = cube_target < 0.07 and cube[2] > 0.08
        reward = -cube_target - 0.25 * grip_cube + (10.0 if success else 0.0)
        return obs, float(reward), bool(success), self.step_count >= self.episode_length, {"success": bool(success)}

    def render(self):
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

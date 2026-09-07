"""Gymnasium environment for Arduinobot cube pick-and-place."""

from pathlib import Path
import gymnasium as gym
import mujoco
import mujoco.viewer
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
        # Start from a repeatable upright pose; randomize only the cube so the
        # first policy learns reaching rather than recovering from bad poses.
        self.data.qpos[:5] = 0.0
        cube_xy = self.np_random.uniform([0.10, 0.10], [0.20, 0.20])
        self.data.qpos[5:12] = [cube_xy[0], cube_xy[1], 0.04, 1, 0, 0, 0]
        self.data.ctrl[:] = self.data.qpos[:self.model.nu]
        self.step_count = 0
        return self._observation(), {}

    def step(self, action):
        # Actions are small joint-position increments, rather than abrupt
        # commands spanning the complete joint range.
        self.data.ctrl[:] = np.clip(self.data.qpos[:5] + np.clip(action, -1, 1) * 0.05, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])
        for _ in range(5):
            mujoco.mj_step(self.model, self.data)
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            return self._observation(), -100.0, False, True, {"success": False, "unstable": True}
        self.step_count += 1
        obs = self._observation()
        cube, target = self.data.xpos[self.cube_body], self.data.site_xpos[self.target_site]
        grip = self.data.site_xpos[self.gripper_site]
        cube_target = np.linalg.norm(cube - target)
        grip_cube = np.linalg.norm(grip - cube)
        grasped = grip_cube < 0.09
        lifted = cube[2] > 0.09
        success = cube_target < 0.07 and cube[2] > 0.10
        # Dense staged shaping: approach the cube, close the distance, lift,
        # then carry it to the target. The large terminal bonus emphasizes
        # actual placement over merely approaching the target.
        reward = -0.5 * grip_cube - 0.25 * cube_target
        if grasped:
            reward += 1.0
        if lifted:
            reward += 2.0
        if success:
            reward += 25.0
        return obs, float(reward), bool(success), self.step_count >= self.episode_length, {"success": bool(success)}

    def render(self):
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

# Arduinobot MuJoCo simulation

This ROS 2 `ament_cmake` package launches the Arduinobot MJCF model in
MuJoCo's native viewer. It currently provides visualization and physics only;
reinforcement-learning files and task objects are not part of this package.

## Directory layout

```text
arduinobot_mujoco/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/mujoco.launch.py
├── scripts/mujoco_sim.py
├── meshes/*.STL
└── urdf/my_robot_mujoco.xml
```

The model is in `urdf/my_robot_mujoco.xml`. It contains the nested fixed-base
robot kinematic tree, five joints, inertial properties, mesh assets, physics
settings, and position actuators. The XML follows a structured MJCF layout:
compiler/settings, defaults, assets, worldbody, and actuators.

The package-local mesh directory is:

```text
/home/shourya/arduinobot_ws/src/arduinobot_mujoco/meshes/
```

The XML uses `meshdir="../meshes"`, so `file="base_plate.STL"` resolves both
from the source tree and from the installed package.

## Requirements

Install MuJoCo into the Python environment used by ROS 2:

```bash
python3 -m pip install mujoco
```

ROS dependencies are declared in `package.xml`: `ament_cmake`,
`ament_index_python`, `launch`, and `launch_ros`.

## Build

```bash
cd /home/shourya/arduinobot_ws
colcon build --packages-select arduinobot_mujoco
source install/setup.bash
```

`--symlink-install` is optional. A normal build copies files into `install/`;
use it when you want to avoid source/install symlinks. Rebuild after changing
the XML, CMake install rules, launch files, or meshes.

## Launch

```bash
ros2 launch arduinobot_mujoco mujoco.launch.py
```

The default model is installed at:

```text
install/arduinobot_mujoco/share/arduinobot_mujoco/urdf/my_robot_mujoco.xml
```

Inspect launch arguments with:

```bash
ros2 launch arduinobot_mujoco mujoco.launch.py --show-args
```

Launch another model with:

```bash
ros2 launch arduinobot_mujoco mujoco.launch.py \
  model:=/absolute/path/to/model.xml
```

## Install mapping

`CMakeLists.txt` installs:

```text
urdf/          -> share/arduinobot_mujoco/urdf/
meshes/        -> share/arduinobot_mujoco/meshes/
launch/        -> share/arduinobot_mujoco/launch/
mujoco_sim.py  -> lib/arduinobot_mujoco/
```

## Troubleshooting

If MuJoCo cannot be imported, run `python3 -m pip install mujoco`. If an STL
cannot be opened, confirm it exists under `arduinobot_mujoco/meshes/`, rebuild,
and source `install/setup.bash` again. If the arm oscillates, tune the
`<option>` timestep/integrator and the `<actuator>` gains in the MJCF file.
A GLFW Wayland window-position warning is normally harmless if the viewer opens.

Check the installed package prefix with:

```bash
ros2 pkg prefix arduinobot_mujoco
```

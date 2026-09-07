#!/usr/bin/env python3

import os
import time
import numpy as np
import mujoco
import mujoco.viewer


# ============================================================
# CONFIGURATION
# ============================================================

XML_PATH = os.path.expanduser(
    "~/arduinobot_ws/src/arduinobot_rl/arduinobot_reach.xml"
)

SIMULATION_TIME = 30.0


# ============================================================
# LOAD MODEL
# ============================================================

print("=" * 60)
print("MuJoCo ArduinoBot Validation")
print("=" * 60)

print(f"\nLoading model:")
print(XML_PATH)

if not os.path.exists(XML_PATH):
    raise FileNotFoundError(
        f"\nERROR: MuJoCo XML file not found:\n{XML_PATH}"
    )

try:
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

except Exception as e:
    print("\nERROR: Failed to load MuJoCo model.")
    print(e)
    raise


print("\n✓ MuJoCo XML loaded successfully")

print(f"Number of joints      : {model.njnt}")
print(f"Number of actuators   : {model.nu}")
print(f"Number of sites       : {model.nsite}")
print(f"Number of sensors     : {model.nsensor}")


# ============================================================
# CHECK EXPECTED JOINTS
# ============================================================

EXPECTED_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
]

print("\nJoint validation:")
print("-" * 60)

joint_ids = {}

for joint_name in EXPECTED_JOINTS:

    try:
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name
        )

        if joint_id == -1:
            print(f"✗ {joint_name}: NOT FOUND")
        else:
            joint_ids[joint_name] = joint_id
            print(f"✓ {joint_name}: found (ID {joint_id})")

    except Exception as e:
        print(f"✗ {joint_name}: ERROR")
        print(e)


# ============================================================
# CHECK END-EFFECTOR SITE
# ============================================================

EE_SITE_NAME = "end_effector"

print("\nEnd-effector validation:")
print("-" * 60)

ee_site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    EE_SITE_NAME
)

if ee_site_id == -1:
    print(f"✗ Site '{EE_SITE_NAME}' not found")
else:
    print(f"✓ Site '{EE_SITE_NAME}' found (ID {ee_site_id})")


# ============================================================
# INITIAL STATE
# ============================================================

print("\nInitial robot state:")
print("-" * 60)

mujoco.mj_forward(model, data)

for joint_name, joint_id in joint_ids.items():

    qpos_index = model.jnt_qposadr[joint_id]

    print(
        f"{joint_name}: "
        f"{data.qpos[qpos_index]: .4f} rad"
    )


if ee_site_id != -1:

    ee_position = data.site_xpos[ee_site_id].copy()

    print(
        "End-effector position: "
        f"[{ee_position[0]: .4f}, "
        f"{ee_position[1]: .4f}, "
        f"{ee_position[2]: .4f}] m"
    )


# ============================================================
# ACTUATOR VALIDATION
# ============================================================

print("\nActuator validation:")
print("-" * 60)

for i in range(model.nu):

    actuator_name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        i
    )

    print(
        f"Actuator {i}: "
        f"{actuator_name if actuator_name else 'unnamed'}"
    )


# ============================================================
# START VIEWER
# ============================================================

print("\nStarting MuJoCo viewer...")
print("The robot will perform a basic joint motion test.")
print("Close the viewer to stop the simulation.\n")

with mujoco.viewer.launch_passive(model, data) as viewer:

    start_time = time.time()

    while viewer.is_running():

        elapsed = time.time() - start_time

        if elapsed > SIMULATION_TIME:
            print("\nSimulation time completed.")
            break


        # ----------------------------------------------------
        # BASIC TEST MOTION
        # ----------------------------------------------------

        # Joint 1
        if model.nu >= 1:
            data.ctrl[0] = 0.5 * np.sin(elapsed)

        # Joint 2
        if model.nu >= 2:
            data.ctrl[1] = 0.4 * np.sin(elapsed * 0.7)

        # Joint 3
        if model.nu >= 3:
            data.ctrl[2] = 0.3 * np.sin(elapsed * 1.2)


        # ----------------------------------------------------
        # STEP SIMULATION
        # ----------------------------------------------------

        mujoco.mj_step(model, data)


        # ----------------------------------------------------
        # PRINT STATE
        # ----------------------------------------------------

        if int(elapsed * 10) % 10 == 0:

            positions = []

            for joint_name, joint_id in joint_ids.items():

                qpos_index = model.jnt_qposadr[joint_id]

                positions.append(
                    data.qpos[qpos_index]
                )

            print(
                f"\rTime: {elapsed:5.1f}s | "
                f"Joints: "
                f"{[round(x, 3) for x in positions]}",
                end="",
                flush=True
            )


        # ----------------------------------------------------
        # UPDATE VIEWER
        # ----------------------------------------------------

        viewer.sync()

        time.sleep(0.002)


# ============================================================
# FINAL STATE
# ============================================================

print("\n\n" + "=" * 60)
print("FINAL VALIDATION STATE")
print("=" * 60)

mujoco.mj_forward(model, data)

for joint_name, joint_id in joint_ids.items():

    qpos_index = model.jnt_qposadr[joint_id]

    print(
        f"{joint_name}: "
        f"{data.qpos[qpos_index]: .4f} rad"
    )


if ee_site_id != -1:

    ee_position = data.site_xpos[ee_site_id].copy()

    print(
        "End-effector position: "
        f"[{ee_position[0]: .4f}, "
        f"{ee_position[1]: .4f}, "
        f"{ee_position[2]: .4f}] m"
    )


print("\n✓ MuJoCo validation program finished.")
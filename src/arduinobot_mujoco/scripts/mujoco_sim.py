#!/usr/bin/env python3
"""Run a MuJoCo XML model in the passive native viewer."""

import argparse
import sys
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Launch a MuJoCo model viewer.")
    parser.add_argument("model", type=Path, help="Path to a MuJoCo XML model")
    args = parser.parse_args()

    if not args.model.is_file():
        parser.error(f"model does not exist: {args.model}")

    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print(
            "MuJoCo's Python package is not installed. Install it with: pip install mujoco",
            file=sys.stderr,
        )
        return 1

    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)

    # launch_passive keeps the viewer responsive while this loop advances the
    # simulation in real time.
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            remaining = model.opt.timestep - (time.time() - step_start)
            if remaining > 0:
                time.sleep(remaining)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

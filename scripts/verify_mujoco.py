#!/usr/bin/env python3
"""Run a deterministic physics and headless-rendering MuJoCo smoke test."""

from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "environment_smoke.xml"


def verify() -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    initial_z = float(data.qpos[2])
    max_contacts = 0

    for _ in range(round(1.5 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        max_contacts = max(max_contacts, data.ncon)

    with mujoco.Renderer(model, height=240, width=320) as renderer:
        renderer.update_scene(data, camera="verification")
        pixels = renderer.render()

    result: dict[str, object] = {
        "mujoco_version": mujoco.__version__,
        "backend": os.environ["MUJOCO_GL"],
        "simulated_seconds": round(float(data.time), 3),
        "initial_body_z_m": round(initial_z, 4),
        "final_body_z_m": round(float(data.qpos[2]), 4),
        "max_contacts": int(max_contacts),
        "render_shape": list(pixels.shape),
        "render_std": round(float(np.std(pixels)), 3),
    }

    assert np.isfinite(data.qpos).all(), "simulation produced non-finite state"
    assert 0.09 <= data.qpos[2] <= 0.12, "body did not settle on the ground"
    assert max_contacts > 0, "collision detection did not produce contacts"
    assert pixels.shape == (240, 320, 3), "unexpected offscreen frame size"
    assert np.std(pixels) > 1, "offscreen rendering produced a blank frame"
    return result


if __name__ == "__main__":
    print(json.dumps(verify(), ensure_ascii=False, indent=2))

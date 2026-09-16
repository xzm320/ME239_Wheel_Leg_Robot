#!/usr/bin/env python3
"""Verify the pinned, unmodified Upkie baseline model."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "upkie"
    / "upstream"
    / "scene.xml"
)
EXPECTED_ACTUATORS = (
    "left_hip",
    "left_knee",
    "left_wheel",
    "right_hip",
    "right_knee",
    "right_wheel",
)


def verify() -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    # Upstream's nominal straight-leg pose and base height.
    data.qpos[:7] = (0.0, 0.0, 0.343, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)

    max_contacts = 0
    for _ in range(round(0.5 / model.opt.timestep)):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        max_contacts = max(max_contacts, data.ncon)

    with mujoco.Renderer(model, height=240, width=320) as renderer:
        renderer.update_scene(data)
        pixels = renderer.render()

    actuator_names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        for index in range(model.nu)
    )
    total_mass = float(np.sum(model.body_mass))
    trunk_tilt = 2.0 * math.acos(float(np.clip(abs(data.qpos[3]), 0.0, 1.0)))

    assert (model.nq, model.nv, model.nu) == (13, 12, 6)
    assert actuator_names == EXPECTED_ACTUATORS
    assert math.isclose(total_mass, 5.4605, abs_tol=1e-6)
    assert np.isfinite(data.qpos).all()
    assert max_contacts > 0
    assert data.qpos[2] > 0.3
    assert trunk_tilt < math.radians(2.0)
    assert pixels.shape == (240, 320, 3)
    assert np.std(pixels) > 1.0

    return {
        "source": "MarcDcls/mjlab_upkie@d7895789",
        "mujoco_version": mujoco.__version__,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "total_mass_kg": round(total_mass, 4),
        "simulated_seconds": round(float(data.time), 3),
        "max_contacts": int(max_contacts),
        "trunk_height_m": round(float(data.qpos[2]), 4),
        "trunk_tilt_deg": round(math.degrees(trunk_tilt), 4),
        "render_std": round(float(np.std(pixels)), 3),
    }


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))

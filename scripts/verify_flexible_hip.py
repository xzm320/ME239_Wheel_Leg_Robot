#!/usr/bin/env python3
"""Verify compliant-hip structure, restoring forces, and damping."""

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
    / "flexible_hip"
    / "scene.xml"
)
SLIDE_PARAMETERS = {
    "left_hip_slide_x": (6500.0, 70.0, (-0.020, 0.020)),
    "left_hip_slide_z": (12000.0, 120.0, (-0.025, 0.025)),
    "right_hip_slide_x": (6500.0, 70.0, (-0.020, 0.020)),
    "right_hip_slide_z": (12000.0, 120.0, (-0.025, 0.025)),
}


def _addresses(model: mujoco.MjModel, joint_name: str) -> tuple[int, int, int]:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    assert joint_id >= 0, f"missing joint: {joint_name}"
    return joint_id, int(model.jnt_qposadr[joint_id]), int(model.jnt_dofadr[joint_id])


def _passive_force(
    model: mujoco.MjModel, joint_name: str, displacement: float, velocity: float
) -> float:
    data = mujoco.MjData(model)
    _, qpos_address, dof_address = _addresses(model, joint_name)
    data.qpos[qpos_address] = displacement
    data.qvel[dof_address] = velocity
    mujoco.mj_forward(model, data)
    return float(data.qfrc_passive[dof_address])


def verify() -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    measured_forces: dict[str, dict[str, float]] = {}

    for joint_name, (stiffness, damping, expected_range) in SLIDE_PARAMETERS.items():
        joint_id, _, dof_address = _addresses(model, joint_name)
        assert model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_SLIDE
        assert np.allclose(model.jnt_axis[joint_id], (1, 0, 0) if "_x" in joint_name else (0, 0, 1))
        assert np.allclose(model.jnt_range[joint_id], expected_range)
        assert math.isclose(float(model.jnt_stiffness[joint_id]), stiffness)
        assert math.isclose(float(model.dof_damping[dof_address]), damping)

        positive_force = _passive_force(model, joint_name, 0.010, 0.0)
        negative_force = _passive_force(model, joint_name, -0.010, 0.0)
        damping_force = _passive_force(model, joint_name, 0.0, 0.10)
        assert math.isclose(positive_force, -stiffness * 0.010, abs_tol=1e-9)
        assert math.isclose(negative_force, stiffness * 0.010, abs_tol=1e-9)
        assert math.isclose(damping_force, -damping * 0.10, abs_tol=1e-9)
        measured_forces[joint_name] = {
            "at_positive_10_mm_n": positive_force,
            "at_negative_10_mm_n": negative_force,
            "at_positive_0.1_mps_n": damping_force,
        }

    data = mujoco.MjData(model)
    data.qpos[:7] = (0.0, 0.0, 0.343, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, height=240, width=320) as renderer:
        renderer.update_scene(data)
        pixels = renderer.render()

    total_mass = float(np.sum(model.body_mass))
    assert (model.nq, model.nv, model.nu) == (17, 16, 6)
    assert math.isclose(total_mass, 5.6205, abs_tol=1e-6)
    assert pixels.shape == (240, 320, 3)
    assert np.std(pixels) > 1.0

    return {
        "mujoco_version": mujoco.__version__,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "total_mass_kg": round(total_mass, 4),
        "passive_slide_count": len(SLIDE_PARAMETERS),
        "forces": measured_forces,
        "render_std": round(float(np.std(pixels)), 3),
    }


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))

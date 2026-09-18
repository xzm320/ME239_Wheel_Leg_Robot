#!/usr/bin/env python3
"""Load both robots on the Perlin strip and check the mechanical models."""

from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROTOTYPE_SCENE = ROOT / "models" / "prototype" / "scene.xml"
WIDE_SCENE = ROOT / "models" / "wide_car" / "scene.xml"


def _names(model: mujoco.MjModel, object_type: int) -> tuple[str, ...]:
    return tuple(
        mujoco.mj_id2name(model, object_type, index) or ""
        for index in range(
            model.njnt if object_type == mujoco.mjtObj.mjOBJ_JOINT else model.nu
        )
    )


def verify() -> dict[str, object]:
    prototype = mujoco.MjModel.from_xml_path(str(PROTOTYPE_SCENE))
    wide = mujoco.MjModel.from_xml_path(str(WIDE_SCENE))
    prototype_data = mujoco.MjData(prototype)
    wide_data = mujoco.MjData(wide)
    mujoco.mj_forward(prototype, prototype_data)
    mujoco.mj_forward(wide, wide_data)

    prototype_joints = tuple(
        mujoco.mj_id2name(prototype, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(prototype.njnt)
    )
    wide_joints = tuple(
        mujoco.mj_id2name(wide, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(wide.njnt)
    )
    wide_equality = tuple(
        mujoco.mj_id2name(wide, mujoco.mjtObj.mjOBJ_EQUALITY, index)
        for index in range(wide.neq)
    )

    assert (prototype.nq, prototype.nv, prototype.nu) == (13, 12, 6)
    assert prototype.nhfield == 1
    assert math_isclose_mass(float(np.sum(prototype.body_mass)), 5.3392)
    assert prototype_joints[-6:] == (
        "left_hip",
        "left_knee",
        "left_wheel",
        "right_hip",
        "right_knee",
        "right_wheel",
    )
    assert "left_hip_slide_x" not in prototype_joints

    assert wide.nhfield == 1
    assert "left_hip_slide_x" in wide_joints
    assert "left_hip_slide_z" in wide_joints
    assert "left_strut_extension" in wide_joints
    assert "left_four_bar_closure" in wide_equality
    assert "left_strut_closure" in wide_equality
    assert np.isfinite(prototype_data.qpos).all()
    assert np.isfinite(wide_data.qpos).all()
    assert prototype_data.qpos[2] > 0.3
    assert wide_data.qpos[2] > 0.3

    return {
        "prototype": {
            "nq": prototype.nq,
            "nv": prototype.nv,
            "nu": prototype.nu,
            "mass_kg": round(float(np.sum(prototype.body_mass)), 4),
            "nhfield": int(prototype.nhfield),
            "source": "https://github.com/upkie/upkie",
        },
        "wide_car": {
            "nq": wide.nq,
            "nv": wide.nv,
            "nu": wide.nu,
            "mass_kg": round(float(np.sum(wide.body_mass)), 4),
            "nhfield": int(wide.nhfield),
            "neq": int(wide.neq),
            "hip_y_m": round(float(abs(wide.body_pos[
                mujoco.mj_name2id(wide, mujoco.mjtObj.mjOBJ_BODY, "left_hip_carrier"),
                1,
            ])), 4),
        },
    }


def math_isclose_mass(value: float, expected: float) -> bool:
    return abs(value - expected) < 1e-3


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))

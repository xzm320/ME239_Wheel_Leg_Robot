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
PROTOTYPE_MASS_KG = 13.168
WIDE_CAR_MASS_KG = 19.374
SHARED_HIP_Y_M = 0.3411
SHARED_STAND_Z_M = 0.408


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

    prototype_mass = float(np.sum(prototype.body_mass))
    wide_mass = float(np.sum(wide.body_mass))
    prototype_hip_y = float(
        abs(
            prototype.body_pos[
                mujoco.mj_name2id(prototype, mujoco.mjtObj.mjOBJ_BODY, "left_hip_mount"),
                1,
            ]
        )
    )
    wide_hip_y = float(
        abs(
            wide.body_pos[
                mujoco.mj_name2id(wide, mujoco.mjtObj.mjOBJ_BODY, "left_hip_carrier"),
                1,
            ]
        )
    )

    assert (prototype.nq, prototype.nv, prototype.nu) == (13, 12, 6)
    assert prototype.nhfield == 1
    assert prototype.neq == 0
    assert math_isclose_mass(prototype_mass, PROTOTYPE_MASS_KG)
    assert math_isclose_mass(wide_mass, WIDE_CAR_MASS_KG)
    assert wide_mass - prototype_mass > 5.0
    assert abs(prototype_hip_y - SHARED_HIP_Y_M) < 1e-6
    assert abs(wide_hip_y - SHARED_HIP_Y_M) < 1e-6
    assert abs(float(prototype_data.qpos[2]) - SHARED_STAND_Z_M) < 1e-6
    assert abs(float(wide_data.qpos[2]) - SHARED_STAND_Z_M) < 1e-6
    assert prototype_joints[-6:] == (
        "left_hip",
        "left_knee",
        "left_wheel_joint",
        "right_hip",
        "right_knee",
        "right_wheel_joint",
    )
    assert "left_hip_slide_x" not in prototype_joints
    assert "left_hip_slide_z" not in prototype_joints
    assert "left_strut_extension" not in prototype_joints
    assert not any("four_bar" in (name or "") for name in prototype_joints)

    assert wide.nhfield == 1
    assert "left_hip_slide_x" in wide_joints
    assert "left_hip_slide_z" in wide_joints
    assert "left_strut_extension" in wide_joints
    assert "left_four_bar_closure" in wide_equality
    assert "left_strut_closure" in wide_equality
    assert np.isfinite(prototype_data.qpos).all()
    assert np.isfinite(wide_data.qpos).all()

    return {
        "prototype": {
            "nq": prototype.nq,
            "nv": prototype.nv,
            "nu": prototype.nu,
            "neq": int(prototype.neq),
            "mass_kg": round(prototype_mass, 4),
            "nhfield": int(prototype.nhfield),
            "hip_y_m": round(prototype_hip_y, 4),
            "stand_z_m": round(float(prototype_data.qpos[2]), 4),
            "source": "wide_car ablation (rigid hip, 2-link, no strut)",
        },
        "wide_car": {
            "nq": wide.nq,
            "nv": wide.nv,
            "nu": wide.nu,
            "mass_kg": round(wide_mass, 4),
            "nhfield": int(wide.nhfield),
            "neq": int(wide.neq),
            "hip_y_m": round(wide_hip_y, 4),
            "stand_z_m": round(float(wide_data.qpos[2]), 4),
        },
    }


def math_isclose_mass(value: float, expected: float) -> bool:
    return abs(value - expected) < 1e-3


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))

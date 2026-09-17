#!/usr/bin/env python3
"""Verify closed-chain geometry and active/passive telescopic behavior."""

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
    / "four_bar"
    / "scene.xml"
)
STRUT_JOINTS = (
    "left_strut_extension",
    "left_strut_extension_stage2",
    "right_strut_extension",
    "right_strut_extension_stage2",
)
STRUT_ACTUATORS = ("left_strut", "right_strut")


def _object_id(model: mujoco.MjModel, object_type: int, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    assert object_id >= 0, f"missing MuJoCo object: {name}"
    return object_id


def _joint_qpos_address(model: mujoco.MjModel, name: str) -> int:
    joint_id = _object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return int(model.jnt_qposadr[joint_id])


def _leg_height(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> float:
    hip_id = _object_id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_hip_origin")
    wheel_id = _object_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel_node")
    return float(data.site_xpos[hip_id, 2] - data.xpos[wheel_id, 2])


def _disable_strut_servos(model: mujoco.MjModel) -> None:
    for actuator_name in STRUT_ACTUATORS:
        actuator_id = _object_id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
        )
        model.actuator_gainprm[actuator_id, :] = 0.0
        model.actuator_biasprm[actuator_id, :] = 0.0


def verify() -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    total_mass = float(np.sum(model.body_mass))
    assert (model.nq, model.nv, model.nu, model.neq) == (29, 28, 6, 6)
    assert math.isclose(total_mass, 5.6185, abs_tol=1e-6)

    # Bench test without gravity or contacts: command both telescopic crossbars.
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    data.qpos[:7] = (0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    neutral_height = _leg_height(model, data, "left")

    desired_extension = 0.094
    actuator_ids = [
        _object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        for actuator_name in STRUT_ACTUATORS
    ]

    max_closure_error = 0.0
    for step in range(round(1.2 / model.opt.timestep)):
        # A half-cosine ramp avoids injecting an unrealistic command step.
        ramp_ratio = min(step * model.opt.timestep / 0.4, 1.0)
        ramp = 0.5 - 0.5 * math.cos(math.pi * ramp_ratio)
        # Both synchronized stages carry a 1600 N/m spring. The first-stage
        # servo therefore sees 3200 N/m through the 1:1 synchronization.
        feedforward_command = desired_extension * ramp * (1.0 + 3200.0 / 8000.0)
        data.ctrl[actuator_ids] = feedforward_command
        mujoco.mj_step(model, data)
        max_closure_error = max(
            max_closure_error, float(np.max(np.abs(data.efc_pos[:14])))
        )

    qpos_addresses = [
        _joint_qpos_address(model, joint_name) for joint_name in STRUT_JOINTS
    ]
    active_extensions = data.qpos[qpos_addresses].copy()
    shortened_height = _leg_height(model, data, "left")
    assert np.allclose(active_extensions, desired_extension, atol=2e-4)
    assert shortened_height < 0.18
    assert shortened_height / neutral_height < 0.52
    left_strut_joint_id = _object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "left_strut_extension"
    )
    assert np.allclose(model.jnt_range[left_strut_joint_id], (-0.020, 0.094))
    outer_sleeve_end_id = _object_id(
        model, mujoco.mjtObj.mjOBJ_SITE, "left_strut_sleeve_end"
    )
    middle_rear_id = _object_id(
        model, mujoco.mjtObj.mjOBJ_SITE, "left_strut_middle_rear"
    )
    middle_end_id = _object_id(
        model, mujoco.mjtObj.mjOBJ_SITE, "left_strut_middle_end"
    )
    inner_rear_id = _object_id(
        model, mujoco.mjtObj.mjOBJ_SITE, "left_strut_rod_rear"
    )
    stage_insertions = (
        float(
            np.linalg.norm(
                data.site_xpos[outer_sleeve_end_id]
                - data.site_xpos[middle_rear_id]
            )
        ),
        float(
            np.linalg.norm(
                data.site_xpos[middle_end_id] - data.site_xpos[inner_rear_id]
            )
        ),
    )
    minimum_insertion = min(stage_insertions)
    assert minimum_insertion > 0.025

    # Remove all active strut force; only the physical spring and damper remain.
    _disable_strut_servos(model)
    data.ctrl[:] = 0.0
    for _ in range(round(1.2 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        max_closure_error = max(
            max_closure_error, float(np.max(np.abs(data.efc_pos[:14])))
        )

    passive_return_extensions = data.qpos[qpos_addresses].copy()
    returned_height = _leg_height(model, data, "left")
    assert np.max(np.abs(passive_return_extensions)) < 1e-4
    assert math.isclose(returned_height, neutral_height, abs_tol=2e-4)
    assert max_closure_error < 5e-4
    assert np.isfinite(data.qpos).all()

    # A 20 mm total crossbar extension is 10 mm at each synchronized stage.
    force_data = mujoco.MjData(model)
    left_stage_joint_ids = [
        _object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        for joint_name in (
            "left_strut_extension",
            "left_strut_extension_stage2",
        )
    ]
    for joint_id in left_stage_joint_ids:
        force_data.qpos[int(model.jnt_qposadr[joint_id])] = 0.010
    mujoco.mj_forward(model, force_data)
    passive_force = float(
        force_data.qfrc_passive[int(model.jnt_dofadr[left_stage_joint_ids[0]])]
    )
    assert math.isclose(passive_force, -16.0, abs_tol=1e-9)

    # Restore a gravity-loaded model and check wheel contact stability.
    contact_model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    contact_data = mujoco.MjData(contact_model)
    contact_data.qpos[:7] = (0.0, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    max_contacts = 0
    for _ in range(round(0.5 / contact_model.opt.timestep)):
        mujoco.mj_step(contact_model, contact_data)
        max_contacts = max(max_contacts, contact_data.ncon)
    assert max_contacts > 0
    assert contact_data.qpos[2] > 0.37
    assert np.isfinite(contact_data.qpos).all()

    with mujoco.Renderer(contact_model, height=360, width=480) as renderer:
        renderer.update_scene(contact_data)
        pixels = renderer.render()
    assert pixels.shape == (360, 480, 3)
    assert np.std(pixels) > 1.0

    return {
        "mujoco_version": mujoco.__version__,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "equality_constraints": model.neq,
        "total_mass_kg": round(total_mass, 4),
        "neutral_leg_height_mm": round(neutral_height * 1000, 2),
        "commanded_total_extension_mm": round(desired_extension * 2000, 2),
        "measured_total_extension_mm": round(
            float(active_extensions[0] + active_extensions[1]) * 1000, 2
        ),
        "minimum_height_ratio": round(shortened_height / neutral_height, 4),
        "shortened_leg_height_mm": round(shortened_height * 1000, 2),
        "minimum_rod_insertion_mm": round(minimum_insertion * 1000, 2),
        "passive_return_extension_mm": round(
            float(passive_return_extensions[0]) * 1000, 4
        ),
        "passive_force_at_20_mm_n": round(passive_force, 2),
        "max_closure_error_mm": round(max_closure_error * 1000, 4),
        "gravity_test_contacts": int(max_contacts),
        "gravity_test_trunk_height_m": round(float(contact_data.qpos[2]), 4),
        "render_std": round(float(np.std(pixels)), 3),
    }


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))

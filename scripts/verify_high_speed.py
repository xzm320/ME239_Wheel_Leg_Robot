#!/usr/bin/env python3
"""Find and verify a robust speed after the requested 80 km/h test."""

from __future__ import annotations

import json
import math
import os

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

if __package__:
    from scripts.balance_controller import (
        BalanceSpeedController,
        quaternion_pitch,
    )
    from scripts.high_speed_evaluation import (
        MODEL_PATH,
        WHEEL_RADIUS_M,
        high_speed_balance_gains,
        result_dict,
        run_high_speed_episode,
    )
else:
    from balance_controller import BalanceSpeedController, quaternion_pitch
    from high_speed_evaluation import (
        MODEL_PATH,
        WHEEL_RADIUS_M,
        high_speed_balance_gains,
        result_dict,
        run_high_speed_episode,
    )

REQUESTED_SPEED_M_S = 80.0 / 3.6
ROBUST_SPEEDS_M_S = (9.7, 10.0, 10.3)
FLAT_MODEL_PATH = (
    MODEL_PATH.parents[1] / "four_bar" / "scene.xml"
)


def _run_flat_acceleration() -> dict[str, float | bool]:
    model = mujoco.MjModel.from_xml_path(str(FLAT_MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[:7] = (0.0, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    wheel_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ("left_wheel", "right_wheel")
    ]
    controller = BalanceSpeedController(
        float(model.opt.timestep),
        high_speed_balance_gains(),
    )
    acceleration = 0.4
    ramp_duration = REQUESTED_SPEED_M_S / acceleration
    duration = 1.0 + ramp_duration + 5.0
    speeds: list[float] = []
    pitches: list[float] = []
    stable = True
    for _ in range(round(duration / model.opt.timestep)):
        target = (
            0.0
            if data.time < 1.0
            else min(
                REQUESTED_SPEED_M_S,
                acceleration * (data.time - 1.0),
            )
        )
        pitch = quaternion_pitch(data.qpos[3:7])
        output = controller.update(
            target_speed_m_s=target,
            forward_speed_m_s=float(data.qvel[0]),
            pitch_rad=pitch,
            pitch_rate_rad_s=float(data.qvel[4]),
        )
        data.ctrl[wheel_ids] = output.wheel_torque_nm
        mujoco.mj_step(model, data)
        speeds.append(float(data.qvel[0]))
        pitches.append(pitch)
        if abs(pitch) > math.radians(15.0) or data.qpos[2] < 0.30:
            stable = False
            break
    hold_steps = round(4.0 / model.opt.timestep)
    return {
        "stable": stable,
        "peak_speed_m_s": float(np.max(speeds)),
        "mean_final_4s_speed_m_s": float(np.mean(speeds[-hold_steps:])),
        "maximum_pitch_deg": math.degrees(float(np.max(np.abs(pitches)))),
    }


def verify() -> dict[str, object]:
    flat_result = _run_flat_acceleration()
    assert flat_result["stable"]
    assert flat_result["mean_final_4s_speed_m_s"] > 21.0

    requested_result = run_high_speed_episode(REQUESTED_SPEED_M_S)
    assert not requested_result.stable
    assert requested_result.failure_reason == "roll exceeded 15 deg"

    neighborhood = [
        run_high_speed_episode(speed) for speed in ROBUST_SPEEDS_M_S
    ]
    assert all(result.stable for result in neighborhood)
    selected = neighborhood[1]
    assert selected.maximum_roll_deg < 1.0
    assert selected.maximum_pitch_deg < 2.0
    assert selected.com_height_std_mm < 3.0
    assert selected.maximum_com_deviation_mm < 8.0
    assert selected.speed_rmse_m_s < 0.5

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    wheel_damping = []
    for name in ("left_wheel_joint", "right_wheel_joint"):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        wheel_damping.append(
            float(model.dof_damping[int(model.jnt_dofadr[joint_id])])
        )
    assert wheel_damping == [0.005, 0.005]

    selected_wheel_speed = selected.target_speed_m_s / WHEEL_RADIUS_M
    selected_peak_power = (
        2.0
        * selected.maximum_wheel_torque_nm
        * selected_wheel_speed
    )
    return {
        "flat_acceleration_toward_80_km_h": flat_result,
        "requested_80_km_h": result_dict(requested_result),
        "selected_robust_speed_m_s": selected.target_speed_m_s,
        "selected_robust_speed_km_h": selected.target_speed_m_s * 3.6,
        "verified_neighborhood_m_s": list(ROBUST_SPEEDS_M_S),
        "verified_neighborhood_km_h": [
            round(speed * 3.6, 2) for speed in ROBUST_SPEEDS_M_S
        ],
        "selected_result": result_dict(selected),
        "wheel_joint_damping_n_m_s_rad": wheel_damping[0],
        "selected_wheel_speed_rpm": round(
            selected_wheel_speed * 60.0 / (2.0 * math.pi),
            1,
        ),
        "estimated_peak_mechanical_power_w": round(
            selected_peak_power,
            1,
        ),
    }


if __name__ == "__main__":
    print(json.dumps(verify(), ensure_ascii=False, indent=2))

#!/usr/bin/env python3
"""Verify flat-ground balance, speed tracking, braking, and tilt recovery."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

if __package__:
    from scripts.balance_controller import (
        BalanceSpeedController,
        flat_speed_profile,
        quaternion_pitch,
    )
else:
    from balance_controller import (
        BalanceSpeedController,
        flat_speed_profile,
        quaternion_pitch,
    )

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "upkie"
    / "four_bar"
    / "scene.xml"
)
WHEEL_ACTUATORS = ("left_wheel", "right_wheel")


def _id(model: mujoco.MjModel, object_type: int, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return object_id


def _initialize(model: mujoco.MjModel, initial_pitch_rad: float = 0.0) -> mujoco.MjData:
    data = mujoco.MjData(model)
    data.qpos[:7] = (
        0.0,
        0.0,
        0.408,
        math.cos(initial_pitch_rad / 2.0),
        0.0,
        math.sin(initial_pitch_rad / 2.0),
        0.0,
    )
    mujoco.mj_forward(model, data)
    return data


def _run(
    duration_s: float,
    *,
    initial_pitch_rad: float = 0.0,
    use_speed_profile: bool = True,
) -> dict[str, np.ndarray | float]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = _initialize(model, initial_pitch_rad)
    controller = BalanceSpeedController(float(model.opt.timestep))
    wheel_ids = [
        _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in WHEEL_ACTUATORS
    ]

    times: list[float] = []
    targets: list[float] = []
    speeds: list[float] = []
    pitches: list[float] = []
    torques: list[float] = []
    minimum_trunk_height = float("inf")

    for _ in range(round(duration_s / model.opt.timestep)):
        time_s = float(data.time)
        target_speed = flat_speed_profile(time_s) if use_speed_profile else 0.0
        pitch = quaternion_pitch(data.qpos[3:7])
        output = controller.update(
            target_speed_m_s=target_speed,
            forward_speed_m_s=float(data.qvel[0]),
            pitch_rad=pitch,
            pitch_rate_rad_s=float(data.qvel[4]),
        )
        data.ctrl[wheel_ids] = output.wheel_torque_nm
        mujoco.mj_step(model, data)

        if not np.isfinite(data.qpos).all():
            raise AssertionError("controller simulation produced non-finite state")
        minimum_trunk_height = min(minimum_trunk_height, float(data.qpos[2]))
        times.append(time_s)
        targets.append(target_speed)
        speeds.append(float(data.qvel[0]))
        pitches.append(pitch)
        torques.append(output.wheel_torque_nm)

    return {
        "time": np.asarray(times),
        "target_speed": np.asarray(targets),
        "speed": np.asarray(speeds),
        "pitch": np.asarray(pitches),
        "torque": np.asarray(torques),
        "minimum_trunk_height": minimum_trunk_height,
        "final_x": float(data.qpos[0]),
    }


def verify() -> dict[str, object]:
    run = _run(14.0)
    times = run["time"]
    speeds = run["speed"]
    pitches = run["pitch"]
    torques = run["torque"]
    assert isinstance(times, np.ndarray)
    assert isinstance(speeds, np.ndarray)
    assert isinstance(pitches, np.ndarray)
    assert isinstance(torques, np.ndarray)

    cruise = (times >= 5.0) & (times < 8.0)
    settled = times >= 13.0
    cruise_mean = float(np.mean(speeds[cruise]))
    cruise_rmse = float(np.sqrt(np.mean((speeds[cruise] - 2.0) ** 2)))
    final_speed = float(np.mean(speeds[settled]))
    maximum_pitch = float(np.max(np.abs(pitches)))
    saturation_fraction = float(np.mean(np.abs(torques) >= 5.999))

    assert run["minimum_trunk_height"] > 0.30
    assert 1.95 < cruise_mean < 2.20
    assert cruise_rmse < 0.16
    assert abs(final_speed) < 0.03
    assert maximum_pitch < math.radians(9.0)
    assert saturation_fraction < 0.55

    disturbance = _run(
        6.0,
        initial_pitch_rad=math.radians(8.0),
        use_speed_profile=False,
    )
    disturbance_speed = disturbance["speed"]
    disturbance_pitch = disturbance["pitch"]
    assert isinstance(disturbance_speed, np.ndarray)
    assert isinstance(disturbance_pitch, np.ndarray)
    assert disturbance["minimum_trunk_height"] > 0.30
    assert abs(float(disturbance_speed[-1])) < 0.05
    assert abs(float(disturbance_pitch[-1])) < math.radians(0.6)

    return {
        "controller": "cascaded speed PI + pitch PD",
        "target_cruise_speed_m_s": 2.0,
        "target_cruise_speed_km_h": 7.2,
        "measured_cruise_speed_m_s": round(cruise_mean, 4),
        "cruise_speed_rmse_m_s": round(cruise_rmse, 4),
        "final_speed_m_s": round(final_speed, 4),
        "maximum_abs_pitch_deg": round(math.degrees(maximum_pitch), 3),
        "minimum_trunk_height_m": round(float(run["minimum_trunk_height"]), 4),
        "wheel_torque_limit_nm": 6.0,
        "torque_saturation_fraction": round(saturation_fraction, 4),
        "distance_m": round(float(run["final_x"]), 3),
        "recovered_initial_pitch_deg": 8.0,
        "disturbance_final_pitch_deg": round(
            math.degrees(float(disturbance_pitch[-1])), 3
        ),
        "disturbance_final_speed_m_s": round(float(disturbance_speed[-1]), 4),
    }


if __name__ == "__main__":
    print(json.dumps(verify(), ensure_ascii=False, indent=2))

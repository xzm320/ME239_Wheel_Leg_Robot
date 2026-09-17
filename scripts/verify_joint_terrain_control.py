#!/usr/bin/env python3
"""Verify combined balance, yaw, and active leg control on medium terrain."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

if __package__:
    from scripts.balance_controller import quaternion_pitch
    from scripts.joint_terrain_controller import (
        JointTerrainController,
        TerrainControlGains,
        medium_terrain_speed_profile,
        quaternion_roll_yaw,
        sample_preview_ground_heights,
    )
else:
    from balance_controller import quaternion_pitch
    from joint_terrain_controller import (
        JointTerrainController,
        TerrainControlGains,
        medium_terrain_speed_profile,
        quaternion_roll_yaw,
        sample_preview_ground_heights,
    )

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "upkie"
    / "terrain"
    / "scene_medium.xml"
)


def _id(model: mujoco.MjModel, object_type: int, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return object_id


def run_episode(
    gains: TerrainControlGains | None = None,
) -> dict[str, np.ndarray | float]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[:7] = (-8.5, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)

    wheel_actuator_ids = [
        _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ("left_wheel", "right_wheel")
    ]
    strut_actuator_ids = [
        _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ("left_strut", "right_strut")
    ]
    wheel_body_ids = tuple(
        _id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in ("left_wheel_node", "right_wheel_node")
    )
    trunk_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
    parameters = gains or TerrainControlGains()
    controller = JointTerrainController(float(model.opt.timestep), parameters)
    ground_heights = sample_preview_ground_heights(
        model, data, wheel_body_ids, 0.0, parameters
    )
    initial_com_height = float(data.subtree_com[trunk_body_id, 2])

    times: list[float] = []
    speeds: list[float] = []
    trunk_heights: list[float] = []
    com_heights: list[float] = []
    rolls: list[float] = []
    pitches: list[float] = []
    yaws: list[float] = []
    stage_targets: list[np.ndarray] = []

    for step in range(round(14.0 / model.opt.timestep)):
        if step % 5 == 0:
            ground_heights = sample_preview_ground_heights(
                model,
                data,
                wheel_body_ids,
                float(data.qvel[0]),
                parameters,
            )
        time_s = float(data.time)
        target_speed = medium_terrain_speed_profile(time_s)
        output = controller.update(
            target_speed_m_s=target_speed,
            forward_speed_m_s=float(data.qvel[0]),
            trunk_height_m=float(data.qpos[2]),
            vertical_speed_m_s=float(data.qvel[2]),
            quaternion_wxyz=data.qpos[3:7],
            angular_velocity_xyz=data.qvel[3:6],
            preview_ground_heights_m=ground_heights,
        )
        data.ctrl[wheel_actuator_ids] = output.wheel_torques_nm
        data.ctrl[strut_actuator_ids] = output.strut_controls_m
        mujoco.mj_step(model, data)

        roll, yaw = quaternion_roll_yaw(data.qpos[3:7])
        pitch = quaternion_pitch(data.qpos[3:7])
        if not np.isfinite(data.qpos).all():
            raise AssertionError("joint controller produced non-finite state")
        assert data.qpos[2] > 0.25, "robot fell below safe trunk height"
        assert abs(roll) < math.radians(15.0), "robot lost lateral balance"
        assert abs(pitch) < math.radians(30.0), "robot lost sagittal balance"

        times.append(time_s)
        speeds.append(float(data.qvel[0]))
        trunk_heights.append(float(data.qpos[2]))
        com_heights.append(float(data.subtree_com[trunk_body_id, 2]))
        rolls.append(roll)
        pitches.append(pitch)
        yaws.append(yaw)
        stage_targets.append(output.stage_targets_m)

    return {
        "time": np.asarray(times),
        "speed": np.asarray(speeds),
        "trunk_height": np.asarray(trunk_heights),
        "com_height": np.asarray(com_heights),
        "roll": np.asarray(rolls),
        "pitch": np.asarray(pitches),
        "yaw": np.asarray(yaws),
        "stage_targets": np.asarray(stage_targets),
        "initial_com_height": initial_com_height,
        "final_x": float(data.qpos[0]),
    }


def verify() -> dict[str, object]:
    run = run_episode()
    times = np.asarray(run["time"])
    speeds = np.asarray(run["speed"])
    trunk_heights = np.asarray(run["trunk_height"])
    com_heights = np.asarray(run["com_height"])
    rolls = np.asarray(run["roll"])
    pitches = np.asarray(run["pitch"])
    yaws = np.asarray(run["yaw"])
    stage_targets = np.asarray(run["stage_targets"])
    active = (times >= 4.0) & (times < 12.0)
    cruise = (times >= 6.0) & (times < 10.0)

    com_standard_deviation = float(np.std(com_heights[active]))
    maximum_com_deviation = float(
        np.max(np.abs(com_heights[active] - float(run["initial_com_height"])))
    )
    cruise_speed = float(np.mean(speeds[cruise]))
    maximum_roll = float(np.max(np.abs(rolls)))
    maximum_pitch = float(np.max(np.abs(pitches)))
    maximum_yaw = float(np.max(np.abs(yaws)))

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    left_wheel_body = _id(model, mujoco.mjtObj.mjOBJ_BODY, "left_wheel")
    wheel_geom_id = int(model.body_geomadr[left_wheel_body])
    wheel_radius = float(model.geom_size[wheel_geom_id, 0])

    assert math.isclose(wheel_radius, 0.120, abs_tol=1e-12)
    assert float(run["final_x"]) > 5.0
    assert com_standard_deviation < 0.012
    assert maximum_com_deviation < 0.045
    assert 1.45 < cruise_speed < 1.75
    assert maximum_roll < math.radians(5.0)
    assert maximum_pitch < math.radians(23.0)
    assert maximum_yaw < math.radians(12.0)
    assert np.min(stage_targets) >= -0.020
    assert np.max(stage_targets) <= 0.094

    return {
        "terrain": "medium",
        "target_speed_m_s": 1.5,
        "measured_cruise_speed_m_s": round(cruise_speed, 4),
        "distance_m": round(float(run["final_x"]) + 8.5, 3),
        "final_world_x_m": round(float(run["final_x"]), 3),
        "wheel_radius_mm": round(wheel_radius * 1000, 1),
        "wheel_diameter_mm": round(wheel_radius * 2000, 1),
        "com_height_standard_deviation_mm": round(
            com_standard_deviation * 1000, 3
        ),
        "maximum_com_height_deviation_mm": round(
            maximum_com_deviation * 1000, 3
        ),
        "maximum_roll_deg": round(math.degrees(maximum_roll), 3),
        "maximum_pitch_deg": round(math.degrees(maximum_pitch), 3),
        "maximum_yaw_deg": round(math.degrees(maximum_yaw), 3),
        "minimum_stage_extension_mm": round(
            float(np.min(stage_targets)) * 1000, 3
        ),
        "maximum_stage_extension_mm": round(
            float(np.max(stage_targets)) * 1000, 3
        ),
    }


if __name__ == "__main__":
    print(json.dumps(verify(), ensure_ascii=False, indent=2))

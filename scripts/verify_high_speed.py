#!/usr/bin/env python3
"""Verify 100 km/h PID acceleration onto the long rough track."""

from __future__ import annotations

import json
import math
import os

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco

if __package__:
    from scripts.high_speed_evaluation import (
        HUNDRED_KMH_WHEEL_TRACK_SCALE,
        MODEL_PATH,
        NOMINAL_HIP_Y_M,
        WHEEL_RADIUS_M,
        result_dict,
        run_high_speed_episode,
        run_hundred_kmh_episode,
    )
else:
    from high_speed_evaluation import (
        HUNDRED_KMH_WHEEL_TRACK_SCALE,
        MODEL_PATH,
        NOMINAL_HIP_Y_M,
        WHEEL_RADIUS_M,
        result_dict,
        run_high_speed_episode,
        run_hundred_kmh_episode,
    )

ROBUST_SPEEDS_M_S = (9.8, 10.0, 10.2)
ROUGH_START_DISTANCE_M = 960.0


def verify() -> dict[str, object]:
    neighborhood = [
        run_high_speed_episode(speed) for speed in ROBUST_SPEEDS_M_S
    ]
    assert all(result.stable for result in neighborhood)
    selected = neighborhood[1]
    assert selected.maximum_roll_deg < 2.0
    assert selected.com_height_std_mm < 4.0

    hundred = run_hundred_kmh_episode()
    assert hundred.wheel_track_scale == HUNDRED_KMH_WHEEL_TRACK_SCALE
    assert hundred.peak_speed_m_s > 27.5
    assert hundred.stable
    assert hundred.distance_m > 1000.0
    assert hundred.maximum_pitch_deg < 25.0
    assert abs(hundred.final_y_m) < 6.0

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    trunk_geom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "trunk_collision"
    )
    assert 0.310 <= float(model.geom_size[trunk_geom, 1]) <= 0.340
    geom_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        for geom_id in range(model.ngeom)
    ]
    assert all(
        name is None or not name.startswith("scenery_")
        for name in geom_names
    )
    wheel_damping = []
    for name in ("left_wheel_joint", "right_wheel_joint"):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        wheel_damping.append(
            float(model.dof_damping[int(model.jnt_dofadr[joint_id])])
        )
    assert wheel_damping == [0.005, 0.005]

    return {
        "selected_robust_speed_m_s": selected.target_speed_m_s,
        "selected_robust_speed_km_h": selected.target_speed_m_s * 3.6,
        "verified_neighborhood_m_s": list(ROBUST_SPEEDS_M_S),
        "selected_result": result_dict(selected),
        "requested_100_km_h": result_dict(hundred),
        "requested_100_km_h_peak_km_h": round(
            hundred.peak_speed_m_s * 3.6, 2
        ),
        "entered_rough_before_failure": hundred.distance_m
        > ROUGH_START_DISTANCE_M,
        "hundred_kmh_stable": hundred.stable,
        "hundred_kmh_failure": hundred.failure_reason,
        "hundred_kmh_final_y_m": hundred.final_y_m,
        "wheel_track_scale": hundred.wheel_track_scale,
        "wheel_track_mm": round(
            2.0 * NOMINAL_HIP_Y_M * hundred.wheel_track_scale * 1000.0, 1
        ),
        "chassis_half_width_mm": round(
            float(model.geom_size[trunk_geom, 1]) * 1000.0, 1
        ),
        "wheel_joint_damping_n_m_s_rad": wheel_damping[0],
        "selected_wheel_speed_rpm": round(
            selected.target_speed_m_s / WHEEL_RADIUS_M * 60.0 / (2.0 * math.pi),
            1,
        ),
    }


if __name__ == "__main__":
    print(json.dumps(verify(), ensure_ascii=False, indent=2))

#!/usr/bin/env python3
"""Grid-search semi-active damping around the selected high-speed band."""

from __future__ import annotations

import itertools
import json
from dataclasses import asdict

import numpy as np

if __package__:
    from scripts.high_speed_evaluation import run_high_speed_episode
    from scripts.joint_terrain_controller import ComplianceParameters
else:
    from high_speed_evaluation import run_high_speed_episode
    from joint_terrain_controller import ComplianceParameters

SPEEDS_M_S = (9.7, 10.0, 10.3)
HIP_X_DAMPING = (100.0, 120.0, 140.0)
HIP_Z_DAMPING = (180.0, 200.0, 220.0)
STRUT_DAMPING = (50.0, 55.0, 60.0)


def tune() -> dict[str, object]:
    evaluations: list[dict[str, object]] = []
    for hip_x, hip_z, strut in itertools.product(
        HIP_X_DAMPING,
        HIP_Z_DAMPING,
        STRUT_DAMPING,
    ):
        parameters = ComplianceParameters(
            hip_x_stiffness_n_m=3000.0,
            hip_x_damping_n_s_m=hip_x,
            hip_z_stiffness_n_m=7000.0,
            hip_z_damping_n_s_m=hip_z,
            strut_equivalent_stiffness_n_m=800.0,
            strut_equivalent_damping_n_s_m=strut,
        )
        results = [
            run_high_speed_episode(speed, compliance=parameters)
            for speed in SPEEDS_M_S
        ]
        stable = all(result.stable for result in results)
        if stable:
            score = float(
                np.mean(
                    [
                        0.25 * result.com_height_std_mm / 3.0
                        + 0.25
                        * result.vertical_acceleration_rms_m_s2
                        / 4.0
                        + 0.20 * result.maximum_roll_deg / 5.0
                        + 0.15 * result.speed_rmse_m_s / 0.5
                        + 0.15 * result.maximum_wheel_torque_nm / 6.0
                        for result in results
                    ]
                )
            )
        else:
            score = None
        evaluations.append(
            {
                "parameters": asdict(parameters),
                "stable_across_band": stable,
                "score": score,
                "results": [asdict(result) for result in results],
            }
        )

    stable_evaluations = [
        evaluation
        for evaluation in evaluations
        if evaluation["stable_across_band"]
    ]
    best = min(
        stable_evaluations,
        key=lambda evaluation: float(evaluation["score"]),
    )
    return {
        "method": "full factorial damping grid",
        "speeds_m_s": list(SPEEDS_M_S),
        "candidate_count": len(evaluations),
        "stable_candidate_count": len(stable_evaluations),
        "best": best,
        "evaluations": sorted(
            evaluations,
            key=lambda evaluation: (
                not bool(evaluation["stable_across_band"]),
                float(evaluation["score"])
                if evaluation["stable_across_band"]
                else float("inf"),
            ),
        ),
    }


if __name__ == "__main__":
    print(json.dumps(tune(), indent=2))

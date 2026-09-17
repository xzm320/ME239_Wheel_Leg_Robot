#!/usr/bin/env python3
"""Deterministically tune passive compliance on the medium terrain course."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

if __package__:
    from scripts.joint_terrain_controller import ComplianceParameters
    from scripts.verify_joint_terrain_control import run_episode
else:
    from joint_terrain_controller import ComplianceParameters
    from verify_joint_terrain_control import run_episode

PARAMETER_BOUNDS = {
    "hip_x_stiffness_n_m": (3000.0, 9000.0),
    "hip_x_damping_n_s_m": (40.0, 140.0),
    "hip_z_stiffness_n_m": (6000.0, 18000.0),
    "hip_z_damping_n_s_m": (80.0, 220.0),
    "strut_equivalent_stiffness_n_m": (600.0, 2200.0),
    "strut_equivalent_damping_n_s_m": (15.0, 60.0),
}
WEIGHTS = {
    "com_height_std_mm": 0.22,
    "maximum_com_deviation_mm": 0.13,
    "vertical_acceleration_rms_m_s2": 0.22,
    "vertical_acceleration_p95_m_s2": 0.13,
    "pitch_rms_deg": 0.10,
    "roll_rms_deg": 0.06,
    "speed_rmse_m_s": 0.06,
    "strut_force_p95_n": 0.08,
}
NORMALIZERS = {
    "com_height_std_mm": 12.0,
    "maximum_com_deviation_mm": 45.0,
    "vertical_acceleration_rms_m_s2": 6.0,
    "vertical_acceleration_p95_m_s2": 13.0,
    "pitch_rms_deg": 5.0,
    "roll_rms_deg": 1.0,
    "speed_rmse_m_s": 0.5,
    "strut_force_p95_n": 200.0,
}


def _metrics(run: dict[str, np.ndarray | float]) -> dict[str, float]:
    times = np.asarray(run["time"])
    active = (times >= 4.0) & (times < 12.0)
    com_height = np.asarray(run["com_height"])[active]
    acceleration = np.asarray(run["vertical_acceleration"])[active]
    pitch = np.asarray(run["pitch"])[active]
    roll = np.asarray(run["roll"])[active]
    speed = np.asarray(run["speed"])[active]
    target = np.array(
        [
            1.5 if time < 10.0 else 0.75 * (12.0 - time)
            for time in times[active]
        ]
    )
    strut_force = np.asarray(run["strut_force"])[active]
    return {
        "com_height_std_mm": float(np.std(com_height) * 1000.0),
        "maximum_com_deviation_mm": float(
            np.max(
                np.abs(com_height - float(run["initial_com_height"]))
            )
            * 1000.0
        ),
        "vertical_acceleration_rms_m_s2": float(
            np.sqrt(np.mean(acceleration**2))
        ),
        "vertical_acceleration_p95_m_s2": float(
            np.percentile(np.abs(acceleration), 95)
        ),
        "pitch_rms_deg": float(np.degrees(np.sqrt(np.mean(pitch**2)))),
        "roll_rms_deg": float(np.degrees(np.sqrt(np.mean(roll**2)))),
        "speed_rmse_m_s": float(np.sqrt(np.mean((speed - target) ** 2))),
        "strut_force_p95_n": float(np.percentile(strut_force, 95)),
    }


def _score(metrics: dict[str, float]) -> float:
    return sum(
        WEIGHTS[name] * metrics[name] / NORMALIZERS[name]
        for name in WEIGHTS
    )


def _latin_hypercube(count: int, seed: int) -> list[ComplianceParameters]:
    rng = np.random.default_rng(seed)
    names = tuple(PARAMETER_BOUNDS)
    unit_samples = np.empty((count, len(names)))
    for dimension in range(len(names)):
        unit_samples[:, dimension] = (
            rng.permutation(count) + rng.random(count)
        ) / count

    candidates: list[ComplianceParameters] = []
    for row in unit_samples:
        values = {}
        for index, name in enumerate(names):
            lower, upper = PARAMETER_BOUNDS[name]
            values[name] = lower + row[index] * (upper - lower)
        candidates.append(ComplianceParameters(**values))
    return candidates


def tune(sample_count: int = 48, seed: int = 20260917) -> dict[str, object]:
    baseline = ComplianceParameters(6500, 70, 12000, 120, 1500, 25)
    recommended = ComplianceParameters(7100, 48, 7800, 165, 900, 50)
    anchors = [
        baseline,
        recommended,
        ComplianceParameters(3000, 100, 7000, 180, 800, 50),
        ComplianceParameters(5000, 90, 9000, 160, 1200, 40),
        ComplianceParameters(8000, 120, 8000, 150, 1500, 45),
    ]
    candidates = anchors + _latin_hypercube(sample_count, seed)
    evaluations: list[dict[str, object]] = []
    for parameters in candidates:
        try:
            metrics = _metrics(run_episode(compliance=parameters))
            score = _score(metrics)
            evaluations.append(
                {
                    "parameters": asdict(parameters),
                    "metrics": metrics,
                    "score": score,
                    "stable": True,
                }
            )
        except AssertionError as error:
            evaluations.append(
                {
                    "parameters": asdict(parameters),
                    "score": None,
                    "stable": False,
                    "failure": str(error),
                }
            )

    stable = [item for item in evaluations if item["stable"]]
    best = min(stable, key=lambda item: float(item["score"]))
    baseline_result = evaluations[0]
    recommended_result = evaluations[1]
    return {
        "method": "seeded Latin hypercube plus engineering anchors",
        "seed": seed,
        "random_sample_count": sample_count,
        "evaluated_count": len(evaluations),
        "stable_count": len(stable),
        "parameter_bounds": PARAMETER_BOUNDS,
        "objective_weights": WEIGHTS,
        "baseline": baseline_result,
        "best": best,
        "recommended_rounded": recommended_result,
        "improvement_percent": 100.0
        * (
            1.0
            - float(best["score"]) / float(baseline_result["score"])
        ),
        "evaluations": sorted(
            evaluations,
            key=lambda item: (
                not bool(item["stable"]),
                float(item["score"]) if item["stable"] else float("inf"),
            ),
        ),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=48)
    parser.add_argument("--seed", type=int, default=20260917)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/compliance_tuning_v1.json"),
    )
    arguments = parser.parse_args()
    result = tune(arguments.samples, arguments.seed)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                key: result[key]
                for key in (
                    "evaluated_count",
                    "stable_count",
                    "improvement_percent",
                    "baseline",
                    "best",
                    "recommended_rounded",
                )
            },
            indent=2,
        )
    )

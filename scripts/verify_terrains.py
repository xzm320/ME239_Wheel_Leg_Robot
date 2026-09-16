#!/usr/bin/env python3
"""Verify graded terrain assets, metric scaling, and robot contacts."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

if __package__:
    from scripts.generate_terrains import OUTPUT_DIRECTORY, SPECS, generate_heightfield
else:
    from generate_terrains import OUTPUT_DIRECTORY, SPECS, generate_heightfield


def _hfield_data(model: mujoco.MjModel, hfield_id: int) -> np.ndarray:
    address = int(model.hfield_adr[hfield_id])
    rows = int(model.hfield_nrow[hfield_id])
    columns = int(model.hfield_ncol[hfield_id])
    return model.hfield_data[address : address + rows * columns].reshape(rows, columns)


def verify() -> dict[str, object]:
    metadata = json.loads((OUTPUT_DIRECTORY / "metadata.json").read_text())
    summaries: dict[str, object] = {}
    rms_values: list[float] = []
    slope_values: list[float] = []

    for spec in SPECS:
        scene_path = OUTPUT_DIRECTORY / f"scene_{spec.name}.xml"
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        hfield_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_HFIELD, f"{spec.name}_heightfield"
        )
        assert hfield_id >= 0
        values = _hfield_data(model, hfield_id)
        assert values.shape == (161, 801)
        assert math.isclose(float(np.min(values)), 0.0, abs_tol=1e-9)
        assert math.isclose(float(np.max(values)), 1.0, abs_tol=1e-9)
        assert np.allclose(
            model.hfield_size[hfield_id],
            (10.0, 2.0, 2.0 * spec.amplitude_m, 0.10),
        )

        # Column 60 corresponds to x=-8.5 m in the flat launch zone.
        flat_value = float(values[values.shape[0] // 2, 60])
        terrain_geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "terrain"
        )
        flat_height = (
            float(model.geom_pos[terrain_geom_id, 2])
            + float(model.hfield_size[hfield_id, 2]) * flat_value
        )
        assert abs(flat_height) < 1e-7

        generated_heights, generated_metrics = generate_heightfield(spec)
        recorded_metrics = metadata["terrains"][spec.name]["metrics"]
        for key, expected in generated_metrics.items():
            assert math.isclose(recorded_metrics[key], expected, abs_tol=1e-12)
        assert np.max(np.abs(generated_heights)) <= spec.amplitude_m + 1e-12

        data = mujoco.MjData(model)
        data.qpos[:7] = (-8.5, 0.0, 0.343, 1.0, 0.0, 0.0, 0.0)
        max_contacts = 0
        for _ in range(round(0.4 / model.opt.timestep)):
            mujoco.mj_step(model, data)
            max_contacts = max(max_contacts, data.ncon)
        assert max_contacts > 0
        assert data.qpos[2] > 0.3
        assert np.isfinite(data.qpos).all()

        rms_height = float(recorded_metrics["rms_height_m"])
        maximum_slope = float(recorded_metrics["maximum_slope_deg"])
        rms_values.append(rms_height)
        slope_values.append(maximum_slope)
        summaries[spec.name] = {
            "peak_design_height_mm": round(spec.amplitude_m * 1000, 1),
            "rms_height_mm": round(rms_height * 1000, 2),
            "maximum_slope_deg": round(maximum_slope, 2),
            "flat_launch_height_mm": round(flat_height * 1000, 6),
            "contact_count": int(max_contacts),
            "trunk_height_after_0.4_s_m": round(float(data.qpos[2]), 4),
        }

    assert rms_values == sorted(rms_values)
    assert slope_values == sorted(slope_values)
    assert slope_values[-1] < 25.0

    return {
        "terrain_count": len(SPECS),
        "road_length_m": metadata["length_m"],
        "road_width_m": metadata["width_m"],
        "grid": metadata["grid"],
        "levels": summaries,
    }


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))

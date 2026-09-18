#!/usr/bin/env python3
"""Print the catalog mass budget from the loaded MJCF models."""

from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def _bodies(model: mujoco.MjModel) -> list[dict[str, object]]:
    rows = []
    for index in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index) or ""
        mass = float(model.body_mass[index])
        if mass <= 0.0:
            continue
        rows.append({"body": name, "mass_kg": round(mass, 4)})
    return rows


def budget() -> dict[str, object]:
    prototype = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "prototype" / "robot.xml"))
    wide = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "wide_car" / "robot.xml"))
    prototype_mass = float(np.sum(prototype.body_mass))
    wide_mass = float(np.sum(wide.body_mass))
    return {
        "prototype_kg": round(prototype_mass, 4),
        "wide_car_kg": round(wide_mass, 4),
        "delta_wide_minus_prototype_kg": round(wide_mass - prototype_mass, 4),
        "prototype_bodies": _bodies(prototype),
        "wide_car_bodies": _bodies(wide),
    }


if __name__ == "__main__":
    print(json.dumps(budget(), indent=2))

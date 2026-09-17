#!/usr/bin/env python3
"""Generate a long high-speed track from Unitree Perlin heightfields.

The surface follows ``AddPerlinHeighField`` in unitree_mujoco/terrain_tool:
octaved Perlin (3 octaves, persistence 0.5, lacunarity 2.0) encoded as
``(noise + 1) / 2``. A flat launch pad with a cosine blend is kept so the
100 km/h PID can finish accelerating before the noise starts.
"""

from __future__ import annotations

import json
import math
import struct
import zlib
from pathlib import Path

import numpy as np

if __package__:
    from scripts.perlin import pnoise2_fbm
else:
    from perlin import pnoise2_fbm

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIRECTORY = ROOT / "models" / "upkie" / "high_speed"
LENGTH_M = 3000.0
WIDTH_M = 64.0
NX = 10001
NY = 257
# Unitree-style amplitude. Stretch the field across the track so both
# wheels see nearly the same height; 16 m along-track whoops stay rideable
# at 100 km/h on the 3x chassis.
HEIGHT_SCALE_M = 0.24
NEGATIVE_HEIGHT_M = 0.10
PERLIN_SMOOTH_X_M = 16.0
PERLIN_SMOOTH_Y_M = 32.0
PERLIN_OCTAVES = 3
PERLIN_PERSISTENCE = 0.5
PERLIN_LACUNARITY = 2.0
SEED = 20260917
FLAT_LAUNCH_END_X_M = -300.0
ROUGH_START_X_M = -140.0
FINISH_BLEND_START_X_M = 780.0
FINISH_END_X_M = 900.0


def generate_heightfield() -> tuple[np.ndarray, dict[str, float]]:
    x = np.linspace(-LENGTH_M / 2.0, LENGTH_M / 2.0, NX)
    y = np.linspace(-WIDTH_M / 2.0, WIDTH_M / 2.0, NY)
    dx = float(x[1] - x[0])
    dy = float(y[1] - y[0])
    grid_y, grid_x = np.meshgrid(y, x, indexing="ij")

    # Same Unitree fBm, stretched across the track so both wheels see
    # nearly the same height while the 3 m camera still reads the whoops.
    unitree_01 = (
        pnoise2_fbm(
            grid_x / PERLIN_SMOOTH_X_M,
            grid_y / PERLIN_SMOOTH_Y_M,
            octaves=PERLIN_OCTAVES,
            persistence=PERLIN_PERSISTENCE,
            lacunarity=PERLIN_LACUNARITY,
            seed=SEED,
        )
        + 1.0
    ) * 0.5

    envelope = np.ones_like(x)
    envelope[x < FLAT_LAUNCH_END_X_M] = 0.0
    transition = (x >= FLAT_LAUNCH_END_X_M) & (x < ROUGH_START_X_M)
    ratio = (x[transition] - FLAT_LAUNCH_END_X_M) / (
        ROUGH_START_X_M - FLAT_LAUNCH_END_X_M
    )
    envelope[transition] = 0.5 - 0.5 * np.cos(math.pi * ratio)
    envelope[x > FINISH_END_X_M] = 0.0
    transition = (x > FINISH_BLEND_START_X_M) & (x <= FINISH_END_X_M)
    ratio = (x[transition] - FINISH_BLEND_START_X_M) / (
        FINISH_END_X_M - FINISH_BLEND_START_X_M
    )
    envelope[transition] = 0.5 + 0.5 * np.cos(math.pi * ratio)

    # Flat pad stays mid-gray (Unitree mean height), noise ramps in with envelope.
    heights_01 = 0.5 + (unitree_01 - 0.5) * envelope[np.newaxis, :]
    heights_m = (heights_01 - 0.5) * HEIGHT_SCALE_M

    slope_y, slope_x = np.gradient(heights_m, dy, dx)
    active = (x >= ROUGH_START_X_M) & (x <= FINISH_BLEND_START_X_M)
    return heights_01, {
        "length_m": LENGTH_M,
        "width_m": WIDTH_M,
        "resolution_m": dx,
        "lateral_resolution_m": dy,
        "minimum_height_m": float(np.min(heights_m[:, active])),
        "maximum_height_m": float(np.max(heights_m[:, active])),
        "rms_height_m": float(np.sqrt(np.mean(heights_m[:, active] ** 2))),
        "maximum_slope_deg": float(
            np.degrees(
                np.arctan(
                    np.max(np.hypot(slope_x[:, active], slope_y[:, active]))
                )
            )
        ),
        "flat_launch_end_x_m": FLAT_LAUNCH_END_X_M,
        "rough_start_x_m": ROUGH_START_X_M,
        "generator": "unitree_AddPerlinHeighField",
        "perlin_smooth_x_m": PERLIN_SMOOTH_X_M,
        "perlin_smooth_y_m": PERLIN_SMOOTH_Y_M,
        "perlin_octaves": PERLIN_OCTAVES,
        "perlin_persistence": PERLIN_PERSISTENCE,
        "perlin_lacunarity": PERLIN_LACUNARITY,
        "height_scale_m": HEIGHT_SCALE_M,
    }


def _write_grayscale_png(path: Path, values: np.ndarray) -> None:
    image = np.asarray(np.round(values), dtype=np.uint8)
    height, width = image.shape
    scanlines = b"".join(b"\x00" + row.tobytes() for row in image)

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
        )

    payload = b"\x89PNG\r\n\x1a\n"
    payload += chunk(
        b"IHDR",
        struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0),
    )
    payload += chunk(b"IDAT", zlib.compress(scanlines, level=9))
    payload += chunk(b"IEND", b"")
    path.write_bytes(payload)


def generate() -> dict[str, object]:
    OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    heights_01, metrics = generate_heightfield()
    encoded = heights_01 * 255.0
    encoded[0, 0] = 0.0
    encoded[-1, -1] = 255.0
    _write_grayscale_png(OUTPUT_DIRECTORY / "heightfield.png", encoded)
    vertical_scale = HEIGHT_SCALE_M
    vertical_offset = -vertical_scale * 128.0 / 255.0
    scene = f"""<!-- Generated by scripts/generate_high_speed_track.py.
Unitree AddPerlinHeighField only: no extra box/cylinder/ellipsoid props. -->
<mujoco model="upkie_high_speed_rough_track">
  <include file="robot.xml"/>
  <visual>
    <headlight diffuse="0.18 0.18 0.17" ambient="0.16 0.15 0.14"/>
    <rgba haze="0.22 0.26 0.30 1"/>
    <quality shadowsize="4096"/>
    <global offwidth="960" offheight="540"/>
  </visual>
  <asset>
    <hfield name="high_speed_track" file="heightfield.png"
            size="1500 32 {vertical_scale:.6f} {NEGATIVE_HEIGHT_M:.2f}"/>
    <texture type="skybox" builtin="gradient" rgb1="0.46 0.58 0.70"
             rgb2="0.10 0.12 0.14" width="512" height="3072"/>
    <material name="track" rgba="0.52 0.42 0.31 1" reflectance="0"
              specular="0.28" shininess="0.06"/>
  </asset>
  <worldbody>
    <light pos="4 -18 7" dir="0.08 0.62 -0.68" directional="true"
           diffuse="1.10 1.02 0.90" specular="0.22 0.20 0.16"
           castshadow="true"/>
    <geom name="terrain" type="hfield" hfield="high_speed_track"
          pos="0 0 {vertical_offset:.9f}" material="track" group="3"
          friction="1.1 0.02 0.002" condim="3"/>
  </worldbody>
</mujoco>
"""
    (OUTPUT_DIRECTORY / "scene.xml").write_text(scene, encoding="utf-8")
    metadata = {
        "seed": SEED,
        "height_scale_m": HEIGHT_SCALE_M,
        "negative_height_m": NEGATIVE_HEIGHT_M,
        "grid": {"rows": NY, "columns": NX},
        "metrics": metrics,
        "source": (
            "unitree_mujoco/terrain_tool AddPerlinHeighField "
            "(pnoise2 octaves/persistence/lacunarity)"
        ),
    }
    (OUTPUT_DIRECTORY / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return metadata


if __name__ == "__main__":
    print(json.dumps(generate(), indent=2))

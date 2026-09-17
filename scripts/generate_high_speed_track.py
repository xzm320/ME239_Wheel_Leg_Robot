#!/usr/bin/env python3
"""Generate a long high-speed track from Unitree Perlin heightfields.

The surface follows ``AddPerlinHeighField`` in unitree_mujoco/terrain_tool:
octaved Perlin (6 octaves, persistence 0.5, lacunarity 2.0) encoded as
``(noise + 1) / 2``. A flat launch pad with a cosine blend is kept so the
100 km/h PID can finish accelerating before the noise starts. Roadside logs
stay visual-only, as in Unitree's mixed geom + hfield scenes.
"""

from __future__ import annotations

import json
import math
import struct
import zlib
from pathlib import Path

import numpy as np

if __package__:
    from scripts.perlin import unitree_perlin_meters
else:
    from perlin import unitree_perlin_meters

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIRECTORY = ROOT / "models" / "upkie" / "high_speed"
LENGTH_M = 3000.0
WIDTH_M = 64.0
NX = 10001
NY = 257
# Unitree default height_scale is 0.2 m; slightly lower so 100 km/h stays
# rideable while the 6-octave field still reads as dirt, not a flat plane.
HEIGHT_SCALE_M = 0.18
NEGATIVE_HEIGHT_M = 0.10
PERLIN_SMOOTH_M = 16.0
PERLIN_OCTAVES = 5
PERLIN_PERSISTENCE = 0.5
PERLIN_LACUNARITY = 2.0
SEED = 20260917
FLAT_LAUNCH_END_X_M = -520.0
ROUGH_START_X_M = -260.0
FINISH_BLEND_START_X_M = 780.0
FINISH_END_X_M = 900.0


def generate_heightfield() -> tuple[np.ndarray, dict[str, float]]:
    x = np.linspace(-LENGTH_M / 2.0, LENGTH_M / 2.0, NX)
    y = np.linspace(-WIDTH_M / 2.0, WIDTH_M / 2.0, NY)
    dx = float(x[1] - x[0])
    dy = float(y[1] - y[0])
    grid_y, grid_x = np.meshgrid(y, x, indexing="ij")

    # Unitree AddPerlinHeighField: (pnoise2 + 1) / 2 in [0, 1].
    unitree_01 = (
        unitree_perlin_meters(
            grid_x,
            grid_y,
            smooth_m=PERLIN_SMOOTH_M,
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
        "perlin_smooth_m": PERLIN_SMOOTH_M,
        "perlin_octaves": PERLIN_OCTAVES,
        "perlin_persistence": PERLIN_PERSISTENCE,
        "perlin_lacunarity": PERLIN_LACUNARITY,
        "height_scale_m": HEIGHT_SCALE_M,
    }


def generate_scenery_props() -> list[dict[str, object]]:
    """Unitree-style logs and dirt piles kept off the 4x wheel track."""

    rng = np.random.default_rng(SEED + 17)
    props: list[dict[str, object]] = []
    x = -230.0
    index = 0
    while x < 740.0:
        side = 1.0 if index % 2 == 0 else -1.0
        y = side * float(rng.uniform(2.3, 4.2))
        radius = float(rng.uniform(0.16, 0.24))
        half_length = float(rng.uniform(1.1, 1.8))
        props.append(
            {
                "name": f"scenery_log_{index:02d}",
                "type": "cylinder",
                "position": [float(x), y, radius],
                "size": [radius, half_length],
                "quaternion": [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0],
                "rgba": "0.78 0.42 0.16 1",
            }
        )
        mound_x = x + float(rng.uniform(2.4, 4.2))
        mound_y = -side * float(rng.uniform(2.4, 4.0))
        props.append(
            {
                "name": f"scenery_mound_{index:02d}",
                "type": "ellipsoid",
                "position": [mound_x, mound_y, 0.055],
                "size": [
                    float(rng.uniform(0.90, 1.45)),
                    float(rng.uniform(0.55, 0.95)),
                    float(rng.uniform(0.08, 0.13)),
                ],
                "quaternion": [1.0, 0.0, 0.0, 0.0],
                "rgba": "0.62 0.38 0.16 1",
            }
        )
        x += float(rng.uniform(14.0, 22.0))
        index += 1
    return props


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


def _prop_xml(props: list[dict[str, object]]) -> str:
    lines = []
    for prop in props:
        position = " ".join(f"{value:.6f}" for value in prop["position"])
        size = " ".join(f"{value:.6f}" for value in prop["size"])
        quaternion = " ".join(f"{value:.8f}" for value in prop["quaternion"])
        lines.append(
            f'    <geom name="{prop["name"]}" type="{prop["type"]}" '
            f'pos="{position}" size="{size}" quat="{quaternion}" '
            f'group="3" rgba="{prop["rgba"]}" contype="0" conaffinity="0"/>'
        )
    return "\n".join(lines)


def generate() -> dict[str, object]:
    OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    heights_01, metrics = generate_heightfield()
    props = generate_scenery_props()
    encoded = heights_01 * 255.0
    encoded[0, 0] = 0.0
    encoded[-1, -1] = 255.0
    _write_grayscale_png(OUTPUT_DIRECTORY / "heightfield.png", encoded)
    vertical_scale = HEIGHT_SCALE_M
    vertical_offset = -vertical_scale * 128.0 / 255.0
    scene = f"""<!-- Generated by scripts/generate_high_speed_track.py.
Unitree AddPerlinHeighField: octaves={PERLIN_OCTAVES},
smooth={PERLIN_SMOOTH_M} m, persistence={PERLIN_PERSISTENCE},
lacunarity={PERLIN_LACUNARITY}, height_scale={HEIGHT_SCALE_M} m. -->
<mujoco model="upkie_high_speed_rough_track">
  <include file="robot.xml"/>
  <visual>
    <headlight diffuse="0.72 0.70 0.66" ambient="0.16 0.18 0.22"/>
    <rgba haze="0.16 0.22 0.28 1"/>
    <quality shadowsize="4096"/>
    <global offwidth="960" offheight="540"/>
  </visual>
  <asset>
    <hfield name="high_speed_track" file="heightfield.png"
            size="1500 32 {vertical_scale:.6f} {NEGATIVE_HEIGHT_M:.2f}"/>
    <texture type="skybox" builtin="gradient" rgb1="0.40 0.56 0.74"
             rgb2="0.05 0.06 0.08" width="512" height="3072"/>
    <texture type="2d" name="track_grid" builtin="checker" mark="edge"
             rgb1="0.66 0.50 0.26" rgb2="0.24 0.16 0.08"
             markrgb="0.94 0.82 0.50" width="512" height="512"/>
    <material name="track" texture="track_grid" texuniform="true"
              texrepeat="36 4" reflectance="0.03"/>
  </asset>
  <worldbody>
    <light pos="10 -20 3.2" dir="-0.18 0.82 -0.42" directional="true"
           diffuse="1.05 0.90 0.68" specular="0.55 0.42 0.28"
           castshadow="true"/>
    <light pos="-14 4 9" dir="0.28 -0.08 -1" directional="true"
           diffuse="0.22 0.26 0.32"/>
    <geom name="terrain" type="hfield" hfield="high_speed_track"
          pos="0 0 {vertical_offset:.9f}" material="track" group="3"
          friction="1.1 0.02 0.002" condim="3"/>
{_prop_xml(props)}
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
        "scenery_prop_count": len(props),
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

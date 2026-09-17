#!/usr/bin/env python3
"""Generate a long high-speed track with visible 2D roughness.

The old 30 mm, 3.5–8 m extruded profile looked flat from a 4 m camera, and a
1.55 m washboard at 100 km/h is an 18 Hz pitch hammer. This generator follows
the usual robotics-sim mix:

- Isaac Lab / ANYmal-style multi-octave 2D noise (not a 1D extrusion)
- Motocross-style isolated cosine whoops for a readable ground silhouette
- Sparse 2D Gaussian dirt piles
- Unitree-style off-line ellipsoid/log props that catch raking light
- A long cosine blend off a flat launch pad
"""

from __future__ import annotations

import json
import math
import struct
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIRECTORY = ROOT / "models" / "upkie" / "high_speed"
LENGTH_M = 3000.0
WIDTH_M = 64.0
NX = 10001
NY = 257
AMPLITUDE_M = 0.16
SEED = 9144


def _gaussian_kernel(sigma_samples: float) -> np.ndarray:
    radius = max(2, round(4.0 * sigma_samples))
    positions = np.arange(-radius, radius + 1)
    kernel = np.exp(-0.5 * (positions / max(sigma_samples, 1e-6)) ** 2)
    kernel /= np.sum(kernel)
    return kernel


def _smooth_1d(signal: np.ndarray, sigma_samples: float) -> np.ndarray:
    kernel = _gaussian_kernel(sigma_samples)
    radius = kernel.size // 2
    padded = np.pad(signal, radius, mode="reflect")
    filtered = np.convolve(padded, kernel, mode="valid")
    filtered -= float(np.mean(filtered))
    peak = float(np.max(np.abs(filtered)))
    if peak > 0.0:
        filtered /= peak
    return filtered


def _smooth_2d(
    field: np.ndarray, sigma_x_samples: float, sigma_y_samples: float
) -> np.ndarray:
    kernel_x = _gaussian_kernel(sigma_x_samples)
    kernel_y = _gaussian_kernel(sigma_y_samples)
    radius_x = kernel_x.size // 2
    radius_y = kernel_y.size // 2
    padded_x = np.pad(field, ((0, 0), (radius_x, radius_x)), mode="reflect")
    along_x = np.apply_along_axis(
        lambda row: np.convolve(row, kernel_x, mode="valid"), 1, padded_x
    )
    padded_y = np.pad(along_x, ((radius_y, radius_y), (0, 0)), mode="reflect")
    blurred = np.apply_along_axis(
        lambda column: np.convolve(column, kernel_y, mode="valid"), 0, padded_y
    )
    blurred -= float(np.mean(blurred))
    peak = float(np.max(np.abs(blurred)))
    if peak > 0.0:
        blurred /= peak
    return blurred


def _cosine_pulse(delta: np.ndarray, length_m: float) -> np.ndarray:
    half = 0.5 * length_m
    pulse = np.zeros_like(delta)
    inside = np.abs(delta) <= half
    pulse[inside] = 0.5 * (1.0 + np.cos(math.pi * delta[inside] / half))
    return pulse


def generate_heightfield() -> tuple[np.ndarray, dict[str, float]]:
    rng = np.random.default_rng(SEED)
    x = np.linspace(-LENGTH_M / 2.0, LENGTH_M / 2.0, NX)
    y = np.linspace(-WIDTH_M / 2.0, WIDTH_M / 2.0, NY)
    dx = float(x[1] - x[0])
    dy = float(y[1] - y[0])
    grid_y, grid_x = np.meshgrid(y, x, indexing="ij")

    # Multi-octave 2D roughness so left and right wheels see different ground.
    hills = 0.024 * _smooth_2d(
        rng.normal(size=(NY, NX)), 8.0 / dx, 6.0 / dy
    )
    medium = 0.014 * _smooth_2d(
        rng.normal(size=(NY, NX)), 3.2 / dx, 2.5 / dy
    )
    ripple = 0.007 * _smooth_2d(
        rng.normal(size=(NY, NX)), 1.3 / dx, 1.2 / dy
    )
    heights = hills + medium + ripple

    # Isolated cosine whoops: readable silhouette without an 18 Hz washboard.
    whoop_x = -250.0
    whoop_index = 0
    while whoop_x < 760.0:
        length = float(rng.uniform(3.2, 4.8))
        gap = float(rng.uniform(5.8, 8.6))
        height = float(rng.uniform(0.070, 0.108))
        if whoop_index == 0:
            height *= 0.45
        elif whoop_index == 1:
            height *= 0.68
        elif whoop_index == 2:
            height *= 0.85
        tilt = float(rng.uniform(-0.008, 0.008))
        pulse = _cosine_pulse(grid_x - whoop_x, length)
        lateral = 1.0 + 0.10 * np.sin(
            2.0 * math.pi * grid_y / 9.0 + rng.uniform(0.0, 6.0)
        )
        heights += pulse * (height + tilt * grid_y) * lateral
        whoop_x += 0.5 * length + gap
        whoop_index += 1

    # Rounded 2D dirt piles on and beside the racing line.
    for _ in range(48):
        center_x = rng.uniform(-220.0, 740.0)
        center_y = rng.uniform(-10.0, 10.0)
        height = rng.uniform(0.028, 0.070)
        width_x = rng.uniform(2.0, 3.8)
        width_y = rng.uniform(1.4, 2.8)
        heights += height * np.exp(
            -0.5
            * (
                ((grid_x - center_x) / width_x) ** 2
                + ((grid_y - center_y) / width_y) ** 2
            )
        )

    envelope = np.ones_like(x)
    envelope[x < -500.0] = 0.0
    transition = (x >= -500.0) & (x < -300.0)
    ratio = (x[transition] + 500.0) / 200.0
    envelope[transition] = 0.5 - 0.5 * np.cos(math.pi * ratio)
    envelope[x > 900.0] = 0.0
    transition = (x > 780.0) & (x <= 900.0)
    ratio = (x[transition] - 780.0) / 120.0
    envelope[transition] = 0.5 + 0.5 * np.cos(math.pi * ratio)
    heights *= envelope[np.newaxis, :]
    heights = AMPLITUDE_M * np.tanh(heights / 0.11)

    slope_y, slope_x = np.gradient(heights, dy, dx)
    active = (x >= -300.0) & (x <= 760.0)
    return heights, {
        "length_m": LENGTH_M,
        "width_m": WIDTH_M,
        "resolution_m": dx,
        "lateral_resolution_m": dy,
        "minimum_height_m": float(np.min(heights[:, active])),
        "maximum_height_m": float(np.max(heights[:, active])),
        "rms_height_m": float(np.sqrt(np.mean(heights[:, active] ** 2))),
        "maximum_slope_deg": float(
            np.degrees(
                np.arctan(
                    np.max(np.hypot(slope_x[:, active], slope_y[:, active]))
                )
            )
        ),
        "flat_launch_end_x_m": -500.0,
        "rough_start_x_m": -300.0,
        "whoop_start_x_m": -250.0,
        "whoop_count": whoop_index,
    }


def generate_scenery_props() -> list[dict[str, object]]:
    """Unitree-style logs and dirt piles kept off the 4x wheel track."""

    rng = np.random.default_rng(SEED + 17)
    props: list[dict[str, object]] = []
    x = -230.0
    index = 0
    while x < 740.0:
        side = 1.0 if index % 2 == 0 else -1.0
        y = side * float(rng.uniform(5.4, 9.6))
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
        mound_y = -side * float(rng.uniform(4.8, 8.8))
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
    heights, metrics = generate_heightfield()
    props = generate_scenery_props()
    encoded = (heights / (2.0 * AMPLITUDE_M) + 0.5) * 255.0
    encoded[0, 0] = 0.0
    encoded[-1, -1] = 255.0
    _write_grayscale_png(OUTPUT_DIRECTORY / "heightfield.png", encoded)
    vertical_scale = 2.0 * AMPLITUDE_M
    vertical_offset = -vertical_scale * 128.0 / 255.0
    scene = f"""<!-- Generated by scripts/generate_high_speed_track.py. -->
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
            size="1500 32 {vertical_scale:.6f} 0.10"/>
    <texture type="skybox" builtin="gradient" rgb1="0.40 0.56 0.74"
             rgb2="0.05 0.06 0.08" width="512" height="3072"/>
    <texture type="2d" name="track_grid" builtin="checker" mark="edge"
             rgb1="0.66 0.50 0.26" rgb2="0.24 0.16 0.08"
             markrgb="0.94 0.82 0.50" width="512" height="512"/>
    <material name="track" texture="track_grid" texuniform="true"
              texrepeat="420 28" reflectance="0.03"/>
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
        "amplitude_limit_m": AMPLITUDE_M,
        "grid": {"rows": NY, "columns": NX},
        "metrics": metrics,
        "scenery_prop_count": len(props),
    }
    (OUTPUT_DIRECTORY / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return metadata


if __name__ == "__main__":
    print(json.dumps(generate(), indent=2))

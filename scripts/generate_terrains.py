#!/usr/bin/env python3
"""Generate deterministic graded heightfields and MuJoCo scenes."""

from __future__ import annotations

import json
import math
import struct
import zlib
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIRECTORY = ROOT / "models" / "upkie" / "terrain"
LENGTH_M = 20.0
WIDTH_M = 4.0
NX = 801
NY = 161


@dataclass(frozen=True)
class TerrainSpec:
    name: str
    seed: int
    amplitude_m: float
    correlation_length_m: float
    bump_count: int
    bump_width_m: float
    obstacle_count: int
    transverse_bump_count: int
    maximum_obstacle_height_m: float


SPECS = (
    TerrainSpec("easy", 1201, 0.020, 0.45, 4, 0.25, 4, 1, 0.040),
    TerrainSpec("medium", 2302, 0.055, 0.28, 10, 0.18, 8, 2, 0.080),
    TerrainSpec("hard", 3403, 0.100, 0.18, 18, 0.12, 14, 3, 0.140),
    TerrainSpec("extreme", 4504, 0.140, 0.14, 24, 0.10, 20, 4, 0.200),
)


def _smooth_noise(
    rng: np.random.Generator, sample_count: int, sigma_samples: float
) -> np.ndarray:
    radius = max(2, round(4.0 * sigma_samples))
    positions = np.arange(-radius, radius + 1)
    kernel = np.exp(-0.5 * (positions / sigma_samples) ** 2)
    kernel /= np.sum(kernel)
    padding = radius
    noise = rng.normal(size=sample_count + 2 * padding)
    filtered = np.convolve(noise, kernel, mode="same")
    result = filtered[padding:-padding]
    result -= np.mean(result)
    result /= max(float(np.std(result)), 1e-9)
    return result


def generate_heightfield(spec: TerrainSpec) -> tuple[np.ndarray, dict[str, float]]:
    rng = np.random.default_rng(spec.seed)
    x = np.linspace(-LENGTH_M / 2, LENGTH_M / 2, NX)
    y = np.linspace(-WIDTH_M / 2, WIDTH_M / 2, NY)
    dx = float(x[1] - x[0])
    dy = float(y[1] - y[0])
    sigma_samples = spec.correlation_length_m / dx

    longitudinal = _smooth_noise(rng, NX, sigma_samples)
    secondary = _smooth_noise(rng, NX, sigma_samples * 0.65)
    lateral_phase = rng.uniform(0.0, 2.0 * math.pi)
    roughness = (
        longitudinal[np.newaxis, :]
        * (1.0 + 0.16 * np.sin(2.0 * math.pi * y[:, np.newaxis] / WIDTH_M + lateral_phase))
        + 0.22
        * secondary[np.newaxis, :]
        * np.sin(4.0 * math.pi * y[:, np.newaxis] / WIDTH_M + lateral_phase)
    )

    grid_y, grid_x = np.meshgrid(y, x, indexing="ij")
    for _ in range(spec.bump_count):
        center_x = rng.uniform(-4.8, 7.5)
        center_y = rng.uniform(-1.35, 1.35)
        bump_height = rng.uniform(-1.0, 1.0)
        width_x = spec.bump_width_m * rng.uniform(0.8, 1.5)
        width_y = spec.bump_width_m * rng.uniform(0.8, 1.8)
        roughness += bump_height * np.exp(
            -0.5
            * (
                ((grid_x - center_x) / width_x) ** 2
                + ((grid_y - center_y) / width_y) ** 2
            )
        )

    # Flat launch and braking zones with cosine transitions.
    envelope = np.ones_like(x)
    envelope[x <= -7.2] = 0.0
    launch_transition = (x > -7.2) & (x < -5.8)
    launch_ratio = (x[launch_transition] + 7.2) / 1.4
    envelope[launch_transition] = 0.5 - 0.5 * np.cos(math.pi * launch_ratio)
    envelope[x >= 9.2] = 0.0
    finish_transition = (x > 7.8) & (x < 9.2)
    finish_ratio = (x[finish_transition] - 7.8) / 1.4
    envelope[finish_transition] = 0.5 + 0.5 * np.cos(math.pi * finish_ratio)
    roughness *= envelope[np.newaxis, :]

    active = (x >= -5.8) & (x <= 7.8)
    active_peak = float(np.max(np.abs(roughness[:, active])))
    roughness *= spec.amplitude_m / max(active_peak, 1e-9)
    roughness = np.clip(roughness, -spec.amplitude_m, spec.amplitude_m)

    slope_y, slope_x = np.gradient(roughness, dy, dx)
    active_values = roughness[:, active]
    active_slopes = np.hypot(slope_x[:, active], slope_y[:, active])
    metrics = {
        "minimum_height_m": float(np.min(active_values)),
        "maximum_height_m": float(np.max(active_values)),
        "rms_height_m": float(np.sqrt(np.mean(active_values**2))),
        "maximum_slope_deg": float(np.degrees(np.arctan(np.max(active_slopes)))),
        "longitudinal_resolution_m": dx,
        "lateral_resolution_m": dy,
        "flat_launch_end_x_m": -7.2,
        "rough_section_start_x_m": -5.8,
        "rough_section_end_x_m": 7.8,
    }
    return roughness, metrics


def _terrain_height(heights: np.ndarray, x_m: float, y_m: float) -> float:
    column = round((x_m + LENGTH_M / 2) / LENGTH_M * (NX - 1))
    row = round((y_m + WIDTH_M / 2) / WIDTH_M * (NY - 1))
    return float(heights[row, column])


def _euler_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def generate_obstacles(
    spec: TerrainSpec, heights: np.ndarray
) -> list[dict[str, object]]:
    """Generate Unitree-style explicit logs and randomized box protrusions."""
    rng = np.random.default_rng(spec.seed + 9000)
    obstacles: list[dict[str, object]] = []

    transverse_x = np.linspace(-4.5, 6.5, spec.transverse_bump_count)
    for index, nominal_x in enumerate(transverse_x):
        x_m = float(nominal_x + rng.uniform(-0.20, 0.20))
        obstacle_height = float(
            rng.uniform(
                0.65 * spec.maximum_obstacle_height_m,
                spec.maximum_obstacle_height_m,
            )
        )
        radius = obstacle_height / 2.0
        half_length = float(rng.uniform(0.70, 1.15))
        ground_height = _terrain_height(heights, x_m, 0.0)
        obstacles.append(
            {
                "name": f"{spec.name}_transverse_log_{index:02d}",
                "type": "cylinder",
                "position": [x_m, 0.0, ground_height + radius],
                "size": [radius, half_length],
                "quaternion": [
                    math.sqrt(0.5),
                    math.sqrt(0.5),
                    0.0,
                    0.0,
                ],
                "height_m": obstacle_height,
            }
        )

    remaining = spec.obstacle_count - spec.transverse_bump_count
    for index in range(remaining):
        x_m = float(rng.uniform(-4.9, 7.4))
        y_m = float(rng.uniform(-1.15, 1.15))
        height = float(
            rng.uniform(
                0.30 * spec.maximum_obstacle_height_m,
                0.85 * spec.maximum_obstacle_height_m,
            )
        )
        size_x = float(rng.uniform(0.10, 0.30))
        size_y = float(rng.uniform(0.12, 0.38))
        roll = float(rng.uniform(-0.16, 0.16))
        pitch = float(rng.uniform(-0.20, 0.20))
        yaw = float(rng.uniform(-0.45, 0.45))
        ground_height = _terrain_height(heights, x_m, y_m)
        obstacles.append(
            {
                "name": f"{spec.name}_rock_box_{index:02d}",
                "type": "box",
                "position": [x_m, y_m, ground_height + height / 2.0],
                "size": [size_x / 2.0, size_y / 2.0, height / 2.0],
                "quaternion": list(_euler_to_quaternion(roll, pitch, yaw)),
                "height_m": height,
            }
        )
    return obstacles


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
        b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    )
    payload += chunk(b"IDAT", zlib.compress(scanlines, level=9))
    payload += chunk(b"IEND", b"")
    path.write_bytes(payload)


def _scene_xml(spec: TerrainSpec, obstacles: list[dict[str, object]]) -> str:
    vertical_scale = 2.0 * spec.amplitude_m
    # Flat terrain encodes to grayscale 128 after rounding.
    vertical_offset = -vertical_scale * 128.0 / 255.0
    obstacle_xml = "\n".join(
        (
            f'    <geom name="{obstacle["name"]}" type="{obstacle["type"]}" '
            f'pos="{" ".join(f"{value:.6f}" for value in obstacle["position"])}" '
            f'size="{" ".join(f"{value:.6f}" for value in obstacle["size"])}" '
            f'quat="{" ".join(f"{value:.8f}" for value in obstacle["quaternion"])}" '
            'rgba="0.48 0.34 0.22 1" friction="1.2 0.02 0.002" condim="3"/>'
        )
        for obstacle in obstacles
    )
    return f"""<!-- Deterministically generated by scripts/generate_terrains.py.
Terrain strategy follows Unitree's public terrain_tool pattern: a heightfield
combined with explicit randomized box and cylinder geometries. -->
<mujoco model="upkie_{spec.name}_terrain">
  <include file="../four_bar/robot.xml"/>
  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35"/>
    <rgba haze="0.14 0.20 0.28 1"/>
    <global offwidth="960" offheight="540" azimuth="120" elevation="-18"/>
  </visual>
  <asset>
    <hfield name="{spec.name}_heightfield" file="heightfields/{spec.name}.png"
            size="{LENGTH_M / 2:.3f} {WIDTH_M / 2:.3f} {vertical_scale:.5f} 0.10"/>
    <texture type="skybox" builtin="gradient" rgb1="0.38 0.52 0.70"
             rgb2="0.04 0.05 0.07" width="512" height="3072"/>
    <texture type="2d" name="{spec.name}_grid" builtin="checker" mark="edge"
             rgb1="0.26 0.30 0.22" rgb2="0.12 0.16 0.11"
             markrgb="0.75 0.80 0.65" width="512" height="512"/>
    <material name="{spec.name}_ground" texture="{spec.name}_grid"
              texuniform="true" texrepeat="20 4" reflectance="0.08"/>
  </asset>
  <worldbody>
    <light pos="-2 -2 4" dir="0.2 0.2 -1" directional="true"/>
    <geom name="terrain" type="hfield" hfield="{spec.name}_heightfield"
          pos="0 0 {vertical_offset:.8f}" material="{spec.name}_ground"
          friction="1.1 0.02 0.002" condim="3"/>
{obstacle_xml}
  </worldbody>
</mujoco>
"""


def generate_all() -> dict[str, object]:
    heightfield_directory = OUTPUT_DIRECTORY / "heightfields"
    heightfield_directory.mkdir(parents=True, exist_ok=True)
    metadata: dict[str, object] = {
        "generator": "scripts/generate_terrains.py",
        "length_m": LENGTH_M,
        "width_m": WIDTH_M,
        "grid": {"columns": NX, "rows": NY},
        "encoding": {
            "flat_grayscale": 128,
            "calibration_pixels": [[0, 0], [NY - 1, NX - 1]],
        },
        "terrains": {},
    }

    for spec in SPECS:
        heights, metrics = generate_heightfield(spec)
        obstacles = generate_obstacles(spec, heights)
        encoded = (heights / (2.0 * spec.amplitude_m) + 0.5) * 255.0
        # MuJoCo normalizes an image heightfield by its extrema. Reserve two
        # far-corner pixels so every level preserves the intended metric scale.
        encoded[0, 0] = 0.0
        encoded[-1, -1] = 255.0
        _write_grayscale_png(heightfield_directory / f"{spec.name}.png", encoded)
        (OUTPUT_DIRECTORY / f"scene_{spec.name}.xml").write_text(
            _scene_xml(spec, obstacles), encoding="utf-8"
        )
        metadata["terrains"][spec.name] = {
            "specification": asdict(spec),
            "metrics": metrics,
            "obstacles": obstacles,
        }

    metadata_path = OUTPUT_DIRECTORY / "metadata.json"
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metadata


if __name__ == "__main__":
    print(json.dumps(generate_all(), indent=2))

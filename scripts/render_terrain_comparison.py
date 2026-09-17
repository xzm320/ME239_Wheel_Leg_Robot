#!/usr/bin/env python3
"""Render the robot standing over each graded rough-terrain level."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

if __package__:
    from scripts.generate_terrains import (
        LENGTH_M,
        NX,
        NY,
        OUTPUT_DIRECTORY,
        SPECS,
        WIDTH_M,
        generate_heightfield,
    )
else:
    from generate_terrains import (
        LENGTH_M,
        NX,
        NY,
        OUTPUT_DIRECTORY,
        SPECS,
        WIDTH_M,
        generate_heightfield,
    )


def _write_ppm(path: Path, pixels: np.ndarray) -> None:
    height, width, _ = pixels.shape
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels.tobytes())


def _terrain_height(
    heights: np.ndarray, x_m: float, y_m: float
) -> float:
    column = round((x_m + LENGTH_M / 2) / LENGTH_M * (NX - 1))
    row = round((y_m + WIDTH_M / 2) / WIDTH_M * (NY - 1))
    return float(heights[row, column])


def render(output_directory: Path) -> Path:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to compose the terrain image")
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / "rough_terrain_obstacles_v2.png"
    metadata = json.loads((OUTPUT_DIRECTORY / "metadata.json").read_text())

    with tempfile.TemporaryDirectory(prefix="terrain_comparison_") as temporary:
        temporary_directory = Path(temporary)
        labelled_paths: list[Path] = []

        for spec in SPECS:
            model = mujoco.MjModel.from_xml_path(
                str(OUTPUT_DIRECTORY / f"scene_{spec.name}.xml")
            )
            heights, _ = generate_heightfield(spec)
            recorded = metadata["terrains"][spec.name]
            transverse_obstacle = next(
                obstacle
                for obstacle in recorded["obstacles"]
                if "transverse_log" in obstacle["name"]
            )
            robot_x = float(transverse_obstacle["position"][0]) - 0.42
            support_height = max(
                _terrain_height(heights, robot_x, -0.1137),
                _terrain_height(heights, robot_x, 0.1137),
            )
            data = mujoco.MjData(model)
            data.qpos[:7] = (
                robot_x,
                0.0,
                0.408 + support_height,
                1.0,
                0.0,
                0.0,
                0.0,
            )
            mujoco.mj_forward(model, data)

            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = (robot_x + 0.05, 0.0, 0.18)
            camera.distance = 1.35
            camera.azimuth = 132
            camera.elevation = -9

            with mujoco.Renderer(model, height=480, width=640) as renderer:
                renderer.update_scene(data, camera=camera)
                pixels = renderer.render()
            assert np.std(pixels) > 1.0
            raw_path = temporary_directory / f"{spec.name}.ppm"
            _write_ppm(raw_path, pixels)

            metrics = recorded["metrics"]
            maximum_obstacle_height = max(
                obstacle["height_m"] for obstacle in recorded["obstacles"]
            )
            label = (
                f"{spec.name.upper()}  RMS {metrics['rms_height_m'] * 1000:.1f} mm"
                f"  BUMP {maximum_obstacle_height * 1000:.0f} mm"
            )
            labelled_path = temporary_directory / f"{spec.name}.png"
            font = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
            subprocess.run(
                [
                    "ffmpeg",
                    "-y",
                    "-loglevel",
                    "error",
                    "-i",
                    str(raw_path),
                    "-vf",
                    (
                        f"drawtext=fontfile={font}:text='{label}':"
                        "x=(w-text_w)/2:y=22:fontsize=23:fontcolor=white:"
                        "box=1:boxcolor=black@0.6"
                    ),
                    "-frames:v",
                    "1",
                    str(labelled_path),
                ],
                check=True,
            )
            labelled_paths.append(labelled_path)

        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                *(item for path in labelled_paths for item in ("-i", str(path))),
                "-filter_complex",
                f"hstack=inputs={len(labelled_paths)}",
                "-frames:v",
                "1",
                str(output_path),
            ],
            check=True,
        )

    print(output_path)
    return output_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=Path("artifacts"),
    )
    arguments = parser.parse_args()
    render(arguments.output_directory)

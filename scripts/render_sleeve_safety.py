#!/usr/bin/env python3
"""Render neutral and maximum-extension telescopic sleeve engagement."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "upkie"
    / "four_bar"
    / "scene.xml"
)


def _write_ppm(path: Path, pixels: np.ndarray) -> None:
    height, width, _ = pixels.shape
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels.tobytes())


def _id(model: mujoco.MjModel, object_type: int, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return object_id


def _render_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera: mujoco.MjvCamera,
    path: Path,
) -> None:
    with mujoco.Renderer(model, height=500, width=800) as renderer:
        renderer.update_scene(data, camera=camera)
        _write_ppm(path, renderer.render())


def render(output_directory: Path) -> Path:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to compose the sleeve image")
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / "telescopic_sleeve_engagement_v2.png"

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    data.qpos[:7] = (0.0, 0.0, 0.80, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.0, -0.1137, 0.686)
    camera.distance = 0.56
    camera.azimuth = 128
    camera.elevation = -4

    actuator_ids = [
        _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{side}_strut")
        for side in ("left", "right")
    ]

    with tempfile.TemporaryDirectory(prefix="sleeve_safety_") as temporary:
        temporary_directory = Path(temporary)
        neutral_path = temporary_directory / "neutral.ppm"
        maximum_path = temporary_directory / "maximum.ppm"
        _render_state(model, data, camera, neutral_path)

        for step in range(round(1.0 / model.opt.timestep)):
            ramp = min(step * model.opt.timestep / 0.35, 1.0)
            data.ctrl[actuator_ids] = 0.1316 * ramp
            mujoco.mj_step(model, data)
        _render_state(model, data, camera, maximum_path)

        font = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        labelled_paths: list[Path] = []
        for source, label, destination in (
            (neutral_path, "NESTED TWO-STAGE - NEUTRAL", "neutral.png"),
            (maximum_path, "188 mm EXTENSION - 26 mm MIN INSERTION", "maximum.png"),
        ):
            labelled = temporary_directory / destination
            subprocess.run(
                [
                    "ffmpeg",
                    "-y",
                    "-loglevel",
                    "error",
                    "-i",
                    str(source),
                    "-vf",
                    (
                        f"drawtext=fontfile={font}:text='{label}':"
                        "x=(w-text_w)/2:y=22:fontsize=28:fontcolor=white:"
                        "box=1:boxcolor=black@0.65"
                    ),
                    "-frames:v",
                    "1",
                    str(labelled),
                ],
                check=True,
            )
            labelled_paths.append(labelled)

        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-i",
                str(labelled_paths[0]),
                "-i",
                str(labelled_paths[1]),
                "-filter_complex",
                "hstack=inputs=2",
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

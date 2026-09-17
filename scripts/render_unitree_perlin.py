#!/usr/bin/env python3
"""Render the Unitree-scale Perlin patch the way the terrain tool screenshot does."""

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
from PIL import Image

if __package__:
    from scripts.generate_high_speed_track import (
        HEIGHT_SCALE_M,
        IMAGE_PX,
        PATCH_SIZE_X_M,
        PATCH_SIZE_Y_M,
        generate_heightfield,
    )
    from scripts.high_speed_evaluation import (
        HUNDRED_KMH_WHEEL_TRACK_SCALE,
        MODEL_PATH,
        apply_hundred_kmh_suspension,
        apply_wheel_track_scale,
    )
    from scripts.joint_terrain_controller import (
        HUNDRED_KMH_COMPLIANCE,
        apply_compliance_parameters,
    )
else:
    from generate_high_speed_track import (
        HEIGHT_SCALE_M,
        IMAGE_PX,
        PATCH_SIZE_X_M,
        PATCH_SIZE_Y_M,
        generate_heightfield,
    )
    from high_speed_evaluation import (
        HUNDRED_KMH_WHEEL_TRACK_SCALE,
        MODEL_PATH,
        apply_hundred_kmh_suspension,
        apply_wheel_track_scale,
    )
    from joint_terrain_controller import (
        HUNDRED_KMH_COMPLIANCE,
        apply_compliance_parameters,
    )

FPS = 24
ORBIT_DURATION_S = 5.0
WIDTH = 960
HEIGHT = 540


def _prepare_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    apply_compliance_parameters(model, HUNDRED_KMH_COMPLIANCE)
    apply_wheel_track_scale(model, HUNDRED_KMH_WHEEL_TRACK_SCALE)
    apply_hundred_kmh_suspension(model)
    model.stat.extent = 2.4
    model.vis.map.znear = 0.01
    model.vis.map.zfar = 20.0
    data = mujoco.MjData(model)
    return model, data


def _patch_height_m(x_m: float, y_m: float) -> float:
    heights_01, _ = generate_heightfield()
    col = np.clip(
        (x_m + PATCH_SIZE_X_M / 2.0) / PATCH_SIZE_X_M * (IMAGE_PX - 1),
        0.0,
        IMAGE_PX - 1,
    )
    # MuJoCo maps image row 0 to +Y.
    row = np.clip(
        (PATCH_SIZE_Y_M / 2.0 - y_m) / PATCH_SIZE_Y_M * (IMAGE_PX - 1),
        0.0,
        IMAGE_PX - 1,
    )
    r0, c0 = int(row), int(col)
    r1 = min(r0 + 1, IMAGE_PX - 1)
    c1 = min(c0 + 1, IMAGE_PX - 1)
    wr, wc = row - r0, col - c0
    sample = (
        heights_01[r0, c0] * (1 - wr) * (1 - wc)
        + heights_01[r0, c1] * (1 - wr) * wc
        + heights_01[r1, c0] * wr * (1 - wc)
        + heights_01[r1, c1] * wr * wc
    )
    return float(sample * HEIGHT_SCALE_M)


def _place_robot(
    model: mujoco.MjModel, data: mujoco.MjData, x_m: float, y_m: float
) -> None:
    ground = _patch_height_m(x_m, y_m)
    data.qpos[:7] = (x_m, y_m, 0.408 + ground, 1.0, 0.0, 0.0, 0.0)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def _hide_robot(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    data.qpos[:7] = (-12.0, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def _scene_option() -> mujoco.MjvOption:
    option = mujoco.MjvOption()
    option.geomgroup[3] = 1
    return option


def _camera(*, distance: float, azimuth: float, elevation: float) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = distance
    camera.azimuth = azimuth
    camera.elevation = elevation
    camera.lookat[:] = (0.0, 0.0, 0.07)
    return camera


def _render_frame(
    renderer: mujoco.Renderer,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera: mujoco.MjvCamera,
    *,
    shadows: bool = False,
) -> np.ndarray:
    renderer.update_scene(data, camera=camera, scene_option=_scene_option())
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = shadows
    return renderer.render()


def render_stills(output_directory: Path) -> dict[str, Path]:
    output_directory.mkdir(parents=True, exist_ok=True)
    model, data = _prepare_model()
    mujoco.mj_forward(model, data)
    paths: dict[str, Path] = {}
    cameras = {
        "unitree_perlin.png": _camera(distance=1.78, azimuth=148, elevation=-48),
        "unitree_perlin_side.png": _camera(distance=2.35, azimuth=122, elevation=-30),
    }
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        _hide_robot(model, data)
        for name, camera in cameras.items():
            path = output_directory / name
            Image.fromarray(_render_frame(renderer, model, data, camera)).save(path)
            paths[name] = path

        _place_robot(model, data, 0.05, 0.0)
        follow = _camera(distance=2.40, azimuth=132, elevation=-34)
        robot_path = output_directory / "unitree_perlin_robot.png"
        Image.fromarray(_render_frame(renderer, model, data, follow, shadows=True)).save(robot_path)
        paths[robot_path.name] = robot_path
    return paths


def render_orbit(output_directory: Path) -> Path:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to encode the demonstration")
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / "unitree_perlin_orbit.mp4"
    model, data = _prepare_model()
    _hide_robot(model, data)
    frame_count = int(ORBIT_DURATION_S * FPS)
    camera = _camera(distance=1.88, azimuth=120.0, elevation=-46.0)
    with tempfile.TemporaryDirectory(prefix="unitree_perlin_") as temporary:
        frame_directory = Path(temporary)
        with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
            for index in range(frame_count):
                camera.azimuth = 120.0 + 75.0 * index / max(frame_count - 1, 1)
                pixels = _render_frame(renderer, model, data, camera)
                Image.fromarray(pixels).save(frame_directory / f"frame_{index:04d}.ppm")
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-framerate",
                str(FPS),
                "-i",
                str(frame_directory / "frame_%04d.ppm"),
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(output_path),
            ],
            check=True,
        )
    return output_path


def render(output_directory: Path) -> dict[str, Path]:
    stills = render_stills(output_directory)
    video = render_orbit(output_directory)
    stills[video.name] = video
    for path in stills.values():
        print(path)
    return stills


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-directory", type=Path, default=Path("artifacts"))
    arguments = parser.parse_args()
    render(arguments.output_directory)

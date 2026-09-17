#!/usr/bin/env python3
"""Render joint control traversing the medium rough-terrain course."""

from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

if __package__:
    from scripts.joint_terrain_controller import (
        JointTerrainController,
        medium_terrain_speed_profile,
        sample_preview_ground_heights,
    )
else:
    from joint_terrain_controller import (
        JointTerrainController,
        medium_terrain_speed_profile,
        sample_preview_ground_heights,
    )

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "upkie"
    / "terrain"
    / "scene_medium.xml"
)
FPS = 24
DURATION_S = 14.0


def _id(model: mujoco.MjModel, object_type: int, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return object_id


def _phase(time_s: float) -> str:
    if time_s < 1.0:
        return "BALANCE"
    if time_s < 3.0:
        return "ACCELERATE"
    if time_s < 10.0:
        return "ROUGH-TERRAIN CRUISE"
    if time_s < 12.0:
        return "BRAKE"
    return "SETTLE"


def _annotate(
    pixels: np.ndarray,
    *,
    time_s: float,
    speed_m_s: float,
    com_error_mm: float,
    stage_targets_mm: np.ndarray,
) -> np.ndarray:
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image, "RGBA")
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    large = ImageFont.truetype(font_path, 24)
    small = ImageFont.truetype(font_path, 17)
    draw.rounded_rectangle((16, 14, 365, 105), radius=8, fill=(0, 0, 0, 160))
    draw.text((30, 22), _phase(time_s), font=large, fill=(255, 255, 255, 255))
    draw.text(
        (30, 55),
        f"v={speed_m_s:+.2f} m/s   COM dz={com_error_mm:+.1f} mm",
        font=small,
        fill=(170, 225, 255, 255),
    )
    draw.text(
        (30, 79),
        f"strut L/R={stage_targets_mm[0]:+.0f}/{stage_targets_mm[1]:+.0f} mm",
        font=small,
        fill=(255, 215, 145, 255),
    )
    return np.asarray(image)


def render(output_directory: Path) -> tuple[Path, Path]:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to encode the demonstration")
    output_directory.mkdir(parents=True, exist_ok=True)
    video_path = output_directory / "joint_control_medium_terrain_v1.mp4"
    chart_path = output_directory / "joint_control_medium_metrics_v1.png"

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[:7] = (-8.5, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    wheel_actuators = [
        _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ("left_wheel", "right_wheel")
    ]
    strut_actuators = [
        _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ("left_strut", "right_strut")
    ]
    wheel_bodies = tuple(
        _id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in ("left_wheel_node", "right_wheel_node")
    )
    trunk_body = _id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
    initial_com_height = float(data.subtree_com[trunk_body, 2])
    controller = JointTerrainController(float(model.opt.timestep))
    ground_heights = sample_preview_ground_heights(
        model, data, wheel_bodies, 0.0
    )

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 1.55
    camera.azimuth = 90
    camera.elevation = -12

    times: list[float] = []
    targets: list[float] = []
    speeds: list[float] = []
    com_errors: list[float] = []
    stages: list[np.ndarray] = []

    with tempfile.TemporaryDirectory(prefix="joint_terrain_") as temporary:
        frame_directory = Path(temporary)
        frame_index = 0
        next_frame_time = 0.0
        with mujoco.Renderer(model, height=540, width=800) as renderer:
            for step in range(round(DURATION_S / model.opt.timestep)):
                if step % 5 == 0:
                    ground_heights = sample_preview_ground_heights(
                        model,
                        data,
                        wheel_bodies,
                        float(data.qvel[0]),
                    )
                time_s = float(data.time)
                target_speed = medium_terrain_speed_profile(time_s)
                output = controller.update(
                    target_speed_m_s=target_speed,
                    forward_speed_m_s=float(data.qvel[0]),
                    trunk_height_m=float(data.qpos[2]),
                    vertical_speed_m_s=float(data.qvel[2]),
                    quaternion_wxyz=data.qpos[3:7],
                    angular_velocity_xyz=data.qvel[3:6],
                    preview_ground_heights_m=ground_heights,
                )
                data.ctrl[wheel_actuators] = output.wheel_torques_nm
                data.ctrl[strut_actuators] = output.strut_controls_m
                mujoco.mj_step(model, data)
                com_error_mm = (
                    float(data.subtree_com[trunk_body, 2]) - initial_com_height
                ) * 1000.0

                times.append(time_s)
                targets.append(target_speed)
                speeds.append(float(data.qvel[0]))
                com_errors.append(com_error_mm)
                stages.append(output.stage_targets_m.copy() * 1000.0)

                if time_s + 1e-9 >= next_frame_time:
                    camera.lookat[:] = (
                        float(data.qpos[0]),
                        float(data.qpos[1]),
                        0.24,
                    )
                    renderer.update_scene(data, camera=camera)
                    pixels = _annotate(
                        renderer.render(),
                        time_s=time_s,
                        speed_m_s=float(data.qvel[0]),
                        com_error_mm=com_error_mm,
                        stage_targets_mm=output.stage_targets_m * 1000.0,
                    )
                    Image.fromarray(pixels).save(
                        frame_directory / f"frame_{frame_index:04d}.ppm"
                    )
                    frame_index += 1
                    next_frame_time = frame_index / FPS

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
                str(video_path),
            ],
            check=True,
        )

    stage_array = np.asarray(stages)
    figure, axes = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    axes[0].plot(times, targets, "--", color="#52606d", label="target")
    axes[0].plot(times, speeds, color="#0077b6", label="measured")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].legend(loc="upper right")
    axes[1].plot(times, com_errors, color="#e76f51")
    axes[1].axhline(0.0, linestyle="--", color="#52606d")
    axes[1].set_ylabel("COM height error (mm)")
    axes[2].plot(times, stage_array[:, 0], label="left", color="#2a9d8f")
    axes[2].plot(times, stage_array[:, 1], label="right", color="#e9c46a")
    axes[2].set_ylabel("Stage extension (mm)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="upper right")
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Joint control on medium rough terrain")
    figure.tight_layout()
    figure.savefig(chart_path, dpi=160)
    plt.close(figure)

    print(video_path)
    print(chart_path)
    return video_path, chart_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=Path("artifacts"),
    )
    arguments = parser.parse_args()
    render(arguments.output_directory)

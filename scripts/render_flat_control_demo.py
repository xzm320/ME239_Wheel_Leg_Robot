#!/usr/bin/env python3
"""Render flat-ground balance/speed control and its tracking curves."""

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
    from scripts.balance_controller import (
        BalanceSpeedController,
        flat_speed_profile,
        quaternion_pitch,
    )
else:
    from balance_controller import (
        BalanceSpeedController,
        flat_speed_profile,
        quaternion_pitch,
    )

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "upkie"
    / "four_bar"
    / "scene.xml"
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
    if time_s < 8.0:
        return "2.0 m/s CRUISE"
    if time_s < 10.0:
        return "BRAKE"
    return "SETTLE"


def _annotate(
    pixels: np.ndarray,
    *,
    time_s: float,
    speed_m_s: float,
    pitch_rad: float,
) -> np.ndarray:
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image, "RGBA")
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    large = ImageFont.truetype(font_path, 25)
    small = ImageFont.truetype(font_path, 18)
    draw.rounded_rectangle((18, 16, 285, 91), radius=8, fill=(0, 0, 0, 155))
    draw.text((32, 24), _phase(time_s), font=large, fill=(255, 255, 255, 255))
    draw.text(
        (32, 57),
        f"v={speed_m_s:+.2f} m/s   pitch={math.degrees(pitch_rad):+.1f} deg",
        font=small,
        fill=(170, 225, 255, 255),
    )
    return np.asarray(image)


def render(output_directory: Path) -> tuple[Path, Path]:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to encode the demonstration")

    output_directory.mkdir(parents=True, exist_ok=True)
    video_path = output_directory / "flat_balance_speed_pid_v1.mp4"
    chart_path = output_directory / "flat_balance_speed_tracking_v1.png"

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[:7] = (0.0, 0.0, 0.343, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    wheel_ids = [
        _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ("left_wheel", "right_wheel")
    ]
    controller = BalanceSpeedController(float(model.opt.timestep))

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 1.25
    camera.azimuth = 90
    camera.elevation = -10

    times: list[float] = []
    targets: list[float] = []
    speeds: list[float] = []
    pitches: list[float] = []
    torques: list[float] = []

    with tempfile.TemporaryDirectory(prefix="flat_control_") as temporary:
        frame_directory = Path(temporary)
        frame_index = 0
        next_frame_time = 0.0
        with mujoco.Renderer(model, height=480, width=640) as renderer:
            for _ in range(round(DURATION_S / model.opt.timestep)):
                time_s = float(data.time)
                target = flat_speed_profile(time_s)
                pitch = quaternion_pitch(data.qpos[3:7])
                output = controller.update(
                    target_speed_m_s=target,
                    forward_speed_m_s=float(data.qvel[0]),
                    pitch_rad=pitch,
                    pitch_rate_rad_s=float(data.qvel[4]),
                )
                data.ctrl[wheel_ids] = output.wheel_torque_nm
                mujoco.mj_step(model, data)

                times.append(time_s)
                targets.append(target)
                speeds.append(float(data.qvel[0]))
                pitches.append(math.degrees(pitch))
                torques.append(output.wheel_torque_nm)

                if time_s + 1e-9 >= next_frame_time:
                    camera.lookat[:] = (float(data.qpos[0]), 0.0, 0.25)
                    renderer.update_scene(data, camera=camera)
                    pixels = _annotate(
                        renderer.render(),
                        time_s=time_s,
                        speed_m_s=float(data.qvel[0]),
                        pitch_rad=pitch,
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

    figure, axes = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    axes[0].plot(times, targets, "--", color="#52606d", label="target")
    axes[0].plot(times, speeds, color="#0077b6", label="measured")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].legend(loc="upper right")
    axes[1].plot(times, pitches, color="#e76f51")
    axes[1].axhline(8.0, linestyle=":", color="#9b2226")
    axes[1].axhline(-8.0, linestyle=":", color="#9b2226")
    axes[1].set_ylabel("Pitch (deg)")
    axes[2].plot(times, torques, color="#2a9d8f")
    axes[2].set_ylabel("Wheel torque (N m)")
    axes[2].set_xlabel("Time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Flat-ground cascaded PI-PD control")
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

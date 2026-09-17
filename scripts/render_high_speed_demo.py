#!/usr/bin/env python3
"""Render 100 km/h PID from the close side-follow camera."""

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
from PIL import Image, ImageDraw, ImageFont

if __package__:
    from scripts.balance_controller import BalanceSpeedController, quaternion_pitch
    from scripts.high_speed_evaluation import (
        HUNDRED_KMH_WHEEL_TRACK_SCALE,
        MODEL_PATH,
        START_X_M,
        TARGET_100_KMH_M_S,
        apply_hundred_kmh_suspension,
        apply_wheel_track_scale,
        hundred_kmh_balance_gains,
        hundred_kmh_heading_torque_nm,
    )
    from scripts.joint_terrain_controller import (
        HUNDRED_KMH_COMPLIANCE,
        apply_compliance_parameters,
        quaternion_roll_yaw,
    )
else:
    from balance_controller import BalanceSpeedController, quaternion_pitch
    from high_speed_evaluation import (
        HUNDRED_KMH_WHEEL_TRACK_SCALE,
        MODEL_PATH,
        START_X_M,
        TARGET_100_KMH_M_S,
        apply_hundred_kmh_suspension,
        apply_wheel_track_scale,
        hundred_kmh_balance_gains,
        hundred_kmh_heading_torque_nm,
    )
    from joint_terrain_controller import (
        HUNDRED_KMH_COMPLIANCE,
        apply_compliance_parameters,
        quaternion_roll_yaw,
    )

ACCEL_M_S2 = 0.70
FPS = 24
RECORD_START_S = 55.5
DURATION_S = 64.0


def _ids(model: mujoco.MjModel, object_type: int, names: tuple[str, ...]) -> list[int]:
    return [mujoco.mj_name2id(model, object_type, name) for name in names]


def _annotate(
    pixels: np.ndarray,
    *,
    speed: float,
    target: float,
    distance: float,
    pitch_deg: float,
    roll_deg: float,
) -> np.ndarray:
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image, "RGBA")
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    large = ImageFont.truetype(font_path, 22)
    small = ImageFont.truetype(font_path, 17)
    draw.rounded_rectangle((14, 12, 470, 92), radius=8, fill=(0, 0, 0, 170))
    draw.text(
        (26, 18),
        "PID  100 km/h  WIDE DECK  WHOOPS",
        font=large,
        fill=(255, 255, 255, 255),
    )
    draw.text(
        (26, 52),
        (
            f"v={speed:.1f}/{target:.1f} m/s  "
            f"{speed * 3.6:.0f} km/h  "
            f"d={distance:.0f} m  "
            f"pitch={pitch_deg:+.1f}°  roll={roll_deg:+.1f}°"
        ),
        font=small,
        fill=(170, 225, 255, 255),
    )
    return np.asarray(image)


def render(output_directory: Path) -> Path:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to encode the demonstration")
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / "high_speed_rough_track_100kmh_v1.mp4"

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    apply_compliance_parameters(model, HUNDRED_KMH_COMPLIANCE)
    apply_wheel_track_scale(model, HUNDRED_KMH_WHEEL_TRACK_SCALE)
    apply_hundred_kmh_suspension(model)
    # Large heightfields inflate model extent and clip a 3 m follow camera.
    model.stat.extent = 3.0
    model.vis.map.znear = 0.01
    model.vis.map.zfar = 80.0
    data = mujoco.MjData(model)
    data.qpos[:7] = (START_X_M, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    wheel_actuators = _ids(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, ("left_wheel", "right_wheel")
    )
    controller = BalanceSpeedController(
        float(model.opt.timestep),
        hundred_kmh_balance_gains(),
    )

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 3.2
    camera.azimuth = 108
    camera.elevation = -14
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[3] = 1

    with tempfile.TemporaryDirectory(prefix="high_speed_demo_") as temporary:
        frame_directory = Path(temporary)
        frame_index = 0
        next_frame_time = RECORD_START_S
        still_path = output_directory / "high_speed_rough_track_100kmh_v1.png"
        with mujoco.Renderer(model, height=540, width=800) as renderer:
            for _ in range(round(DURATION_S / model.opt.timestep)):
                time_s = float(data.time)
                target = (
                    0.0
                    if time_s < 1.0
                    else min(TARGET_100_KMH_M_S, ACCEL_M_S2 * (time_s - 1.0))
                )
                pitch = quaternion_pitch(data.qpos[3:7])
                roll, yaw = quaternion_roll_yaw(data.qpos[3:7])
                output = controller.update(
                    target_speed_m_s=target,
                    forward_speed_m_s=float(data.qvel[0]),
                    pitch_rad=pitch,
                    pitch_rate_rad_s=float(data.qvel[4]),
                )
                heading_torque = hundred_kmh_heading_torque_nm(
                    lateral_m=float(data.qpos[1]),
                    lateral_speed_m_s=float(data.qvel[1]),
                    yaw_rad=yaw,
                    yaw_rate_rad_s=float(data.qvel[5]),
                    roll_rate_rad_s=float(data.qvel[3]),
                    forward_speed_m_s=float(data.qvel[0]),
                )
                data.ctrl[wheel_actuators] = (
                    output.wheel_torque_nm + heading_torque,
                    output.wheel_torque_nm - heading_torque,
                )
                mujoco.mj_step(model, data)
                roll, _ = quaternion_roll_yaw(data.qpos[3:7])
                pitch = quaternion_pitch(data.qpos[3:7])

                if data.qpos[2] < 0.14 or abs(roll) > 0.40:
                    break
                if time_s + 1e-9 < next_frame_time:
                    continue

                camera.lookat[:] = (
                    float(data.qpos[0]) + 0.70,
                    float(data.qpos[1]),
                    0.16,
                )
                renderer.update_scene(
                    data, camera=camera, scene_option=scene_option
                )
                pixels = _annotate(
                    renderer.render(),
                    speed=float(data.qvel[0]),
                    target=target,
                    distance=float(data.qpos[0] - START_X_M),
                    pitch_deg=float(np.degrees(pitch)),
                    roll_deg=float(np.degrees(roll)),
                )
                Image.fromarray(pixels).save(
                    frame_directory / f"frame_{frame_index:04d}.ppm"
                )
                if frame_index == 48:
                    Image.fromarray(pixels).save(still_path)
                frame_index += 1
                next_frame_time = RECORD_START_S + frame_index / FPS

        if frame_index < 8:
            raise RuntimeError("high-speed render produced too few frames")
        if not still_path.exists():
            last_frame = frame_directory / f"frame_{frame_index - 1:04d}.ppm"
            Image.open(last_frame).save(still_path)
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
    print(output_path)
    print(still_path)
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

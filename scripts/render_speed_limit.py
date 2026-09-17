#!/usr/bin/env python3
"""Render rolling-start clips at each robot's tuned pad speed limit."""

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
    from scripts.balance_controller import (
        BalanceGains,
        BalanceSpeedController,
        HeadingGains,
        heading_torque_nm,
        prototype_balance_gains,
        prototype_heading_gains,
        quaternion_pitch,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from scripts.high_speed_evaluation import (
        apply_hundred_kmh_suspension,
        apply_wheel_track_scale,
    )
    from scripts.joint_terrain_controller import (
        HUNDRED_KMH_COMPLIANCE,
        apply_compliance_parameters,
        quaternion_roll_yaw,
    )
    from scripts.speed_limit import (
        PROTOTYPE_LIMIT_KMH,
        PROTOTYPE_MODEL_PATH,
        WIDE_LIMIT_KMH,
        WIDE_MODEL_PATH,
        WHEEL_RADIUS_M,
        _make_floor_infinite,
    )
else:
    from balance_controller import (
        BalanceGains,
        BalanceSpeedController,
        HeadingGains,
        heading_torque_nm,
        prototype_balance_gains,
        prototype_heading_gains,
        quaternion_pitch,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from high_speed_evaluation import (
        apply_hundred_kmh_suspension,
        apply_wheel_track_scale,
    )
    from joint_terrain_controller import (
        HUNDRED_KMH_COMPLIANCE,
        apply_compliance_parameters,
        quaternion_roll_yaw,
    )
    from speed_limit import (
        PROTOTYPE_LIMIT_KMH,
        PROTOTYPE_MODEL_PATH,
        WIDE_LIMIT_KMH,
        WIDE_MODEL_PATH,
        WHEEL_RADIUS_M,
        _make_floor_infinite,
    )

FPS = 24
DURATION_S = 6.0
WIDTH = 800
HEIGHT = 540


def _ids(model: mujoco.MjModel, names: tuple[str, ...]) -> list[int]:
    return [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in names]


def _annotate(
    pixels: np.ndarray,
    *,
    title: str,
    speed: float,
    target: float,
    pitch_deg: float,
    roll_deg: float,
) -> np.ndarray:
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image, "RGBA")
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    large = ImageFont.truetype(font_path, 22)
    small = ImageFont.truetype(font_path, 17)
    draw.rounded_rectangle((14, 12, 520, 92), radius=8, fill=(0, 0, 0, 170))
    draw.text((26, 18), title, font=large, fill=(255, 255, 255, 255))
    draw.text(
        (26, 52),
        (
            f"v={speed:.1f}/{target:.1f} m/s  "
            f"{speed * 3.6:.0f} km/h  "
            f"pitch={pitch_deg:+.1f}°  roll={roll_deg:+.1f}°"
        ),
        font=small,
        fill=(170, 225, 255, 255),
    )
    return np.asarray(image)


def _render_one(
    *,
    output_directory: Path,
    stem: str,
    title: str,
    model_path: Path,
    kmh: float,
    balance: BalanceGains,
    heading: HeadingGains,
    wheel_track_scale: float | None,
    soften_struts: bool,
    camera_distance: float,
) -> tuple[Path, Path]:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to encode the demonstration")
    output_directory.mkdir(parents=True, exist_ok=True)
    video_path = output_directory / f"{stem}.mp4"
    still_path = output_directory / f"{stem}.png"
    target = kmh / 3.6

    model = mujoco.MjModel.from_xml_path(str(model_path))
    _make_floor_infinite(model)
    for material_id in range(model.nmat):
        if model.mat_texuniform[material_id]:
            model.mat_texrepeat[material_id] = (1.0, 1.0)
    apply_compliance_parameters(model, HUNDRED_KMH_COMPLIANCE)
    if wheel_track_scale is not None:
        apply_wheel_track_scale(model, wheel_track_scale)
    if soften_struts:
        apply_hundred_kmh_suspension(model)
    model.stat.extent = 4.0
    model.vis.map.znear = 0.01
    model.vis.map.zfar = 30.0
    data = mujoco.MjData(model)
    launch_x = -2000.0 if wheel_track_scale is not None else 0.0
    data.qpos[:7] = (launch_x, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    data.qvel[0] = target
    for name in ("left_wheel_joint", "right_wheel_joint"):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qvel[int(model.jnt_dofadr[joint_id])] = target / WHEEL_RADIUS_M
    mujoco.mj_forward(model, data)
    wheel_actuators = _ids(model, ("left_wheel", "right_wheel"))
    controller = BalanceSpeedController(float(model.opt.timestep), balance)

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = camera_distance
    camera.azimuth = 128
    camera.elevation = -28
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[3] = 1

    with tempfile.TemporaryDirectory(prefix="speed_limit_") as temporary:
        frame_directory = Path(temporary)
        frame_index = 0
        next_frame_time = 0.0
        with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
            for _ in range(round(DURATION_S / model.opt.timestep)):
                pitch = quaternion_pitch(data.qpos[3:7])
                roll, yaw = quaternion_roll_yaw(data.qpos[3:7])
                output = controller.update(
                    target_speed_m_s=target,
                    forward_speed_m_s=float(data.qvel[0]),
                    pitch_rad=pitch,
                    pitch_rate_rad_s=float(data.qvel[4]),
                )
                heading_torque = heading_torque_nm(
                    heading,
                    lateral_m=float(data.qpos[1]),
                    lateral_speed_m_s=float(data.qvel[1]),
                    yaw_rad=yaw,
                    yaw_rate_rad_s=float(data.qvel[5]),
                    roll_rad=roll,
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
                if data.qpos[2] < 0.14 or abs(roll) > 0.40 or abs(pitch) > 0.45:
                    break
                time_s = float(data.time)
                if time_s + 1e-9 < next_frame_time:
                    continue
                camera.lookat[:] = (
                    float(data.qpos[0]) + 0.40,
                    float(data.qpos[1]),
                    0.08,
                )
                renderer.update_scene(
                    data, camera=camera, scene_option=scene_option
                )
                pixels = _annotate(
                    renderer.render(),
                    title=title,
                    speed=float(data.qvel[0]),
                    target=target,
                    pitch_deg=float(np.degrees(pitch)),
                    roll_deg=float(np.degrees(roll)),
                )
                Image.fromarray(pixels).save(
                    frame_directory / f"frame_{frame_index:04d}.ppm"
                )
                if frame_index == 20:
                    Image.fromarray(pixels).save(still_path)
                frame_index += 1
                next_frame_time = frame_index / FPS

        if frame_index < 8:
            raise RuntimeError(f"{stem} produced too few frames")
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
                str(video_path),
            ],
            check=True,
        )
    print(video_path)
    print(still_path)
    return video_path, still_path


def render(output_directory: Path) -> dict[str, Path]:
    wide = _render_one(
        output_directory=output_directory,
        stem="wide_car_195kmh",
        title="3x TRACK  PID  195 km/h  LIMIT",
        model_path=WIDE_MODEL_PATH,
        kmh=WIDE_LIMIT_KMH,
        balance=wide_car_balance_gains(),
        heading=wide_car_heading_gains(),
        wheel_track_scale=3.0,
        soften_struts=True,
        camera_distance=4.4,
    )
    proto = _render_one(
        output_directory=output_directory,
        stem="prototype_180kmh",
        title="PROTOTYPE  PID  180 km/h  LIMIT",
        model_path=PROTOTYPE_MODEL_PATH,
        kmh=PROTOTYPE_LIMIT_KMH,
        balance=prototype_balance_gains(),
        heading=prototype_heading_gains(),
        wheel_track_scale=None,
        soften_struts=False,
        camera_distance=3.6,
    )
    return {
        "wide_video": wide[0],
        "wide_still": wide[1],
        "prototype_video": proto[0],
        "prototype_still": proto[1],
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-directory", type=Path, default=Path("artifacts"))
    arguments = parser.parse_args()
    render(arguments.output_directory)

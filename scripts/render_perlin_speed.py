#!/usr/bin/env python3
"""Follow-cam clips of Perlin-strip speed limits."""

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
        BalanceSpeedController,
        heading_torque_nm,
        perlin_wide_balance_gains,
        perlin_wide_heading_gains,
        quaternion_pitch,
        upkie_balance_gains,
        upkie_heading_gains,
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
    from scripts.perlin_speed import (
        UPKIE_SCENE,
        WIDE_PERLIN_LIMIT_M_S,
        WIDE_SCENE,
        WIDE_WHEEL_RADIUS_M,
        _ids,
        terrain_height_m,
    )
else:
    from balance_controller import (
        BalanceSpeedController,
        heading_torque_nm,
        perlin_wide_balance_gains,
        perlin_wide_heading_gains,
        quaternion_pitch,
        upkie_balance_gains,
        upkie_heading_gains,
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
    from perlin_speed import (
        UPKIE_SCENE,
        WIDE_PERLIN_LIMIT_M_S,
        WIDE_SCENE,
        WIDE_WHEEL_RADIUS_M,
        _ids,
        terrain_height_m,
    )

FPS = 20
WIDTH = 800
HEIGHT = 450


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
    large = ImageFont.truetype(font_path, 20)
    small = ImageFont.truetype(font_path, 16)
    draw.rounded_rectangle((12, 10, 560, 86), radius=8, fill=(0, 0, 0, 170))
    draw.text((22, 16), title, font=large, fill=(255, 255, 255, 255))
    draw.text(
        (22, 48),
        (
            f"v={speed:.2f}/{target:.2f} m/s  "
            f"{speed * 3.6:.1f} km/h  "
            f"pitch={pitch_deg:+.1f}°  roll={roll_deg:+.1f}°"
        ),
        font=small,
        fill=(170, 225, 255, 255),
    )
    return np.asarray(image)


def _encode(frame_directory: Path, video_path: Path) -> None:
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


def _setup_camera(model: mujoco.MjModel, distance: float) -> tuple[mujoco.MjvCamera, mujoco.MjvOption]:
    model.stat.extent = 4.0
    model.vis.map.znear = 0.01
    model.vis.map.zfar = 40.0
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = distance
    camera.azimuth = 125
    camera.elevation = -24
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[3] = 1
    return camera, scene_option


def render_wide(output_directory: Path) -> tuple[Path, Path]:
    output_directory.mkdir(parents=True, exist_ok=True)
    video_path = output_directory / "wide_perlin_3ms.mp4"
    still_path = output_directory / "wide_perlin_3ms.png"
    target = WIDE_PERLIN_LIMIT_M_S
    start_x = 12.0
    model = mujoco.MjModel.from_xml_path(str(WIDE_SCENE))
    apply_compliance_parameters(model, HUNDRED_KMH_COMPLIANCE)
    apply_wheel_track_scale(model, 3.0)
    apply_hundred_kmh_suspension(model)
    data = mujoco.MjData(model)
    data.qpos[:7] = (
        start_x,
        0.0,
        0.408 + terrain_height_m(start_x, 0.0),
        1.0,
        0.0,
        0.0,
        0.0,
    )
    data.qvel[0] = target
    for name in ("left_wheel_joint", "right_wheel_joint"):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qvel[int(model.jnt_dofadr[joint_id])] = target / WIDE_WHEEL_RADIUS_M
    mujoco.mj_forward(model, data)
    wheels = _ids(model, ("left_wheel", "right_wheel"))
    controller = BalanceSpeedController(
        float(model.opt.timestep), perlin_wide_balance_gains()
    )
    heading = perlin_wide_heading_gains()
    camera, scene_option = _setup_camera(model, 4.2)
    duration_s = 6.0
    with tempfile.TemporaryDirectory(prefix="perlin_wide_") as temporary:
        frame_directory = Path(temporary)
        frame_index = 0
        next_frame_time = 0.0
        with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
            for _ in range(round(duration_s / model.opt.timestep)):
                pitch = quaternion_pitch(data.qpos[3:7])
                roll, yaw = quaternion_roll_yaw(data.qpos[3:7])
                output = controller.update(
                    target_speed_m_s=target,
                    forward_speed_m_s=float(data.qvel[0]),
                    pitch_rad=pitch,
                    pitch_rate_rad_s=float(data.qvel[4]),
                )
                head = heading_torque_nm(
                    heading,
                    lateral_m=float(data.qpos[1]),
                    lateral_speed_m_s=float(data.qvel[1]),
                    yaw_rad=yaw,
                    yaw_rate_rad_s=float(data.qvel[5]),
                    roll_rad=roll,
                    roll_rate_rad_s=float(data.qvel[3]),
                    forward_speed_m_s=float(data.qvel[0]),
                )
                data.ctrl[wheels[0]] = output.wheel_torque_nm + head
                data.ctrl[wheels[1]] = output.wheel_torque_nm - head
                mujoco.mj_step(model, data)
                if float(data.time) + 1e-9 < next_frame_time:
                    continue
                roll, _ = quaternion_roll_yaw(data.qpos[3:7])
                pitch = quaternion_pitch(data.qpos[3:7])
                camera.lookat[:] = (
                    float(data.qpos[0]) + 0.35,
                    float(data.qpos[1]),
                    0.06 + terrain_height_m(float(data.qpos[0]), 0.0),
                )
                renderer.update_scene(data, camera=camera, scene_option=scene_option)
                pixels = _annotate(
                    renderer.render(),
                    title="3x FOUR-BAR  PERLIN  3.0 m/s  LIMIT",
                    speed=float(data.qvel[0]),
                    target=target,
                    pitch_deg=float(np.degrees(pitch)),
                    roll_deg=float(np.degrees(roll)),
                )
                Image.fromarray(pixels).save(
                    frame_directory / f"frame_{frame_index:04d}.ppm"
                )
                if frame_index == 18:
                    Image.fromarray(pixels).save(still_path)
                frame_index += 1
                next_frame_time = frame_index / FPS
        if not still_path.exists():
            Image.open(frame_directory / f"frame_{frame_index - 1:04d}.ppm").save(
                still_path
            )
        _encode(frame_directory, video_path)
    print(video_path)
    print(still_path)
    return video_path, still_path


def render_upkie(output_directory: Path) -> tuple[Path, Path]:
    output_directory.mkdir(parents=True, exist_ok=True)
    video_path = output_directory / "upkie_perlin_limit.mp4"
    still_path = output_directory / "upkie_perlin_limit.png"
    start_x = 12.0
    crawl = 0.35
    model = mujoco.MjModel.from_xml_path(str(UPKIE_SCENE))
    data = mujoco.MjData(model)
    data.qpos[:7] = (
        start_x,
        0.0,
        0.343 + terrain_height_m(start_x, 0.0),
        1.0,
        0.0,
        0.0,
        0.0,
    )
    mujoco.mj_forward(model, data)
    hips = _ids(model, ("left_hip", "left_knee", "right_hip", "right_knee"))
    wheels = _ids(model, ("left_wheel", "right_wheel"))
    left_dof = int(
        model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel")
        ]
    )
    right_dof = int(
        model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel")
        ]
    )
    wheel_kv = float(model.actuator_gainprm[wheels[0], 0])
    controller = BalanceSpeedController(float(model.opt.timestep), upkie_balance_gains())
    heading = upkie_heading_gains()
    camera, scene_option = _setup_camera(model, 2.4)
    duration_s = 5.5
    with tempfile.TemporaryDirectory(prefix="perlin_upkie_") as temporary:
        frame_directory = Path(temporary)
        frame_index = 0
        next_frame_time = 0.0
        with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
            for _ in range(round(duration_s / model.opt.timestep)):
                time_s = float(data.time)
                commanded = 0.0 if time_s < 1.6 else min(crawl, 0.25 * (time_s - 1.6))
                pitch = quaternion_pitch(data.qpos[3:7])
                roll, yaw = quaternion_roll_yaw(data.qpos[3:7])
                output = controller.update(
                    target_speed_m_s=commanded,
                    forward_speed_m_s=float(data.qvel[0]),
                    pitch_rad=pitch,
                    pitch_rate_rad_s=float(data.qvel[4]),
                )
                head = heading_torque_nm(
                    heading,
                    lateral_m=float(data.qpos[1]),
                    lateral_speed_m_s=float(data.qvel[1]),
                    yaw_rad=yaw,
                    yaw_rate_rad_s=float(data.qvel[5]),
                    roll_rad=roll,
                    roll_rate_rad_s=float(data.qvel[3]),
                    forward_speed_m_s=float(data.qvel[0]),
                )
                for actuator_id in hips:
                    data.ctrl[actuator_id] = 0.0
                torque = output.wheel_torque_nm
                data.ctrl[wheels[0]] = data.qvel[left_dof] + (torque + head) / wheel_kv
                data.ctrl[wheels[1]] = data.qvel[right_dof] - (torque - head) / wheel_kv
                mujoco.mj_step(model, data)
                if time_s + 1e-9 < next_frame_time:
                    continue
                roll, _ = quaternion_roll_yaw(data.qpos[3:7])
                pitch = quaternion_pitch(data.qpos[3:7])
                camera.lookat[:] = (
                    float(data.qpos[0]) + 0.15,
                    float(data.qpos[1]),
                    0.04 + terrain_height_m(float(data.qpos[0]), 0.0),
                )
                renderer.update_scene(data, camera=camera, scene_option=scene_option)
                pixels = _annotate(
                    renderer.render(),
                    title="STOCK UPKIE  PERLIN  STAND THEN CRAWL",
                    speed=float(data.qvel[0]),
                    target=commanded,
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
                if data.qpos[2] < 0.16 or abs(pitch) > np.radians(50.0):
                    break
        if frame_index < 8:
            raise RuntimeError("upkie perlin clip produced too few frames")
        if not still_path.exists():
            Image.open(frame_directory / f"frame_{min(20, frame_index - 1):04d}.ppm").save(
                still_path
            )
        _encode(frame_directory, video_path)
    print(video_path)
    print(still_path)
    return video_path, still_path


def render(output_directory: Path) -> dict[str, Path]:
    wide = render_wide(output_directory)
    upkie = render_upkie(output_directory)
    return {
        "wide_video": wide[0],
        "wide_still": wide[1],
        "upkie_video": upkie[0],
        "upkie_still": upkie[1],
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-directory", type=Path, default=Path("artifacts"))
    arguments = parser.parse_args()
    render(arguments.output_directory)

#!/usr/bin/env python3
"""Follow-cam clips of PID episodes on the Unitree Perlin strip."""

from __future__ import annotations

import argparse
import os
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

if __package__:
    from scripts.balance_pid import (
        BalanceSpeedController,
        heading_torque_nm,
        prototype_balance_gains,
        prototype_heading_gains,
        quaternion_pitch,
        quaternion_roll_yaw,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from scripts.generate_perlin import ROOT, terrain_height_m
    from scripts.prototype_perlin import (
        HIP_KNEE_ACTUATORS,
        PROTOTYPE_SCENE,
        STAND_HEIGHT_M,
        WHEEL_ACTUATORS,
        WHEEL_JOINTS,
        WHEEL_KV,
        apply_prototype_pose_hold,
        actuator_ids,
        joint_dof,
    )
    from scripts.wide_car_perlin import apply_passive_struts
else:
    from balance_pid import (
        BalanceSpeedController,
        heading_torque_nm,
        prototype_balance_gains,
        prototype_heading_gains,
        quaternion_pitch,
        quaternion_roll_yaw,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from generate_perlin import ROOT, terrain_height_m
    from prototype_perlin import (
        HIP_KNEE_ACTUATORS,
        PROTOTYPE_SCENE,
        STAND_HEIGHT_M,
        WHEEL_ACTUATORS,
        WHEEL_JOINTS,
        WHEEL_KV,
        apply_prototype_pose_hold,
        actuator_ids,
        joint_dof,
    )
    from wide_car_perlin import apply_passive_struts

FPS = 20
WIDTH = 960
HEIGHT = 540
MEDIA = ROOT / "docs" / "media"
WIDE_SCENE = ROOT / "models" / "wide_car" / "scene.xml"
WIDE_STAND_HEIGHT_M = 0.408


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")
    if path.exists():
        return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def annotate(
    pixels: np.ndarray,
    *,
    title: str,
    speed: float,
    target: float,
    pitch_deg: float,
    roll_deg: float,
    x_m: float,
) -> np.ndarray:
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rounded_rectangle((12, 10, 720, 92), radius=8, fill=(0, 0, 0, 175))
    draw.text((22, 16), title, font=_font(20), fill=(255, 255, 255, 255))
    draw.text(
        (22, 50),
        (
            f"v={speed:.2f}/{target:.2f} m/s  {speed * 3.6:.1f} km/h  "
            f"x={x_m:.1f} m  pitch={pitch_deg:+.1f}°  roll={roll_deg:+.1f}°"
        ),
        font=_font(16),
        fill=(170, 225, 255, 255),
    )
    return np.asarray(image)


def encode_ppm(frame_directory: Path, video_path: Path) -> None:
    video_path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            str(FPS),
            "-i",
            str(frame_directory / "frame_%05d.ppm"),
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


def setup_camera(model: mujoco.MjModel, distance: float) -> tuple[mujoco.MjvCamera, mujoco.MjvOption]:
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


def render_prototype_clip(
    *,
    target_speed_m_s: float,
    duration_s: float,
    start_x_m: float,
    title: str,
    stem: str,
    acceleration_m_s2: float = 0.28,
) -> Path:
    model = mujoco.MjModel.from_xml_path(str(PROTOTYPE_SCENE))
    apply_prototype_pose_hold(model)
    data = mujoco.MjData(model)
    data.qpos[0] = start_x_m
    data.qpos[2] = STAND_HEIGHT_M + terrain_height_m(start_x_m, 0.0)
    mujoco.mj_forward(model, data)
    hips = actuator_ids(model, HIP_KNEE_ACTUATORS)
    wheels = actuator_ids(model, WHEEL_ACTUATORS)
    left_dof = joint_dof(model, WHEEL_JOINTS[0])
    right_dof = joint_dof(model, WHEEL_JOINTS[1])
    controller = BalanceSpeedController(
        float(model.opt.timestep), prototype_balance_gains(), initial_pitch_reference=0.02
    )
    heading = prototype_heading_gains()
    camera, scene_option = setup_camera(model, 2.4)
    video_path = MEDIA / f"{stem}.mp4"
    still_path = MEDIA / f"{stem}.png"

    with tempfile.TemporaryDirectory(prefix="proto_pid_") as temporary:
        frame_directory = Path(temporary)
        frame_index = 0
        next_frame_time = 0.0
        last_pixels: np.ndarray | None = None
        with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
            steps = int(round(duration_s / float(model.opt.timestep)))
            for _ in range(steps):
                time_s = float(data.time)
                commanded = (
                    0.0
                    if target_speed_m_s < 0.02
                    else min(
                        target_speed_m_s,
                        acceleration_m_s2 * max(0.0, time_s - 0.8),
                    )
                )
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
                )
                for hip in hips:
                    data.ctrl[hip] = 0.0
                data.ctrl[wheels[0]] = data.qvel[left_dof] + (
                    output.wheel_torque_nm + head
                ) / WHEEL_KV
                data.ctrl[wheels[1]] = data.qvel[right_dof] + (
                    -output.wheel_torque_nm + head
                ) / WHEEL_KV
                mujoco.mj_step(model, data)
                if float(data.time) + 1e-9 < next_frame_time:
                    continue
                pitch = quaternion_pitch(data.qpos[3:7])
                roll, _ = quaternion_roll_yaw(data.qpos[3:7])
                camera.lookat[:] = (
                    float(data.qpos[0]) + 0.25,
                    float(data.qpos[1]),
                    0.08 + terrain_height_m(float(data.qpos[0]), 0.0),
                )
                renderer.update_scene(data, camera=camera, scene_option=scene_option)
                pixels = annotate(
                    renderer.render(),
                    title=title,
                    speed=float(data.qvel[0]),
                    target=target_speed_m_s,
                    pitch_deg=float(np.degrees(pitch)),
                    roll_deg=float(np.degrees(roll)),
                    x_m=float(data.qpos[0]),
                )
                Image.fromarray(pixels).save(
                    frame_directory / f"frame_{frame_index:05d}.ppm"
                )
                last_pixels = pixels
                frame_index += 1
                next_frame_time += 1.0 / FPS
        encode_ppm(frame_directory, video_path)
        if last_pixels is not None:
            Image.fromarray(last_pixels).save(still_path)
    return video_path


def render_wide_car_clip(
    *,
    target_speed_m_s: float,
    duration_s: float,
    start_x_m: float,
    title: str,
    stem: str,
    acceleration_m_s2: float = 1.8,
    rolling_start: bool = False,
) -> Path:
    """Wheel PID only; hips and struts stay at the nominal pose."""

    model = mujoco.MjModel.from_xml_path(str(WIDE_SCENE))
    apply_passive_struts(model)
    data = mujoco.MjData(model)
    data.qpos[0] = start_x_m
    data.qpos[2] = WIDE_STAND_HEIGHT_M + terrain_height_m(start_x_m, 0.0)
    if rolling_start:
        data.qvel[0] = target_speed_m_s
        radius = 0.120
        for name in ("left_wheel_joint", "right_wheel_joint"):
            data.qvel[joint_dof(model, name)] = target_speed_m_s / radius
    mujoco.mj_forward(model, data)
    pose = actuator_ids(model, ("left_hip", "left_strut", "right_hip", "right_strut"))
    wheels = actuator_ids(model, ("left_wheel", "right_wheel"))
    controller = BalanceSpeedController(float(model.opt.timestep), wide_car_balance_gains())
    heading = wide_car_heading_gains()
    camera, scene_option = setup_camera(model, 4.2)
    video_path = MEDIA / f"{stem}.mp4"
    still_path = MEDIA / f"{stem}.png"

    with tempfile.TemporaryDirectory(prefix="wide_pid_") as temporary:
        frame_directory = Path(temporary)
        frame_index = 0
        next_frame_time = 0.0
        last_pixels: np.ndarray | None = None
        with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
            steps = int(round(duration_s / float(model.opt.timestep)))
            for _ in range(steps):
                time_s = float(data.time)
                commanded = (
                    target_speed_m_s
                    if rolling_start
                    else min(target_speed_m_s, acceleration_m_s2 * max(0.0, time_s - 0.4))
                )
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
                )
                for index in pose:
                    data.ctrl[index] = 0.0
                data.ctrl[wheels[0]] = output.wheel_torque_nm + head
                data.ctrl[wheels[1]] = output.wheel_torque_nm - head
                mujoco.mj_step(model, data)
                if float(data.time) + 1e-9 < next_frame_time:
                    continue
                pitch = quaternion_pitch(data.qpos[3:7])
                roll, _ = quaternion_roll_yaw(data.qpos[3:7])
                camera.lookat[:] = (
                    float(data.qpos[0]) + 0.35,
                    float(data.qpos[1]),
                    0.08 + terrain_height_m(float(data.qpos[0]), 0.0),
                )
                renderer.update_scene(data, camera=camera, scene_option=scene_option)
                pixels = annotate(
                    renderer.render(),
                    title=title,
                    speed=float(data.qvel[0]),
                    target=target_speed_m_s,
                    pitch_deg=float(np.degrees(pitch)),
                    roll_deg=float(np.degrees(roll)),
                    x_m=float(data.qpos[0]),
                )
                Image.fromarray(pixels).save(
                    frame_directory / f"frame_{frame_index:05d}.ppm"
                )
                last_pixels = pixels
                frame_index += 1
                next_frame_time += 1.0 / FPS
        encode_ppm(frame_directory, video_path)
        if last_pixels is not None:
            Image.fromarray(last_pixels).save(still_path)
    return video_path


def render_prototype_set() -> list[Path]:
    clips = [
        dict(
            target_speed_m_s=0.0,
            duration_s=6.0,
            start_x_m=3.5,
            title="PROTOTYPE  PID  STAND  0 m/s",
            stem="prototype_pid_stand",
        ),
        dict(
            target_speed_m_s=0.15,
            duration_s=18.0,
            start_x_m=8.5,
            title="PROTOTYPE  PID  PERLIN  0.15 m/s  LIMIT",
            stem="prototype_pid_0p15ms",
            acceleration_m_s2=0.12,
        ),
        dict(
            target_speed_m_s=0.40,
            duration_s=8.0,
            start_x_m=8.0,
            title="PROTOTYPE  PID  PERLIN  0.40 m/s  FAIL",
            stem="prototype_pid_0p40ms_fail",
            acceleration_m_s2=0.12,
        ),
    ]
    return [render_prototype_clip(**clip) for clip in clips]


def render_wide_car_set() -> list[Path]:
    clips = [
        dict(
            target_speed_m_s=0.0,
            duration_s=5.0,
            start_x_m=3.5,
            title="WIDE_CAR  PID  PASSIVE  STAND",
            stem="wide_car_pid_passive_stand",
        ),
        dict(
            target_speed_m_s=3.7,
            duration_s=10.0,
            start_x_m=6.0,
            title="WIDE_CAR  PID  PASSIVE  3.7 m/s  LIMIT",
            stem="wide_car_pid_passive_3p7ms",
            acceleration_m_s2=1.8,
        ),
        dict(
            target_speed_m_s=5.0,
            duration_s=8.0,
            start_x_m=6.0,
            title="WIDE_CAR  PID  PASSIVE  5.0 m/s  FAIL",
            stem="wide_car_pid_passive_5p0ms_fail",
            acceleration_m_s2=1.8,
        ),
    ]
    return [render_wide_car_clip(**clip) for clip in clips]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        choices=("prototype", "wide_car"),
        default="prototype",
    )
    args = parser.parse_args()
    paths = render_prototype_set() if args.robot == "prototype" else render_wide_car_set()
    for path in paths:
        print(path)


if __name__ == "__main__":
    main()

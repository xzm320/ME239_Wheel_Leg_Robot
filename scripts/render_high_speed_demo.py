#!/usr/bin/env python3
"""Render the selected 10 m/s robust rough-track validation."""

from __future__ import annotations

import argparse
import math
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
    from scripts.high_speed_evaluation import (
        MODEL_PATH,
        START_X_M,
        WHEEL_RADIUS_M,
        high_speed_balance_gains,
        high_speed_terrain_gains,
    )
    from scripts.joint_terrain_controller import (
        HIGH_SPEED_COMPLIANCE,
        JointTerrainController,
        apply_compliance_parameters,
        sample_preview_ground_heights,
    )
else:
    from high_speed_evaluation import (
        MODEL_PATH,
        START_X_M,
        WHEEL_RADIUS_M,
        high_speed_balance_gains,
        high_speed_terrain_gains,
    )
    from joint_terrain_controller import (
        HIGH_SPEED_COMPLIANCE,
        JointTerrainController,
        apply_compliance_parameters,
        sample_preview_ground_heights,
    )

TARGET_SPEED_M_S = 10.0
FPS = 24
DURATION_S = 12.0


def _ids(
    model: mujoco.MjModel,
    object_type: int,
    names: tuple[str, ...],
) -> list[int]:
    return [
        mujoco.mj_name2id(model, object_type, name)
        for name in names
    ]


def _annotate(
    pixels: np.ndarray,
    speed: float,
    distance: float,
    com_error_mm: float,
) -> np.ndarray:
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image, "RGBA")
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    large = ImageFont.truetype(font_path, 25)
    small = ImageFont.truetype(font_path, 18)
    draw.rounded_rectangle((18, 16, 360, 96), radius=8, fill=(0, 0, 0, 165))
    draw.text(
        (32, 24),
        "ROBUST SPEED: 10.0 m/s (36 km/h)",
        font=large,
        fill=(255, 255, 255, 255),
    )
    draw.text(
        (32, 60),
        f"v={speed:.2f} m/s  distance={distance:.1f} m  COM dz={com_error_mm:+.1f} mm",
        font=small,
        fill=(170, 225, 255, 255),
    )
    return np.asarray(image)


def render(output_directory: Path) -> Path:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to encode the demonstration")
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / "high_speed_rough_track_10mps_v1.mp4"

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    apply_compliance_parameters(model, HIGH_SPEED_COMPLIANCE)
    data = mujoco.MjData(model)
    data.qpos[:7] = (START_X_M, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    wheel_actuators = _ids(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        ("left_wheel", "right_wheel"),
    )
    strut_actuators = _ids(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        ("left_strut", "right_strut"),
    )
    wheel_bodies = tuple(
        _ids(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            ("left_wheel_node", "right_wheel_node"),
        )
    )
    wheel_joints = _ids(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        ("left_wheel_joint", "right_wheel_joint"),
    )
    trunk_body = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "trunk"
    )
    terrain_gains = high_speed_terrain_gains(HIGH_SPEED_COMPLIANCE)
    controller = JointTerrainController(
        float(model.opt.timestep),
        terrain_gains,
        high_speed_balance_gains(),
    )
    ground = sample_preview_ground_heights(
        model, data, wheel_bodies, 0.0, terrain_gains
    )
    for _ in range(round(1.0 / model.opt.timestep)):
        output = controller.update(
            target_speed_m_s=0.0,
            forward_speed_m_s=float(data.qvel[0]),
            trunk_height_m=float(data.qpos[2]),
            vertical_speed_m_s=float(data.qvel[2]),
            quaternion_wxyz=data.qpos[3:7],
            angular_velocity_xyz=data.qvel[3:6],
            preview_ground_heights_m=ground,
        )
        data.ctrl[wheel_actuators] = output.wheel_torques_nm
        data.ctrl[strut_actuators] = output.strut_controls_m
        mujoco.mj_step(model, data)

    data.qvel[:] = 0.0
    data.qvel[0] = TARGET_SPEED_M_S
    for joint_id in wheel_joints:
        data.qvel[int(model.jnt_dofadr[joint_id])] = (
            TARGET_SPEED_M_S / WHEEL_RADIUS_M
        )
    mujoco.mj_forward(model, data)
    controller = JointTerrainController(
        float(model.opt.timestep),
        terrain_gains,
        high_speed_balance_gains(),
    )
    initial_com_height = float(data.subtree_com[trunk_body, 2])

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 5.0
    camera.azimuth = 116
    camera.elevation = -18
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[3] = 1

    with tempfile.TemporaryDirectory(prefix="high_speed_demo_") as temporary:
        frame_directory = Path(temporary)
        frame_index = 0
        next_frame_time = 0.0
        ground = sample_preview_ground_heights(
            model,
            data,
            wheel_bodies,
            TARGET_SPEED_M_S,
            terrain_gains,
        )
        with mujoco.Renderer(model, height=540, width=800) as renderer:
            for step in range(round(DURATION_S / model.opt.timestep)):
                if step % 5 == 0:
                    ground = sample_preview_ground_heights(
                        model,
                        data,
                        wheel_bodies,
                        float(data.qvel[0]),
                        terrain_gains,
                    )
                output = controller.update(
                    target_speed_m_s=TARGET_SPEED_M_S,
                    forward_speed_m_s=float(data.qvel[0]),
                    trunk_height_m=float(data.qpos[2]),
                    vertical_speed_m_s=float(data.qvel[2]),
                    quaternion_wxyz=data.qpos[3:7],
                    angular_velocity_xyz=data.qvel[3:6],
                    preview_ground_heights_m=ground,
                )
                data.ctrl[wheel_actuators] = output.wheel_torques_nm
                data.ctrl[strut_actuators] = output.strut_controls_m
                mujoco.mj_step(model, data)

                if data.time - 1.0 + 1e-9 >= next_frame_time:
                    camera.lookat[:] = (
                        float(data.qpos[0]) + 1.2,
                        float(data.qpos[1]),
                        0.20,
                    )
                    renderer.update_scene(
                        data,
                        camera=camera,
                        scene_option=scene_option,
                    )
                    pixels = _annotate(
                        renderer.render(),
                        float(data.qvel[0]),
                        float(data.qpos[0] - START_X_M),
                        (
                            float(data.subtree_com[trunk_body, 2])
                            - initial_com_height
                        )
                        * 1000.0,
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

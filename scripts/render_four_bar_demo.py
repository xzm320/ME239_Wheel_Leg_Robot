#!/usr/bin/env python3
"""Render a bench demonstration of active extension and passive return."""

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

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "upkie"
    / "four_bar"
    / "scene.xml"
)
FPS = 30
DURATION = 5.2
STRUT_ACTUATORS = ("left_strut", "right_strut")


def _id(model: mujoco.MjModel, object_type: int, name: str) -> int:
    result = mujoco.mj_name2id(model, object_type, name)
    if result < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return result


def _smoothstep(start: float, end: float, value: float) -> float:
    ratio = np.clip((value - start) / (end - start), 0.0, 1.0)
    return float(0.5 - 0.5 * math.cos(math.pi * ratio))


def _desired_extension(time_s: float) -> float:
    if time_s < 0.8:
        return 0.0
    if time_s < 1.8:
        return 0.040 * _smoothstep(0.8, 1.8, time_s)
    if time_s < 2.4:
        return 0.040
    if time_s < 3.4:
        blend = _smoothstep(2.4, 3.4, time_s)
        return 0.040 + (-0.035 - 0.040) * blend
    if time_s < 4.0:
        return -0.035
    return 0.0


def _write_ppm(path: Path, pixels: np.ndarray) -> None:
    height, width, _ = pixels.shape
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels.tobytes())


def render(output_directory: Path) -> tuple[Path, Path]:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to encode the demonstration")

    output_directory.mkdir(parents=True, exist_ok=True)
    video_path = output_directory / "four_bar_active_passive_demo.mp4"
    states_path = output_directory / "four_bar_three_states.png"

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    data.qpos[:7] = (0.0, 0.0, 0.68, 1.0, 0.0, 0.0, 0.0)
    actuator_ids = [
        _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in STRUT_ACTUATORS
    ]

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.0, 0.0, 0.50)
    camera.distance = 1.15
    camera.azimuth = 132
    camera.elevation = -8

    state_times = {
        "neutral": 0.5,
        "shortened": 2.1,
        "extended": 3.7,
    }
    state_frames: dict[str, Path] = {}
    servos_disabled = False

    with tempfile.TemporaryDirectory(prefix="four_bar_demo_") as temporary:
        frame_directory = Path(temporary)
        next_frame_time = 0.0
        frame_index = 0

        with mujoco.Renderer(model, height=720, width=960) as renderer:
            while data.time < DURATION:
                desired = _desired_extension(float(data.time))
                if data.time >= 4.0 and not servos_disabled:
                    for actuator_id in actuator_ids:
                        model.actuator_gainprm[actuator_id, :] = 0.0
                        model.actuator_biasprm[actuator_id, :] = 0.0
                    servos_disabled = True

                if not servos_disabled:
                    # Compensate the 1500 N/m passive spring at static equilibrium.
                    data.ctrl[actuator_ids] = desired * 1.6

                # A virtual fixture holds the floating trunk for this mechanism-only test.
                position_error = data.qpos[:3] - np.array((0.0, 0.0, 0.68))
                data.qfrc_applied[:3] = -800.0 * position_error - 80.0 * data.qvel[:3]
                data.qfrc_applied[3:6] = -80.0 * data.qvel[3:6]
                mujoco.mj_step(model, data)

                if data.time + 1e-9 >= next_frame_time:
                    renderer.update_scene(data, camera=camera)
                    pixels = renderer.render()
                    frame_path = frame_directory / f"frame_{frame_index:04d}.ppm"
                    _write_ppm(frame_path, pixels)
                    for state_name, state_time in state_times.items():
                        if (
                            state_name not in state_frames
                            and data.time >= state_time
                        ):
                            state_path = frame_directory / f"{state_name}.ppm"
                            _write_ppm(state_path, pixels)
                            state_frames[state_name] = state_path
                    frame_index += 1
                    next_frame_time = frame_index / FPS

        font = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        phase_overlay = (
            f"drawtext=fontfile={font}:text='NEUTRAL':x=40:y=40:fontsize=36:"
            "fontcolor=white:box=1:boxcolor=black@0.55:enable='between(t,0,0.8)',"
            f"drawtext=fontfile={font}:text='ACTIVE SHORTEN':x=40:y=40:fontsize=36:"
            "fontcolor=white:box=1:boxcolor=black@0.55:enable='between(t,0.8,2.4)',"
            f"drawtext=fontfile={font}:text='ACTIVE EXTEND':x=40:y=40:fontsize=36:"
            "fontcolor=white:box=1:boxcolor=black@0.55:enable='between(t,2.4,4.0)',"
            f"drawtext=fontfile={font}:text='PASSIVE SPRING RETURN':x=40:y=40:fontsize=36:"
            "fontcolor=white:box=1:boxcolor=black@0.55:enable='gte(t,4.0)'"
        )
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
                "-vf",
                phase_overlay,
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

        labels = {
            "neutral": "NEUTRAL - 348 mm",
            "shortened": "ACTIVE SHORTEN - 329 mm",
            "extended": "ACTIVE EXTEND - 364 mm",
        }
        labelled_paths: list[Path] = []
        for state_name in ("neutral", "shortened", "extended"):
            labelled_path = frame_directory / f"{state_name}_labelled.png"
            subprocess.run(
                [
                    "ffmpeg",
                    "-y",
                    "-loglevel",
                    "error",
                    "-i",
                    str(state_frames[state_name]),
                    "-vf",
                    (
                        f"scale=640:-1,drawtext=fontfile={font}:"
                        f"text='{labels[state_name]}':x=(w-text_w)/2:y=24:"
                        "fontsize=28:fontcolor=white:box=1:boxcolor=black@0.55"
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
                "-i",
                str(labelled_paths[0]),
                "-i",
                str(labelled_paths[1]),
                "-i",
                str(labelled_paths[2]),
                "-filter_complex",
                "hstack=inputs=3",
                "-frames:v",
                "1",
                str(states_path),
            ],
            check=True,
        )

    print(video_path)
    print(states_path)
    return video_path, states_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=Path("artifacts"),
    )
    arguments = parser.parse_args()
    render(arguments.output_directory)

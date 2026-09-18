#!/usr/bin/env python3
"""PID-only, passively absorbing 3x-track car on the Unitree Perlin strip.

Version 1 constraint: wheels run cascaded PID; hips stay at the nominal
hinge pose; strut *servos* are turned off so only the joint springs and
the x–z hip slides absorb bumps. No terrain preview, no active length
tracking.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

import mujoco
import numpy as np

if __package__:
    from scripts.balance_pid import (
        BalanceGains,
        BalanceSpeedController,
        HeadingGains,
        heading_torque_nm,
        quaternion_pitch,
        quaternion_roll_yaw,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from scripts.generate_perlin import ROOT, ROUGH_START_X_M, terrain_height_m
    from scripts.prototype_perlin import actuator_ids, joint_dof
else:
    from balance_pid import (
        BalanceGains,
        BalanceSpeedController,
        HeadingGains,
        heading_torque_nm,
        quaternion_pitch,
        quaternion_roll_yaw,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from generate_perlin import ROOT, ROUGH_START_X_M, terrain_height_m
    from prototype_perlin import actuator_ids, joint_dof

WIDE_SCENE = ROOT / "models" / "wide_car" / "scene.xml"
RESULTS_PATH = ROOT / "results" / "wide_car_pid_passive_perlin.json"
LAUNCH_X_M = 3.5
STAND_HEIGHT_M = 0.408
POSE_ACTUATORS = ("left_hip", "left_strut", "right_hip", "right_strut")
STRUT_ACTUATORS = ("left_strut", "right_strut")
WHEEL_ACTUATORS = ("left_wheel", "right_wheel")
ROLL_FAIL_DEG = 18.0
PITCH_FAIL_DEG = 32.0
LANE_Y_M = 1.15
COLLAPSE_Z_M = 0.16
DEFAULT_SPEEDS_M_S = (0.00, 1.00, 2.00, 3.00, 3.50, 3.70, 3.80, 4.00, 5.00)


@dataclass(frozen=True)
class PerlinSpeedResult:
    robot: str
    target_speed_m_s: float
    stable: bool
    held: bool
    failure_reason: str | None
    simulated_time_s: float
    distance_m: float
    peak_speed_m_s: float
    mean_speed_m_s: float
    cruise_speed_m_s: float
    maximum_roll_deg: float
    maximum_pitch_deg: float
    final_y_m: float
    reached_rough: bool
    launch_x_m: float


def apply_passive_struts(model: mujoco.MjModel) -> None:
    """Drop the strut position servos. Joint springs do the absorbing."""

    for name in STRUT_ACTUATORS:
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if actuator_id < 0:
            raise ValueError(f"missing strut actuator {name}")
        model.actuator_gainprm[actuator_id, 0] = 0.0
        model.actuator_biasprm[actuator_id, 1] = 0.0
        model.actuator_biasprm[actuator_id, 2] = 0.0


def episode_duration_s(target_speed_m_s: float, launch_x_m: float = LAUNCH_X_M) -> float:
    if target_speed_m_s < 0.05:
        return 6.0
    return 12.0


def summarize(result: PerlinSpeedResult) -> dict[str, object]:
    return {
        "robot": result.robot,
        "target_m_s": result.target_speed_m_s,
        "target_kmh": round(result.target_speed_m_s * 3.6, 2),
        "peak_kmh": round(result.peak_speed_m_s * 3.6, 2),
        "mean_kmh": round(result.mean_speed_m_s * 3.6, 2),
        "cruise_kmh": round(result.cruise_speed_m_s * 3.6, 2),
        "stable": result.stable,
        "held": result.held,
        "failure": result.failure_reason,
        "time_s": round(result.simulated_time_s, 3),
        "distance_m": round(result.distance_m, 3),
        "roll_deg": round(result.maximum_roll_deg, 2),
        "pitch_deg": round(result.maximum_pitch_deg, 2),
        "y_m": round(result.final_y_m, 3),
        "reached_rough": result.reached_rough,
        "launch_x_m": result.launch_x_m,
    }


def lane_held(result: PerlinSpeedResult) -> bool:
    if not result.stable:
        return False
    if abs(result.final_y_m) >= 1.2 or result.maximum_roll_deg >= 12.0:
        return False
    if result.target_speed_m_s < 0.05:
        return result.maximum_pitch_deg < 12.0
    if not result.reached_rough:
        return False
    if result.cruise_speed_m_s < 0.70 * result.target_speed_m_s:
        return False
    return True


def run_wide_car_episode(
    target_speed_m_s: float,
    *,
    duration_s: float | None = None,
    acceleration_m_s2: float = 1.8,
    balance_gains: BalanceGains | None = None,
    heading_gains: HeadingGains | None = None,
    start_x_m: float = LAUNCH_X_M,
    rolling_start: bool = False,
) -> PerlinSpeedResult:
    balance = balance_gains or wide_car_balance_gains()
    heading = heading_gains or wide_car_heading_gains()
    duration = duration_s if duration_s is not None else episode_duration_s(
        target_speed_m_s, start_x_m
    )
    model = mujoco.MjModel.from_xml_path(str(WIDE_SCENE))
    apply_passive_struts(model)
    data = mujoco.MjData(model)
    data.qpos[0] = start_x_m
    data.qpos[2] = STAND_HEIGHT_M + terrain_height_m(start_x_m, 0.0)
    if rolling_start and target_speed_m_s > 0.05:
        data.qvel[0] = target_speed_m_s
        radius = 0.120
        for name in ("left_wheel_joint", "right_wheel_joint"):
            data.qvel[joint_dof(model, name)] = target_speed_m_s / radius
    mujoco.mj_forward(model, data)

    pose = actuator_ids(model, POSE_ACTUATORS)
    wheels = actuator_ids(model, WHEEL_ACTUATORS)
    controller = BalanceSpeedController(float(model.opt.timestep), balance)

    speeds: list[float] = []
    rough_speeds: list[float] = []
    peak_speed = 0.0
    max_roll = 0.0
    max_pitch = 0.0
    reached_rough = False
    failure: str | None = None
    start_x = float(data.qpos[0])

    steps = int(round(duration / float(model.opt.timestep)))
    for _ in range(steps):
        time_s = float(data.time)
        if rolling_start:
            commanded = target_speed_m_s
        elif target_speed_m_s < 0.05:
            commanded = 0.0
        else:
            commanded = min(target_speed_m_s, acceleration_m_s2 * max(0.0, time_s - 0.4))
        pitch = quaternion_pitch(data.qpos[3:7])
        roll, yaw = quaternion_roll_yaw(data.qpos[3:7])
        speed = float(data.qvel[0])
        output = controller.update(
            target_speed_m_s=commanded,
            forward_speed_m_s=speed,
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
        # Both wheel joints spin about +Y, so balance torque is common-mode.
        data.ctrl[wheels[0]] = output.wheel_torque_nm + head
        data.ctrl[wheels[1]] = output.wheel_torque_nm - head
        mujoco.mj_step(model, data)

        pitch = quaternion_pitch(data.qpos[3:7])
        roll, _ = quaternion_roll_yaw(data.qpos[3:7])
        speed = float(data.qvel[0])
        x_m = float(data.qpos[0])
        peak_speed = max(peak_speed, abs(speed))
        max_roll = max(max_roll, abs(roll))
        max_pitch = max(max_pitch, abs(pitch))
        if time_s > 0.8:
            speeds.append(speed)
        if x_m >= ROUGH_START_X_M:
            reached_rough = True
            rough_speeds.append(speed)
        if abs(math.degrees(pitch)) >= PITCH_FAIL_DEG:
            failure = "pitch exceeded 32 deg"
            break
        if abs(math.degrees(roll)) >= ROLL_FAIL_DEG:
            failure = "roll exceeded 18 deg"
            break
        if float(data.qpos[2]) < COLLAPSE_Z_M:
            failure = "base collapsed"
            break
        if abs(float(data.qpos[1])) >= LANE_Y_M:
            failure = "left the lane"
            break

    cruise_source = rough_speeds if rough_speeds else speeds
    result = PerlinSpeedResult(
        robot="wide_car",
        target_speed_m_s=target_speed_m_s,
        stable=failure is None,
        held=False,
        failure_reason=failure,
        simulated_time_s=float(data.time),
        distance_m=float(data.qpos[0]) - start_x,
        peak_speed_m_s=peak_speed,
        mean_speed_m_s=float(np.mean(speeds)) if speeds else 0.0,
        cruise_speed_m_s=float(np.mean(cruise_source)) if cruise_source else 0.0,
        maximum_roll_deg=float(np.degrees(max_roll)),
        maximum_pitch_deg=float(np.degrees(max_pitch)),
        final_y_m=float(data.qpos[1]),
        reached_rough=reached_rough,
        launch_x_m=start_x,
    )
    return PerlinSpeedResult(**{**asdict(result), "held": lane_held(result)})


def sweep_wide_car(
    speeds_m_s: tuple[float, ...] = DEFAULT_SPEEDS_M_S,
) -> dict[str, object]:
    rows = [run_wide_car_episode(speed) for speed in speeds_m_s]
    held = [row for row in rows if row.held]
    moving_held = [row for row in held if row.target_speed_m_s >= 0.05]
    limit = max(moving_held, key=lambda row: row.target_speed_m_s, default=None)
    payload = {
        "robot": "wide_car",
        "version": "pid-passive-disturbance",
        "controller": (
            "cascaded speed PI + pitch PD on the wheels; hip hinges at 0; "
            "strut servos off; x-z hip slides and strut springs absorb bumps"
        ),
        "terrain": "Unitree AddPerlinHeighField, 48 m x 4 m, relief 0.20 m",
        "gains": asdict(wide_car_balance_gains()),
        "heading_gains": asdict(wide_car_heading_gains()),
        "launch_x_m": LAUNCH_X_M,
        "rough_start_x_m": ROUGH_START_X_M,
        "held_criterion": (
            "survive; |y|<1.2 m; roll<12 deg; if moving: reach x>=10 m "
            "and cruise >= 70% of target on the wrinkles"
        ),
        "episodes": [summarize(row) for row in rows],
        "max_held_speed_m_s": None if limit is None else limit.target_speed_m_s,
        "max_held_speed_kmh": None
        if limit is None
        else round(limit.target_speed_m_s * 3.6, 2),
    }
    RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
    RESULTS_PATH.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return payload


def main() -> None:
    payload = sweep_wide_car()
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()

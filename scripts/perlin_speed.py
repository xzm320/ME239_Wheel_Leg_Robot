#!/usr/bin/env python3
"""PID speed episodes on the long Unitree Perlin strip."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path

import mujoco
import numpy as np

if __package__:
    from scripts.balance_controller import (
        BalanceGains,
        BalanceSpeedController,
        HeadingGains,
        heading_torque_nm,
        perlin_wide_balance_gains,
        perlin_wide_heading_gains,
        quaternion_pitch,
        upkie_balance_gains,
        upkie_heading_gains,
    )
    from scripts.generate_perlin_track import (
        HEIGHT_SCALE_M,
        LENGTH_M,
        ROOT,
        ROUGH_START_X_M,
        WIDTH_M,
        generate_heightfield,
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
else:
    from balance_controller import (
        BalanceGains,
        BalanceSpeedController,
        HeadingGains,
        heading_torque_nm,
        perlin_wide_balance_gains,
        perlin_wide_heading_gains,
        quaternion_pitch,
        upkie_balance_gains,
        upkie_heading_gains,
    )
    from generate_perlin_track import (
        HEIGHT_SCALE_M,
        LENGTH_M,
        ROOT,
        ROUGH_START_X_M,
        WIDTH_M,
        generate_heightfield,
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

WIDE_SCENE = ROOT / "models" / "upkie" / "high_speed" / "scene_perlin.xml"
UPKIE_SCENE = ROOT / "models" / "upkie" / "upstream" / "scene_perlin.xml"
WIDE_WHEEL_RADIUS_M = 0.120
UPKIE_WHEEL_RADIUS_M = 0.055
LAUNCH_X_M = 3.0
ROUGH_X_M = 12.0
ROLL_FAIL_DEG = 18.0
PITCH_FAIL_DEG = 35.0
WIDE_PERLIN_LIMIT_M_S = 3.0
WIDE_PERLIN_FAIL_M_S = 3.4
UPKIE_PERLIN_MOVE_FAIL_M_S = 0.25


@dataclass(frozen=True)
class PerlinSpeedResult:
    robot: str
    target_speed_m_s: float
    stable: bool
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


_HEIGHTS_01: np.ndarray | None = None


def terrain_height_m(x_m: float, y_m: float) -> float:
    global _HEIGHTS_01
    if _HEIGHTS_01 is None:
        _HEIGHTS_01, _ = generate_heightfield()
    heights_01 = _HEIGHTS_01
    rows, cols = heights_01.shape
    col = np.clip(x_m / LENGTH_M * (cols - 1), 0.0, cols - 1)
    row = np.clip((WIDTH_M / 2.0 - y_m) / WIDTH_M * (rows - 1), 0.0, rows - 1)
    r0, c0 = int(row), int(col)
    r1 = min(r0 + 1, rows - 1)
    c1 = min(c0 + 1, cols - 1)
    wr, wc = row - r0, col - c0
    sample = (
        heights_01[r0, c0] * (1 - wr) * (1 - wc)
        + heights_01[r0, c1] * (1 - wr) * wc
        + heights_01[r1, c0] * wr * (1 - wc)
        + heights_01[r1, c1] * wr * wc
    )
    return float((sample - 0.5) * HEIGHT_SCALE_M)


def _ids(model: mujoco.MjModel, names: tuple[str, ...]) -> list[int]:
    result = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in names]
    if min(result) < 0:
        raise ValueError(f"missing actuator from {names}")
    return result


def _summarize(result: PerlinSpeedResult) -> dict[str, object]:
    return {
        "robot": result.robot,
        "target_kmh": round(result.target_speed_m_s * 3.6, 2),
        "peak_kmh": round(result.peak_speed_m_s * 3.6, 2),
        "mean_kmh": round(result.mean_speed_m_s * 3.6, 2),
        "cruise_kmh": round(result.cruise_speed_m_s * 3.6, 2),
        "stable": result.stable,
        "held": lane_held(result),
        "failure": result.failure_reason,
        "time_s": result.simulated_time_s,
        "distance_m": round(result.distance_m, 2),
        "roll_deg": round(result.maximum_roll_deg, 2),
        "pitch_deg": round(result.maximum_pitch_deg, 2),
        "y_m": round(result.final_y_m, 3),
        "reached_rough": result.reached_rough,
    }


def lane_held(result: PerlinSpeedResult) -> bool:
    """Survive the strip, stay in-lane, and keep mean speed near the target."""

    if not result.stable or not result.reached_rough:
        return False
    if abs(result.final_y_m) >= 1.2 or result.maximum_roll_deg >= 12.0:
        return False
    if result.target_speed_m_s < 0.05:
        return abs(result.cruise_speed_m_s) < 0.15
    if result.cruise_speed_m_s < 0.70 * result.target_speed_m_s:
        return False
    return True


def run_wide_perlin_episode(
    target_speed_m_s: float,
    *,
    duration_s: float = 8.0,
    rolling_start: bool = False,
    acceleration_m_s2: float = 0.8,
    balance_gains: BalanceGains | None = None,
    heading_gains: HeadingGains | None = None,
    start_x_m: float | None = None,
) -> PerlinSpeedResult:
    balance = balance_gains or perlin_wide_balance_gains()
    heading = heading_gains or perlin_wide_heading_gains()
    model = mujoco.MjModel.from_xml_path(str(WIDE_SCENE))
    apply_compliance_parameters(model, HUNDRED_KMH_COMPLIANCE)
    apply_wheel_track_scale(model, 3.0)
    apply_hundred_kmh_suspension(model)
    start_x = start_x_m if start_x_m is not None else (
        ROUGH_X_M if rolling_start else LAUNCH_X_M
    )
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
    if rolling_start:
        data.qvel[0] = target_speed_m_s
        for name in ("left_wheel_joint", "right_wheel_joint"):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qvel[int(model.jnt_dofadr[joint_id])] = (
                target_speed_m_s / WIDE_WHEEL_RADIUS_M
            )
    mujoco.mj_forward(model, data)
    wheels = _ids(model, ("left_wheel", "right_wheel"))
    controller = BalanceSpeedController(float(model.opt.timestep), balance)
    return _step_episode(
        model,
        data,
        robot="wide",
        target_speed_m_s=target_speed_m_s,
        duration_s=duration_s,
        rolling_start=rolling_start,
        acceleration_m_s2=acceleration_m_s2,
        start_x=start_x,
        wheel_command=lambda output, head: (
            output.wheel_torque_nm + head,
            output.wheel_torque_nm - head,
        ),
        controller_update=lambda pitch, pitch_rate, commanded, speed: controller.update(
            target_speed_m_s=commanded,
            forward_speed_m_s=speed,
            pitch_rad=pitch,
            pitch_rate_rad_s=pitch_rate,
        ),
        heading_gains=heading,
        wheels=wheels,
        collapse_z=0.16,
    )


def run_upkie_perlin_episode(
    target_speed_m_s: float,
    *,
    duration_s: float = 8.0,
    rolling_start: bool = False,
    acceleration_m_s2: float = 0.6,
    balance_gains: BalanceGains | None = None,
    heading_gains: HeadingGains | None = None,
    start_x_m: float | None = None,
) -> PerlinSpeedResult:
    balance = balance_gains or upkie_balance_gains()
    heading = heading_gains or upkie_heading_gains()
    model = mujoco.MjModel.from_xml_path(str(UPKIE_SCENE))
    start_x = start_x_m if start_x_m is not None else (
        ROUGH_X_M if rolling_start else LAUNCH_X_M
    )
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
    if rolling_start:
        data.qvel[0] = target_speed_m_s
        spins = {
            "left_wheel": -target_speed_m_s / UPKIE_WHEEL_RADIUS_M,
            "right_wheel": target_speed_m_s / UPKIE_WHEEL_RADIUS_M,
        }
        for name, spin in spins.items():
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qvel[int(model.jnt_dofadr[joint_id])] = spin
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
    controller = BalanceSpeedController(float(model.opt.timestep), balance)

    def command(output, head):
        for actuator_id in hips:
            data.ctrl[actuator_id] = 0.0
        torque = output.wheel_torque_nm
        return (
            data.qvel[left_dof] + (torque + head) / wheel_kv,
            data.qvel[right_dof] - (torque - head) / wheel_kv,
        )

    return _step_episode(
        model,
        data,
        robot="upkie",
        target_speed_m_s=target_speed_m_s,
        duration_s=duration_s,
        rolling_start=rolling_start,
        acceleration_m_s2=acceleration_m_s2,
        start_x=start_x,
        wheel_command=command,
        controller_update=lambda pitch, pitch_rate, commanded, speed: controller.update(
            target_speed_m_s=commanded,
            forward_speed_m_s=speed,
            pitch_rad=pitch,
            pitch_rate_rad_s=pitch_rate,
        ),
        heading_gains=heading,
        wheels=wheels,
        collapse_z=0.18,
    )


def _step_episode(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    robot: str,
    target_speed_m_s: float,
    duration_s: float,
    rolling_start: bool,
    acceleration_m_s2: float,
    start_x: float,
    wheel_command,
    controller_update,
    heading_gains: HeadingGains,
    wheels: list[int],
    collapse_z: float,
) -> PerlinSpeedResult:
    speeds: list[float] = []
    rolls: list[float] = []
    pitches: list[float] = []
    failure: str | None = None
    for _ in range(round(duration_s / model.opt.timestep)):
        time_s = float(data.time)
        if rolling_start:
            commanded = target_speed_m_s
        elif time_s < 0.8:
            commanded = 0.0
        else:
            commanded = min(target_speed_m_s, acceleration_m_s2 * (time_s - 0.8))
        pitch = quaternion_pitch(data.qpos[3:7])
        roll, yaw = quaternion_roll_yaw(data.qpos[3:7])
        output = controller_update(pitch, float(data.qvel[4]), commanded, float(data.qvel[0]))
        head = heading_torque_nm(
            heading_gains,
            lateral_m=float(data.qpos[1]),
            lateral_speed_m_s=float(data.qvel[1]),
            yaw_rad=yaw,
            yaw_rate_rad_s=float(data.qvel[5]),
            roll_rad=roll,
            roll_rate_rad_s=float(data.qvel[3]),
            forward_speed_m_s=float(data.qvel[0]),
        )
        left, right = wheel_command(output, head)
        data.ctrl[wheels[0]] = left
        data.ctrl[wheels[1]] = right
        mujoco.mj_step(model, data)
        roll, _ = quaternion_roll_yaw(data.qpos[3:7])
        pitch = quaternion_pitch(data.qpos[3:7])
        speeds.append(float(data.qvel[0]))
        rolls.append(roll)
        pitches.append(pitch)
        if not np.isfinite(data.qpos).all():
            failure = "non-finite state"
        elif data.qpos[2] < collapse_z:
            failure = "trunk height collapsed"
        elif abs(roll) > math.radians(ROLL_FAIL_DEG):
            failure = f"roll exceeded {ROLL_FAIL_DEG:.0f} deg"
        elif abs(pitch) > math.radians(PITCH_FAIL_DEG):
            failure = f"pitch exceeded {PITCH_FAIL_DEG:.0f} deg"
        elif abs(data.qpos[1]) > WIDTH_M / 2.0 - 0.25:
            failure = "left the strip"
        if failure is not None:
            break
    speed_array = np.asarray(speeds)
    window = max(1, int(1.0 / model.opt.timestep))
    cruise = float(np.mean(speed_array[-window:]))
    return PerlinSpeedResult(
        robot=robot,
        target_speed_m_s=target_speed_m_s,
        stable=failure is None,
        failure_reason=failure,
        simulated_time_s=round(len(speeds) * model.opt.timestep, 6),
        distance_m=float(data.qpos[0] - start_x),
        peak_speed_m_s=float(np.max(speed_array)),
        mean_speed_m_s=float(np.mean(speed_array)),
        cruise_speed_m_s=cruise,
        maximum_roll_deg=float(np.degrees(np.max(np.abs(rolls)))),
        maximum_pitch_deg=float(np.degrees(np.max(np.abs(pitches)))),
        final_y_m=float(data.qpos[1]),
        reached_rough=float(data.qpos[0]) >= ROUGH_START_X_M,
    )


def result_dict(result: PerlinSpeedResult) -> dict[str, object]:
    return asdict(result)


def sweep() -> dict[str, object]:
    wide_hold = run_wide_perlin_episode(
        WIDE_PERLIN_LIMIT_M_S, duration_s=8.0, rolling_start=True
    )
    wide_fail = run_wide_perlin_episode(
        WIDE_PERLIN_FAIL_M_S, duration_s=6.0, rolling_start=True
    )
    upkie_stand = run_upkie_perlin_episode(0.0, duration_s=6.0, rolling_start=True)
    upkie_move = run_upkie_perlin_episode(
        UPKIE_PERLIN_MOVE_FAIL_M_S, duration_s=4.0, rolling_start=True
    )
    return {
        "wide_limit_m_s": WIDE_PERLIN_LIMIT_M_S,
        "wide_hold": _summarize(wide_hold),
        "wide_fail": _summarize(wide_fail),
        "upkie_stand": _summarize(upkie_stand),
        "upkie_move": _summarize(upkie_move),
    }


if __name__ == "__main__":
    import json

    print(json.dumps(sweep(), indent=2))

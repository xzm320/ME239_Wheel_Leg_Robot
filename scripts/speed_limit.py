#!/usr/bin/env python3
"""Flat-pad speed-limit episodes for the 3x car and the 1x four-bar prototype."""

from __future__ import annotations

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
        prototype_balance_gains,
        prototype_heading_gains,
        quaternion_pitch,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from scripts.high_speed_evaluation import (
        HighSpeedResult,
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
        prototype_balance_gains,
        prototype_heading_gains,
        quaternion_pitch,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from high_speed_evaluation import (
        HighSpeedResult,
        apply_hundred_kmh_suspension,
        apply_wheel_track_scale,
    )
    from joint_terrain_controller import (
        HUNDRED_KMH_COMPLIANCE,
        apply_compliance_parameters,
        quaternion_roll_yaw,
    )

ROOT = Path(__file__).resolve().parents[1]
WIDE_MODEL_PATH = ROOT / "models" / "upkie" / "high_speed" / "scene.xml"
PROTOTYPE_MODEL_PATH = ROOT / "models" / "upkie" / "four_bar" / "scene.xml"
WHEEL_RADIUS_M = 0.120
START_X_M = -12000.0
WIDE_LIMIT_KMH = 195.0
PROTOTYPE_LIMIT_KMH = 180.0
PAD_ACCEL_M_S2 = 0.50


def wide_car_gains() -> tuple[BalanceGains, HeadingGains]:
    return wide_car_balance_gains(), wide_car_heading_gains()


def prototype_gains() -> tuple[BalanceGains, HeadingGains]:
    return prototype_balance_gains(), prototype_heading_gains()


def _ids(model: mujoco.MjModel, names: tuple[str, ...]) -> list[int]:
    result = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in names]
    if min(result) < 0:
        raise ValueError(f"missing actuator from {names}")
    return result


def _make_floor_infinite(model: mujoco.MjModel) -> None:
    for name in ("floor", "launch_pad"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id >= 0 and model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_PLANE:
            model.geom_size[geom_id, :2] = 0.0


def run_pad_episode(
    *,
    model_path: Path,
    target_speed_m_s: float,
    balance_gains: BalanceGains,
    heading_gains: HeadingGains,
    duration_s: float = 12.0,
    acceleration_m_s2: float | None = 0.70,
    rolling_start: bool = False,
    wheel_track_scale: float | None = None,
    start_x_m: float = START_X_M,
    soften_struts: bool = False,
) -> HighSpeedResult:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    _make_floor_infinite(model)
    apply_compliance_parameters(model, HUNDRED_KMH_COMPLIANCE)
    if wheel_track_scale is not None:
        apply_wheel_track_scale(model, wheel_track_scale)
    if soften_struts:
        apply_hundred_kmh_suspension(model)
    data = mujoco.MjData(model)
    data.qpos[:7] = (start_x_m, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    wheel_actuators = _ids(model, ("left_wheel", "right_wheel"))
    wheel_joints = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in ("left_wheel_joint", "right_wheel_joint")
    ]
    trunk_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
    controller = BalanceSpeedController(float(model.opt.timestep), balance_gains)

    if rolling_start:
        data.qvel[0] = target_speed_m_s
        for joint_id in wheel_joints:
            data.qvel[int(model.jnt_dofadr[joint_id])] = target_speed_m_s / WHEEL_RADIUS_M
        mujoco.mj_forward(model, data)
        controller.reset()

    initial_com_height = float(data.subtree_com[trunk_body, 2])
    speeds: list[float] = []
    com_heights: list[float] = []
    accelerations: list[float] = []
    rolls: list[float] = []
    pitches: list[float] = []
    torques: list[float] = []
    failure_reason: str | None = None

    for _ in range(round(duration_s / model.opt.timestep)):
        time_s = float(data.time)
        if rolling_start:
            commanded = target_speed_m_s
        elif time_s < 1.0:
            commanded = 0.0
        else:
            commanded = min(target_speed_m_s, acceleration_m_s2 * (time_s - 1.0))
        pitch = quaternion_pitch(data.qpos[3:7])
        roll, yaw = quaternion_roll_yaw(data.qpos[3:7])
        output = controller.update(
            target_speed_m_s=commanded,
            forward_speed_m_s=float(data.qvel[0]),
            pitch_rad=pitch,
            pitch_rate_rad_s=float(data.qvel[4]),
        )
        heading = heading_torque_nm(
            heading_gains,
            lateral_m=float(data.qpos[1]),
            lateral_speed_m_s=float(data.qvel[1]),
            yaw_rad=yaw,
            yaw_rate_rad_s=float(data.qvel[5]),
            roll_rad=roll,
            roll_rate_rad_s=float(data.qvel[3]),
            forward_speed_m_s=float(data.qvel[0]),
        )
        data.ctrl[wheel_actuators] = (
            output.wheel_torque_nm + heading,
            output.wheel_torque_nm - heading,
        )
        mujoco.mj_step(model, data)
        roll, _ = quaternion_roll_yaw(data.qpos[3:7])
        pitch = quaternion_pitch(data.qpos[3:7])
        speeds.append(float(data.qvel[0]))
        com_heights.append(float(data.subtree_com[trunk_body, 2]))
        accelerations.append(float(data.qacc[2]))
        rolls.append(roll)
        pitches.append(pitch)
        torques.append(abs(output.wheel_torque_nm))
        if not np.isfinite(data.qpos).all():
            failure_reason = "non-finite state"
        elif data.qpos[2] < 0.14:
            failure_reason = "trunk height collapsed"
        elif abs(roll) > math.radians(15.0):
            failure_reason = "roll exceeded 15 deg"
        elif abs(pitch) > math.radians(30.0):
            failure_reason = "pitch exceeded 30 deg"
        if failure_reason is not None:
            break

    speed_array = np.asarray(speeds)
    com_array = np.asarray(com_heights)
    return HighSpeedResult(
        target_speed_m_s=target_speed_m_s,
        stable=failure_reason is None,
        failure_reason=failure_reason,
        simulated_time_s=round(len(speeds) * model.opt.timestep, 6),
        distance_m=float(data.qpos[0] - start_x_m),
        mean_speed_m_s=float(np.mean(speed_array)),
        speed_rmse_m_s=float(np.sqrt(np.mean((speed_array - target_speed_m_s) ** 2))),
        com_height_std_mm=float(np.std(com_array) * 1000.0),
        maximum_com_deviation_mm=float(
            np.max(np.abs(com_array - initial_com_height)) * 1000.0
        ),
        vertical_acceleration_rms_m_s2=float(
            np.sqrt(np.mean(np.asarray(accelerations) ** 2))
        ),
        maximum_roll_deg=float(np.degrees(np.max(np.abs(rolls)))),
        maximum_pitch_deg=float(np.degrees(np.max(np.abs(pitches)))),
        maximum_wheel_torque_nm=float(np.max(torques)),
        peak_speed_m_s=float(np.max(speed_array)),
        final_y_m=float(data.qpos[1]),
        wheel_track_scale=1.0 if wheel_track_scale is None else wheel_track_scale,
    )


def lane_held(result: HighSpeedResult) -> bool:
    return (
        result.stable
        and abs(result.final_y_m) < 1.0
        and result.maximum_roll_deg < 5.0
    )


def rolling_limit_episode(
    *,
    model_path: Path,
    kmh: float,
    wheel_track_scale: float | None,
    soften_struts: bool,
    balance_gains: BalanceGains,
    heading_gains: HeadingGains,
    duration_s: float = 10.0,
) -> HighSpeedResult:
    return run_pad_episode(
        model_path=model_path,
        target_speed_m_s=kmh / 3.6,
        balance_gains=balance_gains,
        heading_gains=heading_gains,
        duration_s=duration_s,
        rolling_start=True,
        wheel_track_scale=wheel_track_scale,
        soften_struts=soften_struts,
        start_x_m=-2000.0,
    )


def summarize(result: HighSpeedResult) -> dict[str, object]:
    payload = {
        "stable": result.stable,
        "failure": result.failure_reason,
        "target_kmh": round(result.target_speed_m_s * 3.6, 2),
        "peak_kmh": round(result.peak_speed_m_s * 3.6, 2),
        "mean_kmh": round(result.mean_speed_m_s * 3.6, 2),
        "time_s": result.simulated_time_s,
        "distance_m": round(result.distance_m, 1),
        "roll_deg": round(result.maximum_roll_deg, 3),
        "pitch_deg": round(result.maximum_pitch_deg, 3),
        "y_m": round(result.final_y_m, 3),
        "torque_nm": round(result.maximum_wheel_torque_nm, 3),
        "held": lane_held(result),
    }
    return payload

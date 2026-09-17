"""High-speed rolling-start evaluation on a long smooth rough track."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math
from pathlib import Path

import mujoco
import numpy as np

if __package__:
    from scripts.balance_controller import BalanceGains, quaternion_pitch
    from scripts.joint_terrain_controller import (
        ComplianceParameters,
        JointTerrainController,
        TerrainControlGains,
        apply_compliance_parameters,
        compliance_parameters_for_speed,
        quaternion_roll_yaw,
        sample_preview_ground_heights,
        strut_spring_compensation,
    )
else:
    from balance_controller import BalanceGains, quaternion_pitch
    from joint_terrain_controller import (
        ComplianceParameters,
        JointTerrainController,
        TerrainControlGains,
        apply_compliance_parameters,
        compliance_parameters_for_speed,
        quaternion_roll_yaw,
        sample_preview_ground_heights,
        strut_spring_compensation,
    )

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "upkie"
    / "high_speed"
    / "scene.xml"
)
WHEEL_RADIUS_M = 0.120
START_X_M = -280.0


@dataclass(frozen=True)
class HighSpeedResult:
    target_speed_m_s: float
    stable: bool
    failure_reason: str | None
    simulated_time_s: float
    distance_m: float
    mean_speed_m_s: float
    speed_rmse_m_s: float
    com_height_std_mm: float
    maximum_com_deviation_mm: float
    vertical_acceleration_rms_m_s2: float
    maximum_roll_deg: float
    maximum_pitch_deg: float
    maximum_wheel_torque_nm: float


def high_speed_balance_gains() -> BalanceGains:
    return BalanceGains(
        speed_kp=0.030,
        speed_ki=0.0,
        pitch_reference_limit_rad=0.070,
        pitch_reference_rate_rad_s=0.15,
        wheel_torque_limit_nm=6.0,
    )


def high_speed_terrain_gains(
    compliance: ComplianceParameters,
) -> TerrainControlGains:
    return replace(
        TerrainControlGains(),
        preview_base_m=0.20,
        preview_time_s=0.08,
        stage_rate_limit_m_s=0.50,
        strut_spring_compensation=strut_spring_compensation(compliance),
    )


def _object_ids(
    model: mujoco.MjModel,
    object_type: int,
    names: tuple[str, ...],
) -> list[int]:
    result = [
        mujoco.mj_name2id(model, object_type, name)
        for name in names
    ]
    if min(result) < 0:
        raise ValueError(f"missing MuJoCo object from {names}")
    return result


def run_high_speed_episode(
    target_speed_m_s: float,
    *,
    compliance: ComplianceParameters | None = None,
    terrain_gains: TerrainControlGains | None = None,
    balance_gains: BalanceGains | None = None,
    simulation_timestep_s: float | None = None,
    tire_contact_time_constant_s: float | None = None,
    tire_contact_damping_ratio: float = 1.0,
    tire_friction: float | None = None,
    duration_s: float = 12.0,
) -> HighSpeedResult:
    mechanical = compliance or compliance_parameters_for_speed(target_speed_m_s)
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    if simulation_timestep_s is not None:
        model.opt.timestep = simulation_timestep_s
    if tire_contact_time_constant_s is not None or tire_friction is not None:
        for body_name in ("left_wheel", "right_wheel"):
            body_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )
            geom_id = int(model.body_geomadr[body_id])
            if tire_contact_time_constant_s is not None:
                model.geom_solref[geom_id] = (
                    tire_contact_time_constant_s,
                    tire_contact_damping_ratio,
                )
            if tire_friction is not None:
                model.geom_friction[geom_id, 0] = tire_friction
    apply_compliance_parameters(model, mechanical)
    control_parameters = terrain_gains or high_speed_terrain_gains(mechanical)
    balance_parameters = balance_gains or high_speed_balance_gains()
    data = mujoco.MjData(model)
    data.qpos[:7] = (START_X_M, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)

    wheel_actuators = _object_ids(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        ("left_wheel", "right_wheel"),
    )
    for actuator_id in wheel_actuators:
        model.actuator_ctrlrange[actuator_id] = (
            -balance_parameters.wheel_torque_limit_nm,
            balance_parameters.wheel_torque_limit_nm,
        )
        model.actuator_forcerange[actuator_id] = (
            -balance_parameters.wheel_torque_limit_nm,
            balance_parameters.wheel_torque_limit_nm,
        )
    strut_actuators = _object_ids(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        ("left_strut", "right_strut"),
    )
    wheel_bodies = tuple(
        _object_ids(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            ("left_wheel_node", "right_wheel_node"),
        )
    )
    wheel_joints = _object_ids(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        ("left_wheel_joint", "right_wheel_joint"),
    )
    trunk_body = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "trunk"
    )

    controller = JointTerrainController(
        float(model.opt.timestep),
        control_parameters,
        balance_parameters,
    )
    ground = sample_preview_ground_heights(
        model, data, wheel_bodies, 0.0, control_parameters
    )

    # Settle contact penetration before imposing a no-slip rolling state.
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
    data.qvel[0] = target_speed_m_s
    for joint_id in wheel_joints:
        data.qvel[int(model.jnt_dofadr[joint_id])] = (
            target_speed_m_s / WHEEL_RADIUS_M
        )
    mujoco.mj_forward(model, data)
    controller = JointTerrainController(
        float(model.opt.timestep),
        control_parameters,
        balance_parameters,
    )
    initial_com_height = float(data.subtree_com[trunk_body, 2])

    speeds: list[float] = []
    com_heights: list[float] = []
    accelerations: list[float] = []
    rolls: list[float] = []
    pitches: list[float] = []
    torques: list[float] = []
    failure_reason: str | None = None
    ground = sample_preview_ground_heights(
        model,
        data,
        wheel_bodies,
        target_speed_m_s,
        control_parameters,
    )

    for step in range(round(duration_s / model.opt.timestep)):
        if step % 5 == 0:
            ground = sample_preview_ground_heights(
                model,
                data,
                wheel_bodies,
                float(data.qvel[0]),
                control_parameters,
            )
        output = controller.update(
            target_speed_m_s=target_speed_m_s,
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

        roll, _ = quaternion_roll_yaw(data.qpos[3:7])
        pitch = quaternion_pitch(data.qpos[3:7])
        speeds.append(float(data.qvel[0]))
        com_heights.append(float(data.subtree_com[trunk_body, 2]))
        accelerations.append(float(data.qacc[2]))
        rolls.append(roll)
        pitches.append(pitch)
        torques.append(float(np.max(np.abs(output.wheel_torques_nm))))

        if not np.isfinite(data.qpos).all():
            failure_reason = "non-finite state"
        elif data.qpos[2] < 0.25:
            failure_reason = "trunk height below 0.25 m"
        elif abs(roll) > math.radians(15.0):
            failure_reason = "roll exceeded 15 deg"
        elif abs(pitch) > math.radians(30.0):
            failure_reason = "pitch exceeded 30 deg"
        if failure_reason is not None:
            break

    speed_array = np.asarray(speeds)
    com_array = np.asarray(com_heights)
    acceleration_array = np.asarray(accelerations)
    return HighSpeedResult(
        target_speed_m_s=target_speed_m_s,
        stable=failure_reason is None,
        failure_reason=failure_reason,
        simulated_time_s=round(len(speeds) * model.opt.timestep, 6),
        distance_m=float(data.qpos[0] - START_X_M),
        mean_speed_m_s=float(np.mean(speed_array)),
        speed_rmse_m_s=float(
            np.sqrt(np.mean((speed_array - target_speed_m_s) ** 2))
        ),
        com_height_std_mm=float(np.std(com_array) * 1000.0),
        maximum_com_deviation_mm=float(
            np.max(np.abs(com_array - initial_com_height)) * 1000.0
        ),
        vertical_acceleration_rms_m_s2=float(
            np.sqrt(np.mean(acceleration_array**2))
        ),
        maximum_roll_deg=float(np.degrees(np.max(np.abs(rolls)))),
        maximum_pitch_deg=float(np.degrees(np.max(np.abs(pitches)))),
        maximum_wheel_torque_nm=float(np.max(torques)),
    )


def result_dict(result: HighSpeedResult) -> dict[str, object]:
    return asdict(result)

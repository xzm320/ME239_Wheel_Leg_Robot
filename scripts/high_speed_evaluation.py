"""High-speed rolling-start evaluation on a long smooth rough track."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math
from pathlib import Path

import mujoco
import numpy as np

if __package__:
    from scripts.balance_controller import BalanceGains, BalanceSpeedController, quaternion_pitch
    from scripts.joint_terrain_controller import (
        ComplianceParameters,
        HUNDRED_KMH_COMPLIANCE,
        JointTerrainController,
        TerrainControlGains,
        apply_compliance_parameters,
        apply_four_bar_hinge_damping,
        compliance_parameters_for_speed,
        quaternion_roll_yaw,
        sample_preview_ground_heights,
        stage_extension_from_leg_height,
        strut_spring_compensation,
    )
else:
    from balance_controller import BalanceGains, BalanceSpeedController, quaternion_pitch
    from joint_terrain_controller import (
        ComplianceParameters,
        HUNDRED_KMH_COMPLIANCE,
        JointTerrainController,
        TerrainControlGains,
        apply_compliance_parameters,
        apply_four_bar_hinge_damping,
        compliance_parameters_for_speed,
        quaternion_roll_yaw,
        sample_preview_ground_heights,
        stage_extension_from_leg_height,
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
NOMINAL_HIP_Y_M = 0.1137
START_X_M = -1100.0
CROUCHED_TRUNK_HEIGHT_M = 0.408
TRUNK_COM_Z_OFFSET_M = 0.0
ROUGH_START_X_M = -140.0
TARGET_100_KMH_M_S = 100.0 / 3.6
HUNDRED_KMH_WHEEL_TRACK_SCALE = 3.0
HUNDRED_KMH_STRUT_KP = 2800.0
HUNDRED_KMH_STRUT_KV = 80.0


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
    peak_speed_m_s: float = 0.0
    final_y_m: float = 0.0
    wheel_track_scale: float = 1.0


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
        yaw_kp=0.0,
        yaw_kd=0.0,
        strut_spring_compensation=strut_spring_compensation(compliance),
    )


def hundred_kmh_balance_gains() -> BalanceGains:
    """Lean-limited speed PI that can finish a 100 km/h ramp on the long pad."""

    return BalanceGains(
        pitch_kp=465.6879,
        pitch_kd=55.0,
        speed_kp=0.030,
        speed_ki=0.0,
        pitch_reference_limit_rad=0.085,
        pitch_reference_rate_rad_s=0.15,
        wheel_torque_limit_nm=6.0,
    )


def hundred_kmh_heading_torque_nm(
    *,
    lateral_m: float,
    lateral_speed_m_s: float,
    yaw_rad: float,
    yaw_rate_rad_s: float,
    roll_rad: float,
    roll_rate_rad_s: float,
    forward_speed_m_s: float,
) -> float:
    """Weak yaw hold plus roll PD. Strong yaw differentials roll the 3x track."""

    if forward_speed_m_s < 8.0:
        return 0.0
    scale = float(np.clip((forward_speed_m_s - 8.0) / 12.0, 0.0, 1.0))
    torque = (
        0.58 * yaw_rad
        + 0.17 * yaw_rate_rad_s
        + 0.025 * lateral_m
        + 0.02 * lateral_speed_m_s
        - 3.4 * roll_rad
        - 0.50 * roll_rate_rad_s
    )
    return float(np.clip(scale * torque, -0.22, 0.22))


def apply_hundred_kmh_suspension(model: mujoco.MjModel) -> None:
    """Let the diamond four-bar yield over whoops instead of locking at kp=8000."""

    for name in ("left_strut", "right_strut"):
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
        )
        if actuator_id < 0:
            raise ValueError(f"actuator not found: {name}")
        model.actuator_gainprm[actuator_id, 0] = HUNDRED_KMH_STRUT_KP
        model.actuator_biasprm[actuator_id, 1] = -HUNDRED_KMH_STRUT_KP
        model.actuator_biasprm[actuator_id, 2] = -HUNDRED_KMH_STRUT_KV


def apply_wheel_track_scale(model: mujoco.MjModel, scale: float) -> None:
    """Scale left/right hip Y so wheel track stays within 4x of the original."""

    if scale < 1.0 or scale > 4.0:
        raise ValueError(f"wheel track scale {scale} is outside 1x–4x")
    for name, sign in (("left_hip_carrier", 1.0), ("right_hip_carrier", -1.0)):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise ValueError(f"body not found: {name}")
        model.body_pos[body_id, 1] = sign * NOMINAL_HIP_Y_M * scale


def lower_trunk_center_of_mass(
    model: mujoco.MjModel,
    offset_z_m: float = TRUNK_COM_Z_OFFSET_M,
) -> None:
    trunk_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
    if trunk_body < 0:
        raise ValueError("trunk body not found")
    model.body_ipos[trunk_body, 2] += offset_z_m


def hundred_kmh_terrain_gains(
    compliance: ComplianceParameters,
) -> TerrainControlGains:
    return replace(
        TerrainControlGains(),
        target_trunk_height_m=CROUCHED_TRUNK_HEIGHT_M,
        preview_base_m=0.40,
        preview_time_s=0.08,
        preview_symmetry=1.0,
        stage_rate_limit_m_s=0.05,
        height_kp=0.0,
        height_kd=0.0,
        roll_kp=0.0,
        roll_kd=0.0,
        yaw_kp=0.0,
        yaw_kd=0.0,
        roll_torque_kp=0.0,
        roll_torque_kd=0.0,
        roll_torque_ki=0.0,
        differential_torque_limit_nm=0.0,
        strut_spring_compensation=strut_spring_compensation(compliance),
    )


STRUT_JOINT_NAMES = (
    "left_strut_extension",
    "left_strut_extension_stage2",
    "right_strut_extension",
    "right_strut_extension_stage2",
)


def pose_at_trunk_height(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    world_x_m: float,
    trunk_height_m: float,
    gains: TerrainControlGains,
) -> None:
    """Place the free trunk and both telescopic stages at a crouched stance."""

    stage = stage_extension_from_leg_height(
        trunk_height_m + gains.hip_offset_z_m - gains.wheel_radius_m,
        gains,
    )
    data.qpos[:7] = (world_x_m, 0.0, trunk_height_m, 1.0, 0.0, 0.0, 0.0)
    for joint_name in STRUT_JOINT_NAMES:
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        data.qpos[int(model.jnt_qposadr[joint_id])] = stage
    mujoco.mj_forward(model, data)


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
    rolling_start: bool = True,
    acceleration_m_s2: float = 2.5,
    start_x_m: float | None = None,
    wheel_only: bool = False,
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
    data = mujoco.MjData(model)
    launch_x = (
        start_x_m
        if start_x_m is not None
        else (START_X_M if not rolling_start else -600.0)
    )
    data.qpos[:7] = (launch_x, 0.0, 0.408, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)

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

    if rolling_start:
        settle_s = 1.0
        for _ in range(round(settle_s / model.opt.timestep)):
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
            if not wheel_only:
                data.ctrl[strut_actuators] = output.strut_controls_m
            mujoco.mj_step(model, data)

        data.qvel[:] = 0.0
        data.qvel[0] = target_speed_m_s
        for joint_id in wheel_joints:
            data.qvel[int(model.jnt_dofadr[joint_id])] = (
                target_speed_m_s / WHEEL_RADIUS_M
            )
        mujoco.mj_forward(model, data)
        controller.balance.reset()
        controller.roll_integral = 0.0
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
        if not wheel_only and step % 5 == 0:
            ground = sample_preview_ground_heights(
                model,
                data,
                wheel_bodies,
                float(data.qvel[0]),
                control_parameters,
            )
        commanded_speed = target_speed_m_s
        if not rolling_start:
            elapsed = step * model.opt.timestep
            if elapsed < 1.0:
                commanded_speed = 0.0
            else:
                commanded_speed = min(
                    target_speed_m_s,
                    acceleration_m_s2 * (elapsed - 1.0),
                )
        if wheel_only:
            pitch = quaternion_pitch(data.qpos[3:7])
            balance = controller.balance.update(
                target_speed_m_s=commanded_speed,
                forward_speed_m_s=float(data.qvel[0]),
                pitch_rad=pitch,
                pitch_rate_rad_s=float(data.qvel[4]),
            )
            data.ctrl[wheel_actuators] = balance.wheel_torque_nm
            applied_torque = abs(balance.wheel_torque_nm)
        else:
            output = controller.update(
                target_speed_m_s=commanded_speed,
                forward_speed_m_s=float(data.qvel[0]),
                trunk_height_m=float(data.qpos[2]),
                vertical_speed_m_s=float(data.qvel[2]),
                quaternion_wxyz=data.qpos[3:7],
                angular_velocity_xyz=data.qvel[3:6],
                preview_ground_heights_m=ground,
            )
            data.ctrl[wheel_actuators] = output.wheel_torques_nm
            data.ctrl[strut_actuators] = output.strut_controls_m
            applied_torque = float(np.max(np.abs(output.wheel_torques_nm)))
        mujoco.mj_step(model, data)

        roll, _ = quaternion_roll_yaw(data.qpos[3:7])
        pitch = quaternion_pitch(data.qpos[3:7])
        speeds.append(float(data.qvel[0]))
        com_heights.append(float(data.subtree_com[trunk_body, 2]))
        accelerations.append(float(data.qacc[2]))
        rolls.append(roll)
        pitches.append(pitch)
        torques.append(applied_torque)

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
    acceleration_array = np.asarray(accelerations)
    return HighSpeedResult(
        target_speed_m_s=target_speed_m_s,
        stable=failure_reason is None,
        failure_reason=failure_reason,
        simulated_time_s=round(len(speeds) * model.opt.timestep, 6),
        distance_m=float(data.qpos[0] - launch_x),
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
        peak_speed_m_s=float(np.max(speed_array)),
        final_y_m=float(data.qpos[1]),
        wheel_track_scale=1.0,
    )


def run_hundred_kmh_episode(
    target_speed_m_s: float = TARGET_100_KMH_M_S,
    duration_s: float = 64.0,
    acceleration_m_s2: float = 0.70,
    *,
    compliance: ComplianceParameters | None = None,
    hinge_damping_n_m_s_rad: float | None = None,
    trunk_com_z_offset_m: float = 0.0,
    balance_gains: BalanceGains | None = None,
    start_x_m: float | None = None,
    start_y_m: float = 0.0,
    wheel_track_scale: float = HUNDRED_KMH_WHEEL_TRACK_SCALE,
) -> HighSpeedResult:
    """Accelerate with wheel PID, optional lowered trunk CoM, and passive struts."""

    mechanical = compliance or HUNDRED_KMH_COMPLIANCE
    launch_x = START_X_M if start_x_m is None else start_x_m
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    apply_compliance_parameters(model, mechanical)
    apply_wheel_track_scale(model, wheel_track_scale)
    apply_hundred_kmh_suspension(model)
    if hinge_damping_n_m_s_rad is not None:
        apply_four_bar_hinge_damping(model, hinge_damping_n_m_s_rad)
    if trunk_com_z_offset_m:
        lower_trunk_center_of_mass(model, trunk_com_z_offset_m)
    data = mujoco.MjData(model)
    data.qpos[:7] = (launch_x, start_y_m, 0.408, 1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)
    wheel_actuators = _object_ids(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, ("left_wheel", "right_wheel")
    )
    trunk_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
    controller = BalanceSpeedController(
        float(model.opt.timestep),
        balance_gains or hundred_kmh_balance_gains(),
    )
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
        commanded = (
            0.0
            if time_s < 1.0
            else min(target_speed_m_s, acceleration_m_s2 * (time_s - 1.0))
        )
        pitch = quaternion_pitch(data.qpos[3:7])
        roll, yaw = quaternion_roll_yaw(data.qpos[3:7])
        output = controller.update(
            target_speed_m_s=commanded,
            forward_speed_m_s=float(data.qvel[0]),
            pitch_rad=pitch,
            pitch_rate_rad_s=float(data.qvel[4]),
        )
        heading_torque = hundred_kmh_heading_torque_nm(
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
    acceleration_array = np.asarray(accelerations)
    return HighSpeedResult(
        target_speed_m_s=target_speed_m_s,
        stable=failure_reason is None,
        failure_reason=failure_reason,
        simulated_time_s=round(len(speeds) * model.opt.timestep, 6),
        distance_m=float(data.qpos[0] - launch_x),
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
        peak_speed_m_s=float(np.max(speed_array)),
        final_y_m=float(data.qpos[1]),
        wheel_track_scale=wheel_track_scale,
    )


def result_dict(result: HighSpeedResult) -> dict[str, object]:
    return asdict(result)

"""Joint wheel/attitude/leg-height controller for rough terrain."""

from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np

if __package__:
    from scripts.balance_controller import (
        BalanceGains,
        BalanceSpeedController,
        quaternion_pitch,
    )
else:
    from balance_controller import BalanceGains, BalanceSpeedController, quaternion_pitch


@dataclass(frozen=True)
class TerrainControlGains:
    target_trunk_height_m: float = 0.408
    wheel_radius_m: float = 0.120
    hip_offset_z_m: float = 0.0602
    link_length_m: float = 0.190
    nominal_crossbar_length_m: float = 0.150
    stage_min_m: float = -0.020
    stage_max_m: float = 0.094
    stage_rate_limit_m_s: float = 0.30
    strut_spring_compensation: float = 1.40
    height_kp: float = 0.40
    height_kd: float = 0.10
    roll_kp: float = 0.290419
    roll_kd: float = 0.015922
    yaw_kp: float = 8.0
    yaw_kd: float = 1.0
    roll_torque_kp: float = 0.0
    roll_torque_kd: float = 0.0
    roll_torque_ki: float = 0.0
    roll_integral_limit: float = 0.4
    differential_torque_limit_nm: float = 2.5
    preview_base_m: float = 0.080
    preview_time_s: float = 0.030
    preview_symmetry: float = 0.0


@dataclass(frozen=True)
class ComplianceParameters:
    """Mechanical compliance values shared by both sides."""

    hip_x_stiffness_n_m: float = 3000.0
    hip_x_damping_n_s_m: float = 100.0
    hip_z_stiffness_n_m: float = 7000.0
    hip_z_damping_n_s_m: float = 180.0
    strut_equivalent_stiffness_n_m: float = 800.0
    strut_equivalent_damping_n_s_m: float = 50.0


HIGH_SPEED_COMPLIANCE = ComplianceParameters(
    hip_x_stiffness_n_m=3000.0,
    hip_x_damping_n_s_m=140.0,
    hip_z_stiffness_n_m=7000.0,
    hip_z_damping_n_s_m=220.0,
    strut_equivalent_stiffness_n_m=800.0,
    strut_equivalent_damping_n_s_m=60.0,
)

HUNDRED_KMH_COMPLIANCE = ComplianceParameters(
    hip_x_stiffness_n_m=3000.0,
    hip_x_damping_n_s_m=100.0,
    hip_z_stiffness_n_m=7000.0,
    hip_z_damping_n_s_m=180.0,
    strut_equivalent_stiffness_n_m=800.0,
    strut_equivalent_damping_n_s_m=50.0,
)
HUNDRED_KMH_HINGE_DAMPING_N_M_S_RAD = 0.15
FOUR_BAR_HINGE_JOINTS = (
    "left_hip",
    "left_front_knee_hinge",
    "left_rear_hip_hinge",
    "left_rear_knee_hinge",
    "left_strut_angle",
    "left_front_bottom_hinge",
    "right_hip",
    "right_front_knee_hinge",
    "right_rear_hip_hinge",
    "right_rear_knee_hinge",
    "right_strut_angle",
    "right_front_bottom_hinge",
)


@dataclass(frozen=True)
class JointControlOutput:
    wheel_torques_nm: np.ndarray
    strut_controls_m: np.ndarray
    stage_targets_m: np.ndarray
    pitch_reference_rad: float


def quaternion_roll_yaw(quaternion_wxyz: np.ndarray) -> tuple[float, float]:
    w, x, y, z = quaternion_wxyz
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, yaw


def stage_extension_from_leg_height(
    leg_height_m: float,
    gains: TerrainControlGains | None = None,
) -> float:
    parameters = gains or TerrainControlGains()
    maximum_height = 2.0 * parameters.link_length_m
    bounded_height = float(np.clip(leg_height_m, 0.0, maximum_height))
    half_crossbar = math.sqrt(
        max(parameters.link_length_m**2 - (bounded_height / 2.0) ** 2, 0.0)
    )
    stage_extension = (
        2.0 * half_crossbar - parameters.nominal_crossbar_length_m
    ) / 2.0
    return float(
        np.clip(
            stage_extension,
            parameters.stage_min_m,
            parameters.stage_max_m,
        )
    )


def apply_compliance_parameters(
    model: mujoco.MjModel,
    parameters: ComplianceParameters,
) -> None:
    """Apply tunable joint springs and dampers to a loaded model."""

    for side in ("left", "right"):
        joint_values = (
            (
                f"{side}_hip_slide_x",
                parameters.hip_x_stiffness_n_m,
                parameters.hip_x_damping_n_s_m,
            ),
            (
                f"{side}_hip_slide_z",
                parameters.hip_z_stiffness_n_m,
                parameters.hip_z_damping_n_s_m,
            ),
            (
                f"{side}_strut_extension",
                2.0 * parameters.strut_equivalent_stiffness_n_m,
                2.0 * parameters.strut_equivalent_damping_n_s_m,
            ),
            (
                f"{side}_strut_extension_stage2",
                2.0 * parameters.strut_equivalent_stiffness_n_m,
                2.0 * parameters.strut_equivalent_damping_n_s_m,
            ),
        )
        for joint_name, stiffness, damping in joint_values:
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            if joint_id < 0:
                raise ValueError(f"MuJoCo joint not found: {joint_name}")
            model.jnt_stiffness[joint_id] = stiffness
            model.dof_damping[int(model.jnt_dofadr[joint_id])] = damping


def apply_four_bar_hinge_damping(
    model: mujoco.MjModel,
    damping_n_m_s_rad: float,
) -> None:
    """Raise diamond-linkage hinge damping without touching the wheels."""

    for joint_name in FOUR_BAR_HINGE_JOINTS:
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        if joint_id < 0:
            raise ValueError(f"MuJoCo joint not found: {joint_name}")
        model.dof_damping[int(model.jnt_dofadr[joint_id])] = (
            damping_n_m_s_rad
        )


def apply_wheel_joint_damping(
    model: mujoco.MjModel,
    damping_n_m_s_rad: float,
) -> None:
    """Set symmetric wheel damping for an operating-point controller."""

    for joint_name in ("left_wheel_joint", "right_wheel_joint"):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        if joint_id < 0:
            raise ValueError(f"MuJoCo joint not found: {joint_name}")
        model.dof_damping[int(model.jnt_dofadr[joint_id])] = (
            damping_n_m_s_rad
        )


def strut_spring_compensation(
    parameters: ComplianceParameters,
    servo_stiffness_n_m: float = 8000.0,
) -> float:
    """Return command scale accounting for both synchronized stage springs."""

    return (
        1.0
        + 4.0
        * parameters.strut_equivalent_stiffness_n_m
        / servo_stiffness_n_m
    )


def compliance_parameters_for_speed(
    speed_m_s: float,
) -> ComplianceParameters:
    """Blend low-speed rough-terrain and high-speed damping settings."""

    low_speed = ComplianceParameters()
    blend = float(np.clip((abs(speed_m_s) - 3.0) / 5.0, 0.0, 1.0))
    return ComplianceParameters(
        hip_x_stiffness_n_m=low_speed.hip_x_stiffness_n_m,
        hip_x_damping_n_s_m=(
            low_speed.hip_x_damping_n_s_m
            + blend
            * (
                HIGH_SPEED_COMPLIANCE.hip_x_damping_n_s_m
                - low_speed.hip_x_damping_n_s_m
            )
        ),
        hip_z_stiffness_n_m=low_speed.hip_z_stiffness_n_m,
        hip_z_damping_n_s_m=(
            low_speed.hip_z_damping_n_s_m
            + blend
            * (
                HIGH_SPEED_COMPLIANCE.hip_z_damping_n_s_m
                - low_speed.hip_z_damping_n_s_m
            )
        ),
        strut_equivalent_stiffness_n_m=(
            low_speed.strut_equivalent_stiffness_n_m
        ),
        strut_equivalent_damping_n_s_m=(
            low_speed.strut_equivalent_damping_n_s_m
            + blend
            * (
                HIGH_SPEED_COMPLIANCE.strut_equivalent_damping_n_s_m
                - low_speed.strut_equivalent_damping_n_s_m
            )
        ),
    )


class JointTerrainController:
    """Balance and yaw torques plus previewed independent leg lengths."""

    def __init__(
        self,
        timestep: float,
        gains: TerrainControlGains | None = None,
        balance_gains: BalanceGains | None = None,
    ) -> None:
        self.timestep = timestep
        self.gains = gains or TerrainControlGains()
        self.balance = BalanceSpeedController(timestep, balance_gains)
        self.stage_targets = np.zeros(2)
        self.roll_integral = 0.0

    def reset(self) -> None:
        self.balance.reset()
        self.roll_integral = 0.0
        self.stage_targets[:] = 0.0

    def update(
        self,
        *,
        target_speed_m_s: float,
        forward_speed_m_s: float,
        trunk_height_m: float,
        vertical_speed_m_s: float,
        quaternion_wxyz: np.ndarray,
        angular_velocity_xyz: np.ndarray,
        preview_ground_heights_m: np.ndarray,
    ) -> JointControlOutput:
        gains = self.gains
        pitch = quaternion_pitch(quaternion_wxyz)
        roll, yaw = quaternion_roll_yaw(quaternion_wxyz)
        balance = self.balance.update(
            target_speed_m_s=target_speed_m_s,
            forward_speed_m_s=forward_speed_m_s,
            pitch_rad=pitch,
            pitch_rate_rad_s=float(angular_velocity_xyz[1]),
        )

        self.roll_integral = float(
            np.clip(
                self.roll_integral + roll * self.timestep,
                -gains.roll_integral_limit,
                gains.roll_integral_limit,
            )
        )
        differential_torque = float(
            np.clip(
                gains.yaw_kp * yaw
                + gains.yaw_kd * float(angular_velocity_xyz[2])
                + gains.roll_torque_kp * roll
                + gains.roll_torque_kd * float(angular_velocity_xyz[0])
                + gains.roll_torque_ki * self.roll_integral,
                -gains.differential_torque_limit_nm,
                gains.differential_torque_limit_nm,
            )
        )
        torque_limit = self.balance.gains.wheel_torque_limit_nm
        common_torque_limit = max(
            torque_limit - abs(differential_torque),
            0.0,
        )
        common_torque = float(
            np.clip(
                balance.wheel_torque_nm,
                -common_torque_limit,
                common_torque_limit,
            )
        )
        wheel_torques = np.clip(
            (
                common_torque + differential_torque,
                common_torque - differential_torque,
            ),
            -torque_limit,
            torque_limit,
        )

        height_correction = (
            gains.height_kp * (gains.target_trunk_height_m - trunk_height_m)
            - gains.height_kd * vertical_speed_m_s
        )
        roll_correction = (
            gains.roll_kp * roll
            + gains.roll_kd * float(angular_velocity_xyz[0])
        )
        mean_preview = float(np.mean(preview_ground_heights_m))
        blended_preview = (
            (1.0 - gains.preview_symmetry) * preview_ground_heights_m
            + gains.preview_symmetry * mean_preview
        )
        nominal_hip_height = gains.target_trunk_height_m + gains.hip_offset_z_m
        leg_heights = np.array(
            (
                nominal_hip_height
                - (blended_preview[0] + gains.wheel_radius_m)
                + height_correction
                - roll_correction,
                nominal_hip_height
                - (blended_preview[1] + gains.wheel_radius_m)
                + height_correction
                + roll_correction,
            )
        )
        desired_stages = np.array(
            [stage_extension_from_leg_height(value, gains) for value in leg_heights]
        )
        maximum_step = gains.stage_rate_limit_m_s * self.timestep
        self.stage_targets += np.clip(
            desired_stages - self.stage_targets,
            -maximum_step,
            maximum_step,
        )
        strut_controls = self.stage_targets * gains.strut_spring_compensation
        return JointControlOutput(
            wheel_torques_nm=np.asarray(wheel_torques),
            strut_controls_m=strut_controls,
            stage_targets_m=self.stage_targets.copy(),
            pitch_reference_rad=balance.pitch_reference_rad,
        )


def sample_preview_ground_heights(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    wheel_body_ids: tuple[int, int],
    forward_speed_m_s: float,
    gains: TerrainControlGains | None = None,
) -> np.ndarray:
    """Raycast only against terrain group 3 ahead of both wheels."""

    parameters = gains or TerrainControlGains()
    preview_distance = (
        parameters.preview_base_m
        + max(forward_speed_m_s, 0.0) * parameters.preview_time_s
    )
    geom_group = np.zeros(6, dtype=np.uint8)
    geom_group[3] = 1
    heights = np.zeros(2)
    ray_direction = np.array((0.0, 0.0, -1.0))
    ray_origin_z = max(float(data.qpos[2]) + 0.7, 1.2)

    for index, body_id in enumerate(wheel_body_ids):
        origin = np.array(
            (
                float(data.xpos[body_id, 0]) + preview_distance,
                float(data.xpos[body_id, 1]),
                ray_origin_z,
            )
        )
        distance = mujoco.mj_ray(
            model,
            data,
            origin,
            ray_direction,
            geom_group,
            True,
            -1,
            None,
        )
        if distance < 0.0:
            heights[index] = 0.0
            continue
        heights[index] = ray_origin_z - distance
    return heights


def medium_terrain_speed_profile(time_s: float) -> float:
    if time_s < 1.0:
        return 0.0
    if time_s < 3.0:
        return 0.75 * (time_s - 1.0)
    if time_s < 10.0:
        return 1.5
    if time_s < 12.0:
        return 0.75 * (12.0 - time_s)
    return 0.0

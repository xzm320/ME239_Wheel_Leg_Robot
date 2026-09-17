"""Joint wheel/attitude/leg-height controller for rough terrain."""

from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np

if __package__:
    from scripts.balance_controller import BalanceSpeedController, quaternion_pitch
else:
    from balance_controller import BalanceSpeedController, quaternion_pitch


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
    strut_spring_compensation: float = 1.75
    height_kp: float = 0.40
    height_kd: float = 0.10
    roll_kp: float = 0.290419
    roll_kd: float = 0.015922
    yaw_kp: float = 8.0
    yaw_kd: float = 1.0
    differential_torque_limit_nm: float = 2.5
    preview_base_m: float = 0.080
    preview_time_s: float = 0.030


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


class JointTerrainController:
    """Balance and yaw torques plus previewed independent leg lengths."""

    def __init__(
        self,
        timestep: float,
        gains: TerrainControlGains | None = None,
    ) -> None:
        self.timestep = timestep
        self.gains = gains or TerrainControlGains()
        self.balance = BalanceSpeedController(timestep)
        self.stage_targets = np.zeros(2)

    def reset(self) -> None:
        self.balance.reset()
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

        differential_torque = float(
            np.clip(
                gains.yaw_kp * yaw
                + gains.yaw_kd * float(angular_velocity_xyz[2]),
                -gains.differential_torque_limit_nm,
                gains.differential_torque_limit_nm,
            )
        )
        wheel_torques = np.clip(
            (
                balance.wheel_torque_nm + differential_torque,
                balance.wheel_torque_nm - differential_torque,
            ),
            -self.balance.gains.wheel_torque_limit_nm,
            self.balance.gains.wheel_torque_limit_nm,
        )

        height_correction = (
            gains.height_kp * (gains.target_trunk_height_m - trunk_height_m)
            - gains.height_kd * vertical_speed_m_s
        )
        roll_correction = (
            gains.roll_kp * roll
            + gains.roll_kd * float(angular_velocity_xyz[0])
        )
        nominal_hip_height = gains.target_trunk_height_m + gains.hip_offset_z_m
        leg_heights = np.array(
            (
                nominal_hip_height
                - (preview_ground_heights_m[0] + gains.wheel_radius_m)
                + height_correction
                - roll_correction,
                nominal_hip_height
                - (preview_ground_heights_m[1] + gains.wheel_radius_m)
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
            raise RuntimeError("terrain preview ray did not hit group 3")
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

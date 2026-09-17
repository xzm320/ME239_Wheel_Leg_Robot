"""Cascaded speed PI and body-pitch PD controller."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BalanceGains:
    """Cascaded speed PI + pitch PD. Defaults are the 2 m/s flat-ground set."""

    pitch_kp: float = 465.6879
    pitch_kd: float = 11.8895
    speed_kp: float = 0.189834
    speed_ki: float = 0.008800
    pitch_reference_limit_rad: float = 0.18
    pitch_reference_rate_rad_s: float = 0.55173
    speed_integral_limit: float = 2.0
    wheel_torque_limit_nm: float = 6.0


@dataclass(frozen=True)
class HeadingGains:
    """Differential-wheel lane hold. Signs assume left = +torque, right = -torque."""

    yaw_kp: float = 0.58
    yaw_kd: float = 0.17
    lateral_kp: float = 0.025
    lateral_kd: float = 0.02
    roll_kp: float = 3.4
    roll_kd: float = 0.50
    torque_limit_nm: float = 0.22
    engage_speed_m_s: float = 8.0
    blend_speed_m_s: float = 12.0


def heading_torque_nm(gains: HeadingGains, **state: float) -> float:
    speed = state["forward_speed_m_s"]
    if speed < gains.engage_speed_m_s:
        return 0.0
    scale = float(
        np.clip(
            (speed - gains.engage_speed_m_s) / max(gains.blend_speed_m_s, 1e-6),
            0.0,
            1.0,
        )
    )
    torque = (
        gains.yaw_kp * state["yaw_rad"]
        + gains.yaw_kd * state["yaw_rate_rad_s"]
        + gains.lateral_kp * state["lateral_m"]
        + gains.lateral_kd * state["lateral_speed_m_s"]
        - gains.roll_kp * state["roll_rad"]
        - gains.roll_kd * state["roll_rate_rad_s"]
    )
    return float(np.clip(scale * torque, -gains.torque_limit_nm, gains.torque_limit_nm))


def wide_car_balance_gains() -> BalanceGains:
    """3x-track pad cruise. Holds 195 km/h; 196 km/h rolls over."""

    return BalanceGains(
        pitch_kp=480.0,
        pitch_kd=70.0,
        speed_kp=0.022,
        speed_ki=0.0,
        pitch_reference_limit_rad=0.070,
        pitch_reference_rate_rad_s=0.12,
        wheel_torque_limit_nm=6.0,
    )


def wide_car_heading_gains() -> HeadingGains:
    return HeadingGains(
        yaw_kp=0.45,
        yaw_kd=0.20,
        lateral_kp=0.04,
        lateral_kd=0.05,
        roll_kp=5.0,
        roll_kd=0.70,
        torque_limit_nm=0.18,
        engage_speed_m_s=8.0,
        blend_speed_m_s=12.0,
    )


def prototype_balance_gains() -> BalanceGains:
    """1x four-bar prototype. Holds 180 km/h from rest; 182 km/h has a heading hole."""

    return BalanceGains(
        pitch_kp=540.0,
        pitch_kd=48.0,
        speed_kp=0.024,
        speed_ki=0.0,
        pitch_reference_limit_rad=0.085,
        pitch_reference_rate_rad_s=0.12,
        wheel_torque_limit_nm=6.0,
    )


def prototype_heading_gains() -> HeadingGains:
    """Narrow 227 mm track: earlier engage, stronger roll, tiny differentials."""

    return HeadingGains(
        yaw_kp=0.18,
        yaw_kd=0.10,
        lateral_kp=0.05,
        lateral_kd=0.04,
        roll_kp=7.2,
        roll_kd=1.05,
        torque_limit_nm=0.10,
        engage_speed_m_s=2.5,
        blend_speed_m_s=5.0,
    )


@dataclass(frozen=True)
class ControlOutput:
    wheel_torque_nm: float
    pitch_reference_rad: float
    speed_integral: float


class BalanceSpeedController:
    """Outer speed PI feeding a rate-limited inner pitch PD loop."""

    def __init__(self, timestep: float, gains: BalanceGains | None = None) -> None:
        self.timestep = timestep
        self.gains = gains or BalanceGains()
        self.reset()

    def reset(self) -> None:
        self.speed_integral = 0.0
        self.pitch_reference = 0.0

    def update(
        self,
        *,
        target_speed_m_s: float,
        forward_speed_m_s: float,
        pitch_rad: float,
        pitch_rate_rad_s: float,
    ) -> ControlOutput:
        gains = self.gains
        speed_error = target_speed_m_s - forward_speed_m_s
        self.speed_integral = float(
            np.clip(
                self.speed_integral + speed_error * self.timestep,
                -gains.speed_integral_limit,
                gains.speed_integral_limit,
            )
        )

        raw_pitch_reference = float(
            np.clip(
                gains.speed_kp * speed_error
                + gains.speed_ki * self.speed_integral,
                -gains.pitch_reference_limit_rad,
                gains.pitch_reference_limit_rad,
            )
        )
        maximum_reference_step = gains.pitch_reference_rate_rad_s * self.timestep
        self.pitch_reference += float(
            np.clip(
                raw_pitch_reference - self.pitch_reference,
                -maximum_reference_step,
                maximum_reference_step,
            )
        )

        torque = (
            gains.pitch_kp * (pitch_rad - self.pitch_reference)
            + gains.pitch_kd * pitch_rate_rad_s
        )
        torque = float(
            np.clip(
                torque,
                -gains.wheel_torque_limit_nm,
                gains.wheel_torque_limit_nm,
            )
        )
        return ControlOutput(torque, self.pitch_reference, self.speed_integral)


def quaternion_pitch(quaternion_wxyz: np.ndarray) -> float:
    """Return world-frame pitch for a MuJoCo w-x-y-z quaternion."""

    w, x, y, z = quaternion_wxyz
    sine = 2.0 * (w * y - z * x)
    return math.asin(float(np.clip(sine, -1.0, 1.0)))


def flat_speed_profile(time_s: float) -> float:
    """Accelerate to 2 m/s, cruise, brake, then settle."""

    if time_s < 1.0:
        return 0.0
    if time_s < 3.0:
        return time_s - 1.0
    if time_s < 8.0:
        return 2.0
    if time_s < 10.0:
        return 10.0 - time_s
    return 0.0

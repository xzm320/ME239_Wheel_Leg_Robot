"""Cascaded speed PI and body-pitch PD controller."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BalanceGains:
    """Flat-ground gains found by deterministic batch simulation."""

    pitch_kp: float = 465.6879
    pitch_kd: float = 11.8895
    speed_kp: float = 0.189834
    speed_ki: float = 0.008800
    pitch_reference_limit_rad: float = 0.18
    pitch_reference_rate_rad_s: float = 0.55173
    speed_integral_limit: float = 2.0
    wheel_torque_limit_nm: float = 6.0


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

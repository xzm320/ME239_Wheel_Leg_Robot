"""Cascaded speed PI + body-pitch PD, with a small heading/lane loop.

Wheel-legged balance is an inverted pendulum: the outer loop asks for a
lean (pitch reference) from the speed error, the inner loop turns that
lean into a common wheel torque. A second, weaker loop adds a
differential heading torque so the robot stays in the lane. Both robots
use the same common-mode wheel mapping (both wheel axes are +Y).

The pitch convention matches MuJoCo's w-x-y-z quaternion: positive pitch
is nose-down (rotation about +Y).
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BalanceGains:
    """Outer speed PI feeding a rate-limited inner pitch PD."""

    pitch_kp: float
    pitch_kd: float
    speed_kp: float
    speed_ki: float
    pitch_reference_limit_rad: float = 0.14
    pitch_reference_rate_rad_s: float = 0.45
    speed_integral_limit: float = 2.5
    wheel_torque_limit_nm: float = 1.7


@dataclass(frozen=True)
class HeadingGains:
    """Keep yaw, lateral offset, and roll from walking off the strip."""

    yaw_kp: float
    yaw_kd: float
    lateral_kp: float
    lateral_kd: float
    roll_kp: float
    roll_kd: float
    torque_limit_nm: float = 0.55


@dataclass(frozen=True)
class ControlOutput:
    wheel_torque_nm: float
    pitch_reference_rad: float
    speed_integral: float


def prototype_balance_gains() -> BalanceGains:
    """Rigid 2-link ablation. Gains match ``wide_car`` (same motors)."""

    return wide_car_balance_gains()


def prototype_heading_gains() -> HeadingGains:
    """Lane loop matched to ``wide_car`` for the ablation."""

    return wide_car_heading_gains()


def wide_car_balance_gains() -> BalanceGains:
    """3× four-bar, PID on the wheels only. Hips/struts stay at nominal."""

    return BalanceGains(
        pitch_kp=420.0,
        pitch_kd=95.0,
        speed_kp=0.075,
        speed_ki=0.004,
        pitch_reference_limit_rad=0.12,
        pitch_reference_rate_rad_s=0.28,
        wheel_torque_limit_nm=6.0,
    )


def wide_car_heading_gains() -> HeadingGains:
    return HeadingGains(
        yaw_kp=0.70,
        yaw_kd=0.28,
        lateral_kp=0.22,
        lateral_kd=0.12,
        roll_kp=2.2,
        roll_kd=0.45,
        torque_limit_nm=0.45,
    )


class BalanceSpeedController:
    """Outer speed PI feeding a rate-limited inner pitch PD loop."""

    def __init__(self, timestep: float, gains: BalanceGains, initial_pitch_reference: float = 0.0) -> None:
        self.timestep = timestep
        self.gains = gains
        self.initial_pitch_reference = initial_pitch_reference
        self.reset()

    def reset(self) -> None:
        self.speed_integral = 0.0
        self.pitch_reference = self.initial_pitch_reference

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
        raw_reference = float(
            np.clip(
                gains.speed_kp * speed_error + gains.speed_ki * self.speed_integral,
                -gains.pitch_reference_limit_rad,
                gains.pitch_reference_limit_rad,
            )
        )
        maximum_step = gains.pitch_reference_rate_rad_s * self.timestep
        self.pitch_reference += float(
            np.clip(raw_reference - self.pitch_reference, -maximum_step, maximum_step)
        )
        torque = float(
            np.clip(
                gains.pitch_kp * (pitch_rad - self.pitch_reference)
                + gains.pitch_kd * pitch_rate_rad_s,
                -gains.wheel_torque_limit_nm,
                gains.wheel_torque_limit_nm,
            )
        )
        return ControlOutput(torque, self.pitch_reference, self.speed_integral)


def heading_torque_nm(gains: HeadingGains, **state: float) -> float:
    torque = (
        gains.yaw_kp * state["yaw_rad"]
        + gains.yaw_kd * state["yaw_rate_rad_s"]
        + gains.lateral_kp * state["lateral_m"]
        + gains.lateral_kd * state["lateral_speed_m_s"]
        - gains.roll_kp * state["roll_rad"]
        - gains.roll_kd * state["roll_rate_rad_s"]
    )
    return float(np.clip(torque, -gains.torque_limit_nm, gains.torque_limit_nm))


def quaternion_pitch(quaternion_wxyz: np.ndarray) -> float:
    """World-frame pitch: positive is nose-down (rotation about +Y)."""

    w, x, y, z = quaternion_wxyz
    sine = 2.0 * (w * y - z * x)
    return math.asin(float(np.clip(sine, -1.0, 1.0)))


def quaternion_roll_yaw(quaternion_wxyz: np.ndarray) -> tuple[float, float]:
    """Return ``(roll, yaw)`` from a MuJoCo w-x-y-z quaternion."""

    w, x, y, z = quaternion_wxyz
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, yaw

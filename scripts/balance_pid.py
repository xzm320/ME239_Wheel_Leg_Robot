"""Cascaded speed PI + body-pitch PD, with a small heading/lane loop.

Wheel-legged balance is an inverted pendulum: the outer loop asks for a
lean (pitch reference) from the speed error, the inner loop turns that
lean into a common wheel torque. A second, weaker loop adds a
differential (prototype) or common-mode (wide_car) heading torque so the
robot stays in the lane.

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
    """Official Upkie on the Unitree Perlin strip.

    Hip/knee are held at the URDF zero pose by a stiff position servo
    (see ``apply_prototype_pose_hold``). Only the wheels run this PID.
    """

    return BalanceGains(
        pitch_kp=18.0,
        pitch_kd=3.8,
        speed_kp=0.18,
        speed_ki=0.04,
        pitch_reference_limit_rad=0.14,
        pitch_reference_rate_rad_s=0.45,
        wheel_torque_limit_nm=1.7,
    )


def prototype_heading_gains() -> HeadingGains:
    return HeadingGains(
        yaw_kp=0.60,
        yaw_kd=0.18,
        lateral_kp=3.0,
        lateral_kd=0.80,
        roll_kp=2.2,
        roll_kd=0.40,
        torque_limit_nm=0.55,
    )


def wide_car_balance_gains() -> BalanceGains:
    """3× four-bar, PID on the wheels only. Hips/struts stay at nominal."""

    return BalanceGains(
        pitch_kp=380.0,
        pitch_kd=90.0,
        speed_kp=0.055,
        speed_ki=0.0,
        pitch_reference_limit_rad=0.09,
        pitch_reference_rate_rad_s=0.18,
        wheel_torque_limit_nm=6.0,
    )


def wide_car_heading_gains() -> HeadingGains:
    return HeadingGains(
        yaw_kp=0.55,
        yaw_kd=0.22,
        lateral_kp=0.12,
        lateral_kd=0.08,
        roll_kp=1.6,
        roll_kd=0.35,
        torque_limit_nm=0.32,
    )


class BalanceSpeedController:
    """Outer speed PI feeding a rate-limited inner pitch PD loop."""

    def __init__(self, timestep: float, gains: BalanceGains) -> None:
        self.timestep = timestep
        self.gains = gains
        self.reset()

    def reset(self) -> None:
        self.speed_integral = 0.0
        # Small forward lean offsets the prototype COM, which sits ~6 mm aft
        # of the wheel contact at the URDF zero pose.
        self.pitch_reference = 0.02

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

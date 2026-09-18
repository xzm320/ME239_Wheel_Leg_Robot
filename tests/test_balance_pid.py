import math

import numpy as np

from scripts.balance_pid import (
    BalanceSpeedController,
    prototype_balance_gains,
    quaternion_pitch,
    quaternion_roll_yaw,
)


def test_quaternion_conventions() -> None:
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    assert abs(quaternion_pitch(identity)) < 1e-9
    roll, yaw = quaternion_roll_yaw(identity)
    assert abs(roll) < 1e-9 and abs(yaw) < 1e-9

    pitch_angle = math.radians(8.0)
    half = pitch_angle / 2.0
    pitched = np.array([math.cos(half), 0.0, math.sin(half), 0.0])
    assert abs(quaternion_pitch(pitched) - pitch_angle) < 1e-6


def test_pitch_pd_recovers_nose_down() -> None:
    controller = BalanceSpeedController(0.001, prototype_balance_gains())
    output = controller.update(
        target_speed_m_s=0.0,
        forward_speed_m_s=0.0,
        pitch_rad=0.10,
        pitch_rate_rad_s=0.0,
    )
    assert output.wheel_torque_nm > 0.0

from scripts.generate_perlin_track import LENGTH_M, PERLIN_OCTAVES, PERLIN_SMOOTH_M
from scripts.verify_perlin_speed import verify


def test_perlin_strip_is_unitree_scale() -> None:
    assert LENGTH_M == 48.0
    assert abs(PERLIN_SMOOTH_M - 1.5625) < 1e-9
    assert PERLIN_OCTAVES == 6


def test_wide_car_and_upkie_perlin_limits() -> None:
    result = verify()

    assert result["upkie_model"]["nq"] == 13
    assert result["upkie_model"]["mass_kg"] == 5.4605

    assert result["wide"]["limit_m_s"] == 3.0
    assert result["wide"]["hold"]["held"]
    assert result["wide"]["hold"]["peak_kmh"] > 9.5
    assert result["wide"]["fail"]["failure"] is not None
    assert result["wide"]["pid"]["balance"]["pitch_kd"] == 90.0
    assert result["wide"]["pid"]["heading"]["yaw_kp"] == 0.55

    assert result["upkie"]["stand"]["held"]
    assert result["upkie"]["stand"]["pitch_deg"] < 5.0
    assert result["upkie"]["move_fail"]["failure"] is not None
    assert result["upkie"]["pid"]["balance"]["wheel_torque_limit_nm"] == 1.7

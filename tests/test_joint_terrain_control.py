from scripts.verify_joint_terrain_control import verify


def test_joint_control_on_medium_terrain() -> None:
    result = verify()

    assert result["wheel_diameter_mm"] == 240.0
    assert result["measured_cruise_speed_m_s"] > 1.45
    assert result["com_height_standard_deviation_mm"] < 12.0
    assert result["maximum_com_height_deviation_mm"] < 45.0
    assert result["maximum_roll_deg"] < 5.0
    assert result["maximum_pitch_deg"] < 15.0
    assert result["vertical_acceleration_rms_m_s2"] < 3.5
    assert result["vertical_acceleration_p95_m_s2"] < 4.0

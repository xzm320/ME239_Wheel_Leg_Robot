from scripts.verify_flat_control import verify


def test_flat_ground_balance_and_speed_control() -> None:
    result = verify()

    assert result["target_cruise_speed_m_s"] == 2.0
    assert result["cruise_speed_rmse_m_s"] < 0.16
    assert result["maximum_abs_pitch_deg"] < 8.0
    assert abs(result["final_speed_m_s"]) < 0.03
    assert result["recovered_initial_pitch_deg"] == 8.0

from scripts.verify_speed_limit import verify


def test_wide_car_and_prototype_speed_limits() -> None:
    result = verify()

    assert result["wide_car"]["limit_kmh"] == 195.0
    assert result["wide_car"]["cruise_190"]["held"]
    assert result["wide_car"]["cruise_195"]["held"]
    assert result["wide_car"]["cruise_195"]["peak_kmh"] > 193.0
    assert result["wide_car"]["pid"]["balance"]["pitch_kd"] == 70.0
    assert result["wide_car"]["pid"]["heading"]["roll_kp"] == 5.0

    assert result["prototype"]["limit_kmh"] == 180.0
    assert result["prototype"]["cruise_170"]["held"]
    assert result["prototype"]["cruise_180"]["held"]
    assert result["prototype"]["cruise_180"]["peak_kmh"] > 178.0
    assert result["prototype"]["pid"]["balance"]["pitch_kp"] == 540.0
    assert result["prototype"]["pid"]["heading"]["torque_limit_nm"] == 0.10

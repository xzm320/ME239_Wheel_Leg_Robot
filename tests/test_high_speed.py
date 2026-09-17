from scripts.verify_high_speed import verify


def test_high_speed_envelope_and_100kmh_attempt() -> None:
    result = verify()

    assert result["selected_robust_speed_km_h"] == 36.0
    assert result["selected_result"]["stable"]
    assert result["requested_100_km_h_peak_km_h"] > 99.0
    assert result["entered_rough_before_failure"]
    assert result["requested_100_km_h"]["peak_speed_m_s"] > 27.5
    assert result["wheel_track_scale"] == 4.0
    assert result["chassis_half_width_mm"] >= 420.0
    assert result["requested_100_km_h"]["maximum_pitch_deg"] < 24.0

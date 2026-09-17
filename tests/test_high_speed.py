from scripts.verify_high_speed import verify


def test_high_speed_envelope_and_100kmh_attempt() -> None:
    result = verify()

    assert result["selected_robust_speed_km_h"] == 36.0
    assert result["selected_result"]["stable"]
    assert result["requested_100_km_h_peak_km_h"] > 99.0
    assert result["entered_rough_before_failure"]
    assert result["requested_100_km_h"]["peak_speed_m_s"] > 27.5
    assert result["hundred_kmh_stable"]
    assert result["wheel_track_scale"] == 3.0
    assert 310.0 <= result["chassis_half_width_mm"] <= 340.0
    assert result["requested_100_km_h"]["maximum_pitch_deg"] < 25.0

from scripts.verify_high_speed import verify


def test_high_speed_envelope_and_fallback() -> None:
    result = verify()

    assert not result["requested_80_km_h"]["stable"]
    assert result["selected_robust_speed_km_h"] == 36.0
    assert result["verified_neighborhood_km_h"] == [34.92, 36.0, 37.08]
    assert result["selected_result"]["stable"]
    assert result["selected_result"]["com_height_std_mm"] < 3.0

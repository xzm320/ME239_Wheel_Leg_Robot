from scripts.verify_terrains import verify


def test_graded_rough_terrains() -> None:
    result = verify()

    assert result["terrain_count"] == 3
    assert result["levels"]["easy"]["rms_height_mm"] < result["levels"]["medium"]["rms_height_mm"]
    assert result["levels"]["medium"]["rms_height_mm"] < result["levels"]["hard"]["rms_height_mm"]
    assert result["levels"]["hard"]["maximum_slope_deg"] < 25.0

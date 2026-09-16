from scripts.verify_terrains import verify


def test_graded_rough_terrains() -> None:
    result = verify()

    assert result["terrain_count"] == 4
    rms_heights = [
        result["levels"][level]["rms_height_mm"]
        for level in ("easy", "medium", "hard", "extreme")
    ]
    assert rms_heights == sorted(rms_heights)
    assert result["levels"]["hard"]["obstacle_count"] == 14
    assert result["levels"]["extreme"]["obstacle_count"] == 20

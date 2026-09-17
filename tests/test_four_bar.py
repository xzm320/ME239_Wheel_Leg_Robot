from scripts.verify_four_bar import verify


def test_active_elastic_four_bar() -> None:
    result = verify()

    assert result["equality_constraints"] == 6
    assert result["measured_total_extension_mm"] > 187.9
    assert result["minimum_height_ratio"] < 0.52
    assert result["minimum_rod_insertion_mm"] > 25.0
    assert result["passive_return_extension_mm"] == 0.0
    assert result["passive_force_at_20_mm_n"] == -30.0

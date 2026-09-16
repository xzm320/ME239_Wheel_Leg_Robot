from scripts.verify_four_bar import verify


def test_active_elastic_four_bar() -> None:
    result = verify()

    assert result["equality_constraints"] == 4
    assert result["measured_extension_mm"] == 40.0
    assert result["passive_return_extension_mm"] == 0.0
    assert result["passive_force_at_20_mm_n"] == -30.0

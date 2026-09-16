from scripts.verify_upkie_baseline import verify


def test_pinned_upkie_model() -> None:
    result = verify()

    assert result["source"] == "MarcDcls/mjlab_upkie@d7895789"
    assert result["total_mass_kg"] == 5.4605

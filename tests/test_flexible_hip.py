from scripts.verify_flexible_hip import verify


def test_compliant_hip_restoring_forces() -> None:
    result = verify()

    assert result["passive_slide_count"] == 4
    assert result["nu"] == 6
    assert result["forces"]["left_hip_slide_x"]["at_positive_10_mm_n"] == -65.0
    assert result["forces"]["left_hip_slide_z"]["at_positive_10_mm_n"] == -120.0

from scripts.four_bar_kinematics import (
    NOMINAL_LINK_LENGTH_M,
    available_compression_m,
    lengthening_report,
)


def test_current_links_already_have_whoop_scale_stroke() -> None:
    stroke = available_compression_m(NOMINAL_LINK_LENGTH_M)
    assert stroke > 0.15


def test_lengthening_without_longer_strut_reduces_stroke() -> None:
    report = lengthening_report(0.230)
    assert (
        report["candidate_compression_stroke_m"]
        < report["current_compression_stroke_m"]
    )
    assert report["candidate_standing_stage_m"] > 0.05

import mujoco

from scripts.high_speed_evaluation import MODEL_PATH
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


def test_unitree_perlin_patch_matches_demo_size() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    assert model.nhfield == 1
    radius_x, radius_y, elevation, base = model.hfield_size[0]
    assert abs(float(radius_x) - 1.0) < 1e-6
    assert abs(float(radius_y) - 0.75) < 1e-6
    assert abs(float(elevation) - 0.20) < 1e-6
    assert abs(float(base) - 0.20) < 1e-6
    assert int(model.hfield_nrow[0]) >= 128
    assert int(model.hfield_ncol[0]) >= 128

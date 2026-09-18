from scripts.verify_models import verify


def test_prototype_and_wide_car_load_on_perlin() -> None:
    result = verify()

    assert result["prototype"]["nq"] == 13
    assert result["prototype"]["nu"] == 6
    assert result["prototype"]["neq"] == 0
    assert result["prototype"]["mass_kg"] == 13.168
    assert result["prototype"]["hip_y_m"] == 0.3411
    assert result["prototype"]["stand_z_m"] == 0.408
    assert result["prototype"]["nhfield"] == 1
    assert "ablation" in result["prototype"]["source"]
    assert "upkie" not in result["prototype"]["source"].lower()

    assert result["wide_car"]["nhfield"] == 1
    assert result["wide_car"]["neq"] >= 6
    assert result["wide_car"]["mass_kg"] == 19.374
    assert result["wide_car"]["mass_kg"] - result["prototype"]["mass_kg"] > 5.0
    assert result["wide_car"]["hip_y_m"] == 0.3411
    assert result["wide_car"]["stand_z_m"] == 0.408

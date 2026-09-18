from scripts.verify_models import verify


def test_prototype_and_wide_car_load_on_perlin() -> None:
    result = verify()

    assert result["prototype"]["nq"] == 13
    assert result["prototype"]["nu"] == 6
    assert result["prototype"]["mass_kg"] == 5.3392
    assert result["prototype"]["nhfield"] == 1
    assert result["prototype"]["source"] == "https://github.com/upkie/upkie"

    assert result["wide_car"]["nhfield"] == 1
    assert result["wide_car"]["neq"] >= 6
    assert result["wide_car"]["hip_y_m"] == 0.3411

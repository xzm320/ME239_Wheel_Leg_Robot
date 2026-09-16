from scripts.verify_mujoco import verify


def test_physics_and_headless_rendering() -> None:
    result = verify()

    assert result["mujoco_version"] == "3.13.0"
    assert result["backend"] == "osmesa"

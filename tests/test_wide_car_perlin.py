from scripts.wide_car_perlin import run_wide_car_episode


def test_wide_car_pid_can_stand_on_flat_strip() -> None:
    result = run_wide_car_episode(0.0, duration_s=1.5, start_x_m=3.5)
    assert result.stable
    assert result.failure_reason is None
    assert result.maximum_pitch_deg < 10.0
    assert result.maximum_roll_deg < 8.0
    assert abs(result.final_y_m) < 0.20

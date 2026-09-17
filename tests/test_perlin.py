import numpy as np

from scripts.perlin import pnoise2_fbm, unitree_perlin_image, unitree_perlin_meters


def test_unitree_perlin_defaults_are_bounded() -> None:
    x = np.linspace(0.0, 4.0, 32)
    y = np.linspace(0.0, 3.0, 24)
    grid_y, grid_x = np.meshgrid(y, x, indexing="ij")
    field = pnoise2_fbm(grid_x, grid_y, seed=7)
    assert float(np.max(np.abs(field))) <= 2.1


def test_unitree_image_encoding_is_0_to_1() -> None:
    image = unitree_perlin_image(64, 48, smooth=20.0, seed=11)
    assert image.shape == (48, 64)
    assert 0.0 <= float(np.min(image)) <= float(np.max(image)) <= 1.0


def test_meter_space_field_is_not_extruded() -> None:
    x = np.linspace(-8.0, 8.0, 80)
    y = np.linspace(-4.0, 4.0, 40)
    grid_y, grid_x = np.meshgrid(y, x, indexing="ij")
    field = unitree_perlin_meters(grid_x, grid_y, smooth_m=2.0, seed=21)
    left = field[:, 20]
    right = field[:, 60]
    assert float(np.mean(np.abs(left - right))) > 0.02

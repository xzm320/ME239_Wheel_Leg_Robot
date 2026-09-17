"""Unitree-style 2D Perlin / fBm heightfields.

Matches ``AddPerlinHeighField`` in unitree_mujoco/terrain_tool/terrain_generator.py:
``noise.pnoise2(x / smooth, y / smooth, octaves, persistence, lacunarity)``
mapped with ``(value + 1) / 2 * 255``.
"""

from __future__ import annotations

import numpy as np

_GRADIENTS = np.array(
    (
        (1.0, 1.0),
        (1.0, -1.0),
        (-1.0, 1.0),
        (-1.0, -1.0),
        (1.0, 0.0),
        (-1.0, 0.0),
        (0.0, 1.0),
        (0.0, -1.0),
    ),
    dtype=np.float64,
)


def permutation_table(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    table = np.arange(256, dtype=np.int32)
    rng.shuffle(table)
    return np.concatenate((table, table))


def _fade(t: np.ndarray) -> np.ndarray:
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def _perlin2d(x: np.ndarray, y: np.ndarray, permutation: np.ndarray) -> np.ndarray:
    x0 = np.floor(x).astype(np.int32)
    y0 = np.floor(y).astype(np.int32)
    xf = x - x0
    yf = y - y0
    x1 = x0 + 1
    y1 = y0 + 1

    aa = permutation[permutation[x0 & 255] + (y0 & 255)]
    ab = permutation[permutation[x0 & 255] + (y1 & 255)]
    ba = permutation[permutation[x1 & 255] + (y0 & 255)]
    bb = permutation[permutation[x1 & 255] + (y1 & 255)]

    def dot(hash_values: np.ndarray, dx: np.ndarray, dy: np.ndarray) -> np.ndarray:
        gradient = _GRADIENTS[hash_values & 7]
        return gradient[..., 0] * dx + gradient[..., 1] * dy

    u = _fade(xf)
    v = _fade(yf)
    x_interp_1 = dot(aa, xf, yf) + u * (dot(ba, xf - 1.0, yf) - dot(aa, xf, yf))
    x_interp_2 = dot(ab, xf, yf - 1.0) + u * (
        dot(bb, xf - 1.0, yf - 1.0) - dot(ab, xf, yf - 1.0)
    )
    return x_interp_1 + v * (x_interp_2 - x_interp_1)


def pnoise2_fbm(
    x: np.ndarray,
    y: np.ndarray,
    *,
    octaves: int = 6,
    persistence: float = 0.5,
    lacunarity: float = 2.0,
    permutation: np.ndarray | None = None,
    seed: int = 0,
) -> np.ndarray:
    """Octaved Perlin, Unitree ``noise.pnoise2`` defaults (unnormalized sum)."""

    table = permutation if permutation is not None else permutation_table(seed)
    total = np.zeros(np.broadcast(x, y).shape, dtype=np.float64)
    amplitude = 1.0
    frequency = 1.0
    for _ in range(max(1, octaves)):
        total += amplitude * _perlin2d(x * frequency, y * frequency, table)
        amplitude *= persistence
        frequency *= lacunarity
    return total


def unitree_perlin_image(
    width_px: int,
    height_px: int,
    *,
    smooth: float = 100.0,
    octaves: int = 6,
    persistence: float = 0.5,
    lacunarity: float = 2.0,
    seed: int = 0,
) -> np.ndarray:
    """Reproduce Unitree's pixel-space ``pnoise2(x / smooth, y / smooth)``."""

    column = np.arange(width_px, dtype=np.float64)
    row = np.arange(height_px, dtype=np.float64)
    grid_y, grid_x = np.meshgrid(column, row, indexing="xy")
    # Unitree iterates y,x over image_width even when the image is rectangular.
    noise = pnoise2_fbm(
        grid_x / smooth,
        grid_y / smooth,
        octaves=octaves,
        persistence=persistence,
        lacunarity=lacunarity,
        seed=seed,
    )
    return np.clip((noise + 1.0) * 0.5, 0.0, 1.0)


def unitree_perlin_meters(
    x_m: np.ndarray,
    y_m: np.ndarray,
    *,
    smooth_m: float = 8.0,
    octaves: int = 6,
    persistence: float = 0.5,
    lacunarity: float = 2.0,
    seed: int = 0,
) -> np.ndarray:
    """Same fBm, but frequencies are in metres so a long track stays isotropic."""

    return pnoise2_fbm(
        x_m / smooth_m,
        y_m / smooth_m,
        octaves=octaves,
        persistence=persistence,
        lacunarity=lacunarity,
        seed=seed,
    )

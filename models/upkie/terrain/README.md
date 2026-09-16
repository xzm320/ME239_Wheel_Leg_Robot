# Graded rough-terrain suite

The generator at `scripts/generate_terrains.py` creates four deterministic
20 m × 4 m heightfield roads at 25 mm grid resolution.

| Level | Heightfield peak | Correlation | Explicit obstacles | Max obstacle | RMS | Max slope |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Easy | ±20 mm | 450 mm | 4 | 31.5 mm | 8.11 mm | 2.61° |
| Medium | ±55 mm | 280 mm | 8 | 74.3 mm | 18.98 mm | 10.33° |
| Hard | ±100 mm | 180 mm | 14 | 129.6 mm | 37.61 mm | 27.39° |
| Extreme | ±180 mm | 140 mm | 20 | 199.7 mm | 43.68 mm | 38.93° |

Each road contains:

- a flat launch zone through `x=-7.2 m`;
- a smooth transition into roughness by `x=-5.8 m`;
- correlated longitudinal and lateral roughness;
- deterministic positive and negative heightfield bumps;
- cross-lane cylindrical logs and randomly tilted rock boxes;
- a smooth braking-zone transition from `x=7.8 m`.

The mixed construction follows the approach exposed by Unitree's public
[`terrain_tool`](https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/terrain_tool):
combine heightfields with explicit boxes, cylinders, stairs, and randomized
rough-ground geometry. This project uses its own deterministic implementation
and dimensions scaled for the 55 mm Upkie wheels.

Run the generator again with:

```bash
uv run python scripts/generate_terrains.py
```

The PNG corners contain two reserved scale-calibration pixels outside the
driving corridor. They force MuJoCo's image normalization to preserve metric
height for all four levels. Scene z-offsets compensate grayscale 128 so the
flat launch surface is exactly `z=0`.

This module validates terrain generation and collision only. Speed commands,
balance control, and traversal success criteria belong to the controller
module.

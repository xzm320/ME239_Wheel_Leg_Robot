# Graded rough-terrain suite

The generator at `scripts/generate_terrains.py` creates three deterministic
20 m × 4 m heightfield roads at 25 mm grid resolution.

| Level | Design peak | Correlation length | Local bumps | Measured RMS | Max slope |
| --- | ---: | ---: | ---: | ---: | ---: |
| Easy | ±15 mm | 600 mm | 2 | 5.04 mm | 1.18° |
| Medium | ±40 mm | 360 mm | 6 | 13.57 mm | 4.86° |
| Hard | ±75 mm | 220 mm | 12 | 30.91 mm | 17.93° |

Each road contains:

- a flat launch zone through `x=-7.2 m`;
- a smooth transition into roughness by `x=-5.8 m`;
- correlated longitudinal and lateral roughness;
- deterministic positive and negative local bumps;
- a smooth braking-zone transition from `x=7.8 m`.

Run the generator again with:

```bash
uv run python scripts/generate_terrains.py
```

The PNG corners contain two reserved scale-calibration pixels outside the
driving corridor. They force MuJoCo's image normalization to preserve metric
height for all three levels. Scene z-offsets compensate grayscale 128 so the
flat launch surface is exactly `z=0`.

This module validates terrain generation and collision only. Speed commands,
balance control, and traversal success criteria belong to the controller
module.

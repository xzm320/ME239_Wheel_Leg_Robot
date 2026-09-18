# Ablation prototype

`wide_car` with the three comparison features removed. Chassis pack, hip
spacing, and 10-inch hub motors are shared. Masses are catalog parts
(`models/PHYSICS.md`); there is **no ballast**.

Removed relative to `wide_car`:

- both x–z motorized hip stages (2 × 1.80 kg)
- both telescopic actuators (2 × 1.60 kg)
- rear four-bar links (4 × 0.16 kg)

Added because a serial 2-link needs an actuated knee:

- qdd100 at each knee (2 × 0.507 kg)
- welded 6061 hip brackets (2 × 0.15 kg) instead of the stages

Net: prototype 13.168 kg, wide_car 19.374 kg.

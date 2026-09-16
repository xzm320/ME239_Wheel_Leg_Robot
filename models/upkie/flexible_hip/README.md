# X-Z compliant hip variant

This model derives from `../upstream/robot.xml` and adds one passive carrier
between the trunk and each original hip hinge. Each carrier has two orthogonal
prismatic joints:

| Coordinate | Range | Stiffness | Damping |
| --- | ---: | ---: | ---: |
| Forward `x` | ±20 mm | 6500 N/m | 70 N·s/m |
| Vertical `z` | ±25 mm | 12000 N/m | 120 N·s/m |

Both springs use zero displacement as their reference. Positive and negative
displacements therefore receive a force directed back toward the nominal hip
location. The slide joints are passive and have no actuator.

## Initial parameter rationale

The original links downstream of each hip weigh 1.4095 kg. Including the new
80 g carrier gives an effective first-pass moving mass of 1.4895 kg. The
undamped natural-frequency estimate
`f = sqrt(k / m) / (2 pi)` gives:

- `x`: 10.51 Hz, damping ratio 0.356
- `z`: 14.28 Hz, damping ratio 0.449

With the 5.6205 kg modified robot resting symmetrically, the estimated vertical
static deflection is 2.30 mm per side. Limit forces are 130 N in `x` and 300 N
in `z`, before the joint-limit constraint contributes.

These are engineering starting values, not optimized hardware parameters.
They will be swept after the four-bar linkage, actuator limits, terrain
spectrum, and target speed are present because each changes the effective
mass and impact bandwidth.

## Coordinate choice

Upkie uses `x` forward, `y` lateral (wheel axle), and `z` upward. The chosen
`x-z` compliance therefore acts in the sagittal wheel-motion plane. The
original hip hinge remains actuated and unchanged inside each carrier.

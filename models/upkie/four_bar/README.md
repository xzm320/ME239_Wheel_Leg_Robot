# Active elastic four-bar wheel legs

This variant replaces each serial Upkie leg with a planar diamond four-bar
mechanism while retaining the x-z compliant hip carrier.

## Geometry

- Four load-bearing links per leg, each 190 mm between pivots
- Nominal knee-to-knee crossbar length: 150 mm
- Nominal hip-to-wheel distance: 348 mm
- Two synchronized telescopic stages: -20 to +94 mm per stage
- Total crossbar range: 110–338 mm
- Geometric hip-to-wheel range: approximately 169–364 mm
- Minimum-to-neutral leg-height ratio: 48.7%
- Wheel radius/diameter: 120/240 mm

The orange branch owns the wheel body. The blue branch closes onto the same
bottom pivot through a three-dimensional `equality/connect` constraint. A
second closure connects the telescopic green crossbar to the opposite knee.
This keeps an explicit rigid-body closed chain instead of approximating its
kinematics with a polynomial joint mapping.

## Active elastic crossbar

Each crossbar is represented by three nested bodies. Two 1:1 synchronized
slide stages provide the long travel without allowing a single inner rod to
leave its sleeve:

| Property | Value |
| --- | ---: |
| Equivalent passive stiffness | 900 N/m |
| Equivalent passive damping | 50 N·s/m |
| Per-stage stiffness/damping | 1800 N/m / 100 N·s/m |
| Position-servo stiffness/damping | 8000 N/m / 100 N·s/m |
| Actuator force limit | ±700 N |
| Per-stage physical range | -20 to +94 mm |
| Total active crossbar travel | 228 mm |
| Minimum stage insertion | 26 mm |

A 20 mm total passive displacement produces a 30 N restoring force. The actuator
can push and pull; it is not modeled as a tension-only cable.

Because both synchronized stage springs load the first-stage actuator, a
desired per-stage extension `q_des` needs the static feed-forward command
`ctrl = (1 + 3600 / 8000) * q_des = 1.45 * q_des`.

Positive crossbar extension widens the diamond and shortens the leg. Negative
extension narrows it and lengthens the leg.

At maximum shortening, the outer-to-middle and middle-to-inner overlaps are
approximately 36 and 26 mm. Thus both visual geometry and equality-constrained
physics remain engaged over the complete travel.

## Tuned passive compliance

The deployed values are rounded engineering values from a seeded 53-candidate
search on the medium terrain at 1.5 m/s:

| Element | Stiffness | Damping |
| --- | ---: | ---: |
| Hip carrier x | 7100 N/m | 48 N·s/m |
| Hip carrier z | 7800 N/m | 165 N·s/m |
| Complete telescopic crossbar | 900 N/m | 50 N·s/m |

The multi-objective score includes center-of-mass height variation, vertical
acceleration, attitude, speed tracking, and strut force. Relative to the prior
6500/70, 12000/120, and 1500/25 values, the rounded result reduces RMS
vertical acceleration by about 45%, its 95th percentile by about 71%, and
95th-percentile strut force by about 36%. Center-of-mass standard deviation
changes from 10.94 to 11.00 mm, so the impact reduction is not obtained by
allowing materially larger body-height motion. Reproduce the search with
`uv run python scripts/tune_compliance.py`.

## Mass and scope

The model is 5.6185 kg. Per-leg moving mass is held within 1 g of the imported
Upkie baseline by redistributing the original limb mass over four links, the
telescopic assembly, wheel node, and wheel.

The baseline 1.7 N·m wheel velocity servos are replaced by direct-drive motor
actuators with a symmetric ±6 N·m peak torque limit. Direct torque input is
required by the cascaded balance controller, and the higher peak limit is an
explicit high-speed drivetrain upgrade rather than a controller-only change.
The larger wheels retain the lightweight 0.2385 kg design mass but have higher
rotational inertia from their 120 mm radius. Continuous motor and thermal
limits are not yet modeled.

Link collision is intentionally disabled in this mechanism-validation model
to prevent adjacent capsules at ideal pin joints from self-penetrating.
Wheel and trunk collision remain active. Dedicated link collision shapes will
be introduced when the rough-terrain module defines clearance requirements.

The flat-ground controller gains are deterministic initial values. They are
not yet optimized against terrain or motor thermal constraints.

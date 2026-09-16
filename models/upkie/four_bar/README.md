# Active elastic four-bar wheel legs

This variant replaces each serial Upkie leg with a planar diamond four-bar
mechanism while retaining the x-z compliant hip carrier.

## Geometry

- Four load-bearing links per leg, each 190 mm between pivots
- Nominal knee-to-knee crossbar length: 150 mm
- Nominal hip-to-wheel distance: 348 mm
- Crossbar extension range: ±40 mm
- Geometric hip-to-wheel range: approximately 328–364 mm

The orange branch owns the wheel body. The blue branch closes onto the same
bottom pivot through a three-dimensional `equality/connect` constraint. A
second closure connects the telescopic green crossbar to the opposite knee.
This keeps an explicit rigid-body closed chain instead of approximating its
kinematics with a polynomial joint mapping.

## Active elastic crossbar

Each crossbar is represented by two rigid telescopic bodies:

| Property | Value |
| --- | ---: |
| Passive stiffness | 1500 N/m |
| Passive damping | 25 N·s/m |
| Position-servo stiffness | 2500 N/m |
| Position-servo damping | 45 N·s/m |
| Actuator force limit | ±250 N |
| Physical slide range | ±40 mm |
| Minimum rod insertion | 30 mm |

A 20 mm passive displacement produces a 30 N restoring force. The actuator
can push and pull; it is not modeled as a tension-only cable.

Because the passive spring also pulls toward zero, a desired physical
extension `q_des` needs the static feed-forward command
`ctrl = (1 + k_spring / k_servo) * q_des = 1.6 * q_des`.
The actuator control range is therefore ±64 mm while the mechanical joint
remains limited to ±40 mm.

Positive crossbar extension widens the diamond and shortens the leg. Negative
extension narrows it and lengthens the leg.

The outer sleeve is 110 mm long. The visible inner rod starts 30 mm behind
its body origin, so it overlaps the sleeve by 70 mm at neutral and 30 mm at
maximum extension. The MuJoCo slide joint is connected for its full range
regardless of visual geometry, while this overlap constraint also keeps the
model mechanically realizable.

## Mass and scope

The model is 5.6185 kg. Per-leg moving mass is held within 1 g of the imported
Upkie baseline by redistributing the original limb mass over four links, the
telescopic assembly, wheel node, and wheel.

Link collision is intentionally disabled in this mechanism-validation model
to prevent adjacent capsules at ideal pin joints from self-penetrating.
Wheel and trunk collision remain active. Dedicated link collision shapes will
be introduced when the rough-terrain module defines clearance requirements.

The current gains are engineering initial values. They are not the final
high-speed control gains and have not yet been optimized against terrain.

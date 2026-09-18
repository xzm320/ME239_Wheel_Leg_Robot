# Ablation prototype

This is `wide_car` with the three comparison features stripped out, so the
two robots share trunk, track (±0.3411 m), wheels (r = 0.120 m, ±6 N·m),
standing height (z = 0.408 m), and mass (5.877 kg):

- hip mounts are welded to the trunk (no x–z compliant slides)
- each leg is a serial 2-link chain (hip → knee → wheel)
- no telescopic crossbar, no diamond four-bar, no equality constraints

The knee is held by a position servo at the same `kp=30` used on the
wide_car hip; under the four-bar that hinge was passive. Ballast in the
hip mounts replaces the mass of the deleted rear links and struts, and
is placed low so the standing COM height stays close to `wide_car`.

This is **not** the official Upkie. Licensed under Apache-2.0; see
`LICENSE`.

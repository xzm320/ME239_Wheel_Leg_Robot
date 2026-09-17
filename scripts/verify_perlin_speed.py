#!/usr/bin/env python3
"""Verify Perlin-strip speed limits for the 3x car and stock Upkie."""

from __future__ import annotations

from dataclasses import asdict
import json

import mujoco
import numpy as np

if __package__:
    from scripts.balance_controller import (
        perlin_wide_balance_gains,
        perlin_wide_heading_gains,
        upkie_balance_gains,
        upkie_heading_gains,
    )
    from scripts.perlin_speed import (
        UPKIE_PERLIN_MOVE_FAIL_M_S,
        UPKIE_SCENE,
        WIDE_PERLIN_FAIL_M_S,
        WIDE_PERLIN_LIMIT_M_S,
        lane_held,
        run_upkie_perlin_episode,
        run_wide_perlin_episode,
        _summarize,
    )
else:
    from balance_controller import (
        perlin_wide_balance_gains,
        perlin_wide_heading_gains,
        upkie_balance_gains,
        upkie_heading_gains,
    )
    from perlin_speed import (
        UPKIE_PERLIN_MOVE_FAIL_M_S,
        UPKIE_SCENE,
        WIDE_PERLIN_FAIL_M_S,
        WIDE_PERLIN_LIMIT_M_S,
        lane_held,
        run_upkie_perlin_episode,
        run_wide_perlin_episode,
        _summarize,
    )


def verify() -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(UPKIE_SCENE))
    assert (model.nq, model.nv, model.nu) == (13, 12, 6)
    mass = round(float(np.sum(model.body_mass)), 4)
    assert mass == 5.4605

    wide_hold = run_wide_perlin_episode(
        WIDE_PERLIN_LIMIT_M_S, duration_s=8.0, rolling_start=True
    )
    wide_fail = run_wide_perlin_episode(
        WIDE_PERLIN_FAIL_M_S, duration_s=5.0, rolling_start=True
    )
    upkie_stand = run_upkie_perlin_episode(0.0, duration_s=6.0, rolling_start=True)
    upkie_move = run_upkie_perlin_episode(
        UPKIE_PERLIN_MOVE_FAIL_M_S, duration_s=4.0, rolling_start=True
    )

    wide_hold_summary = _summarize(wide_hold)
    assert lane_held(wide_hold), wide_hold_summary
    assert wide_hold.peak_speed_m_s * 3.6 > 9.5, wide_hold_summary
    assert not lane_held(wide_fail), _summarize(wide_fail)
    assert wide_fail.failure_reason is not None, _summarize(wide_fail)

    stand = _summarize(upkie_stand)
    assert lane_held(upkie_stand), stand
    assert upkie_stand.maximum_pitch_deg < 5.0, stand
    assert not lane_held(upkie_move), _summarize(upkie_move)
    assert upkie_move.failure_reason is not None, _summarize(upkie_move)

    return {
        "upkie_model": {"nq": model.nq, "nu": model.nu, "mass_kg": mass},
        "wide": {
            "limit_m_s": WIDE_PERLIN_LIMIT_M_S,
            "pid": {
                "balance": asdict(perlin_wide_balance_gains()),
                "heading": asdict(perlin_wide_heading_gains()),
            },
            "hold": wide_hold_summary,
            "fail": _summarize(wide_fail),
        },
        "upkie": {
            "pid": {
                "balance": asdict(upkie_balance_gains()),
                "heading": asdict(upkie_heading_gains()),
            },
            "stand": stand,
            "move_fail": _summarize(upkie_move),
        },
    }


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))

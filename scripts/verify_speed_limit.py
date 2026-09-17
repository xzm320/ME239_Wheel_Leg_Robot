#!/usr/bin/env python3
"""Verify tuned pad PID at the 3x-car and prototype speed limits."""

from __future__ import annotations

from dataclasses import asdict
import json

if __package__:
    from scripts.balance_controller import (
        prototype_balance_gains,
        prototype_heading_gains,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from scripts.speed_limit import (
        PROTOTYPE_LIMIT_KMH,
        PROTOTYPE_MODEL_PATH,
        WIDE_LIMIT_KMH,
        WIDE_MODEL_PATH,
        lane_held,
        rolling_limit_episode,
        summarize,
    )
else:
    from balance_controller import (
        prototype_balance_gains,
        prototype_heading_gains,
        wide_car_balance_gains,
        wide_car_heading_gains,
    )
    from speed_limit import (
        PROTOTYPE_LIMIT_KMH,
        PROTOTYPE_MODEL_PATH,
        WIDE_LIMIT_KMH,
        WIDE_MODEL_PATH,
        lane_held,
        rolling_limit_episode,
        summarize,
    )


def _check(
    *,
    name: str,
    model_path,
    kmh: float,
    scale: float | None,
    soften: bool,
    balance,
    heading,
) -> dict[str, object]:
    result = rolling_limit_episode(
        model_path=model_path,
        kmh=kmh,
        wheel_track_scale=scale,
        soften_struts=soften,
        balance_gains=balance,
        heading_gains=heading,
    )
    payload = summarize(result)
    payload["robot"] = name
    assert lane_held(result), payload
    assert result.peak_speed_m_s * 3.6 > kmh - 2.0, payload
    return payload


def verify() -> dict[str, object]:
    wide_balance = wide_car_balance_gains()
    wide_heading = wide_car_heading_gains()
    proto_balance = prototype_balance_gains()
    proto_heading = prototype_heading_gains()
    wide = {
        "limit_kmh": WIDE_LIMIT_KMH,
        "pid": {
            "balance": asdict(wide_balance),
            "heading": asdict(wide_heading),
        },
        "cruise_190": _check(
            name="wide",
            model_path=WIDE_MODEL_PATH,
            kmh=190.0,
            scale=3.0,
            soften=True,
            balance=wide_balance,
            heading=wide_heading,
        ),
        "cruise_195": _check(
            name="wide",
            model_path=WIDE_MODEL_PATH,
            kmh=WIDE_LIMIT_KMH,
            scale=3.0,
            soften=True,
            balance=wide_balance,
            heading=wide_heading,
        ),
    }
    prototype = {
        "limit_kmh": PROTOTYPE_LIMIT_KMH,
        "pid": {
            "balance": asdict(proto_balance),
            "heading": asdict(proto_heading),
        },
        "cruise_170": _check(
            name="prototype",
            model_path=PROTOTYPE_MODEL_PATH,
            kmh=170.0,
            scale=None,
            soften=False,
            balance=proto_balance,
            heading=proto_heading,
        ),
        "cruise_180": _check(
            name="prototype",
            model_path=PROTOTYPE_MODEL_PATH,
            kmh=PROTOTYPE_LIMIT_KMH,
            scale=None,
            soften=False,
            balance=proto_balance,
            heading=proto_heading,
        ),
    }
    return {"wide_car": wide, "prototype": prototype}


if __name__ == "__main__":
    print(json.dumps(verify(), ensure_ascii=False, indent=2))

"""Standing-height kinematics for the diamond four-bar.

Widening the wheel track and lengthening the links are complementary, but they
solve different problems. Track width is a roll lever. Link length is stroke.
Raising standing height to "use" longer links undoes the low CoG that the wide
chassis is trying to keep.
"""

from __future__ import annotations

import math


NOMINAL_LINK_LENGTH_M = 0.190
NOMINAL_HALF_CROSSBAR_M = 0.075
STANDING_DIAMOND_HEIGHT_M = 0.348
STAGE_MIN_M = -0.020
STAGE_MAX_M = 0.094
NOMINAL_CROSSBAR_LENGTH_M = 0.150


def diamond_height_m(link_length_m: float, half_crossbar_m: float) -> float:
    inner = link_length_m**2 - half_crossbar_m**2
    if inner <= 0.0:
        return 0.0
    return 2.0 * math.sqrt(inner)


def half_crossbar_for_height_m(
    link_length_m: float, diamond_height_m: float
) -> float:
    inner = link_length_m**2 - (0.5 * diamond_height_m) ** 2
    if inner <= 0.0:
        return 0.0
    return math.sqrt(inner)


def stage_extension_m(half_crossbar_m: float) -> float:
    return (2.0 * half_crossbar_m - NOMINAL_CROSSBAR_LENGTH_M) / 2.0


def available_compression_m(
    link_length_m: float,
    standing_height_m: float = STANDING_DIAMOND_HEIGHT_M,
) -> float:
    """Stroke from the standing pose down to the strut packing limit."""

    standing_half = half_crossbar_for_height_m(link_length_m, standing_height_m)
    packed_half = min(
        link_length_m - 1e-4,
        0.5 * (NOMINAL_CROSSBAR_LENGTH_M + 2.0 * STAGE_MAX_M),
    )
    packed_height = diamond_height_m(link_length_m, packed_half)
    return max(0.0, standing_height_m - packed_height)


def lengthening_report(
    candidate_length_m: float = 0.230,
) -> dict[str, float]:
    """Compare current links with a longer set kept at the same standing CoG."""

    current_stroke = available_compression_m(NOMINAL_LINK_LENGTH_M)
    longer_half = half_crossbar_for_height_m(
        candidate_length_m, STANDING_DIAMOND_HEIGHT_M
    )
    longer_stage = stage_extension_m(longer_half)
    longer_stroke = available_compression_m(candidate_length_m)
    return {
        "standing_diamond_height_m": STANDING_DIAMOND_HEIGHT_M,
        "current_link_length_m": NOMINAL_LINK_LENGTH_M,
        "current_compression_stroke_m": current_stroke,
        "candidate_link_length_m": candidate_length_m,
        "candidate_standing_half_crossbar_m": longer_half,
        "candidate_standing_stage_m": longer_stage,
        "candidate_compression_stroke_m": longer_stroke,
        "extra_stroke_m": longer_stroke - current_stroke,
    }

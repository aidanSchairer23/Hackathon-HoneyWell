"""Protection shells: the boundaries around an aircraft, as objects.

Every threshold the alerting logic uses is restated here as a Shell with
attributes a display can draw: a horizontal radius, a vertical half-height and
a severity rank. Nothing here changes any alerting decision; alerting.py is
still the only place that decides TA and RA. This module answers a different
question: "which boundaries is this target inside right now, and how far from
the next one?"

Shape. The boundaries are flat discs, not spheres. The threat volume is
0.5 nm (3,038 ft) wide but only 600 ft tall, so a sphere would be five times
too tall. Seen from above a disc is a ring; seen from the side it is a box.

Two kinds of shell:

  distance  a fixed radius around the aircraft (surveillance, watch, threat,
            core). A target is inside when its current horizontal range and
            its current altitude difference are both inside the shell.

  time      the TA and RA thresholds are times to closest approach, not
            distances, so their radius depends on how fast the two aircraft
            are closing. For a target flying a straight line,

                range(t)^2 = HMD^2 + (relative speed * time to CPA)^2

            so the range at which time-to-CPA equals T seconds is

                radius = sqrt(HMD^2 + (relative speed * T)^2)

            A fast head-on pushes the ring out to miles; a slow overtake pulls
            it in to a few hundred metres. The ring is exact: a threat is
            inside it if and only if alerting.classify() gives that level.
            It only applies to targets that meet the threat test, because a
            target that will miss by a mile never alerts however near in time.

Shells are listed from the outside in, rank 0 (just tracked) to rank 5 (the RA
did not achieve its target separation).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .alerting import is_threat
from .config import DEFAULT, M_PER_FT, Config
from .prediction import Prediction


@dataclass(frozen=True)
class Shell:
    key: str                       # stable identifier: "surveillance", "watch", "ta", "ra", "threat", "core"
    label: str                     # short name for a display
    rank: int                      # 0 = outermost; higher = closer / more severe
    kind: str                      # "distance" or "time"
    radius_m: float | None         # fixed horizontal radius (distance shells)
    seconds: float | None          # time-to-CPA threshold (time shells)
    half_height_ft: float | None   # vertical half-extent; None = no vertical limit
    meaning: str                   # what being inside this shell means
    source: tuple[str, ...]        # the Config fields that define it


def shells(cfg: Config = DEFAULT) -> tuple[Shell, ...]:
    """The shells around an aircraft, outermost first, built from Config."""
    return (
        Shell("surveillance", "Surveillance", 0, "distance", cfg.surveillance_range_m, None, None,
              "Tracked.", ("surveillance_range_mi",)),
        Shell("watch", "Watch", 1, "distance", cfg.mandatory_range_m, None, None,
              "Evaluated on every cycle.", ("mandatory_range_mi",)),
        Shell("ta", "Traffic advisory", 2, "time", None, cfg.ta_tcpa_s, cfg.threat_vmd_ft,
              "A threat this close in time gets a traffic advisory.",
              ("ta_tcpa_s", "threat_hmd_nm", "threat_vmd_ft")),
        Shell("ra", "Resolution advisory", 3, "time", None, cfg.ra_tcpa_s, cfg.threat_vmd_ft,
              "A threat this close in time gets a resolution advisory.",
              ("ra_tcpa_s", "threat_hmd_nm", "threat_vmd_ft")),
        Shell("threat", "Threat volume", 4, "distance", cfg.threat_hmd_m, None, cfg.threat_vmd_ft,
              "A predicted miss inside this volume is a threat. A target physically inside it gets an RA.",
              ("threat_hmd_nm", "threat_vmd_ft")),
        Shell("core", "RA target", 5, "distance", cfg.ra_success_hmd_m, None, cfg.ra_success_vmd_ft,
              "An RA is meant to keep traffic out of this volume.",
              ("ra_success_hmd_nm", "ra_success_vmd_ft")),
    )


def time_shell_radius_m(seconds: float, hmd_m: float, rel_speed_mps: float) -> float:
    """Range at which a straight-line target's time to CPA equals `seconds`."""
    return math.hypot(hmd_m, rel_speed_mps * seconds)


@dataclass(frozen=True)
class ShellState:
    key: str
    applies: bool            # False for a time shell when the target is not a threat
    radius_m: float | None   # resolved radius for this target; None when it does not apply
    inside: bool
    margin_m: float | None   # horizontal range minus radius (negative = inside the ring)


def shell_states(pred: Prediction, rel_speed_mps: float, cfg: Config = DEFAULT,
                 alertable: bool = True) -> tuple[ShellState, ...]:
    """Where one target stands against every shell, outermost first.

    rel_speed_mps is the horizontal speed of the target relative to ownship.
    alertable is False for an aircraft on the ground: it is never alerted on,
    so the TA and RA time shells do not apply to it. The distance shells still
    report where it physically is.
    """
    threat = alertable and is_threat(pred, cfg)
    out = []
    for sh in shells(cfg):
        if sh.kind == "time":
            if not threat:
                out.append(ShellState(sh.key, False, None, False, None))
                continue
            radius = time_shell_radius_m(sh.seconds, pred.hmd_m, rel_speed_mps)
            # Decide on the time itself, which is what alerting.classify() does;
            # the radius is that same test expressed as a distance.
            inside = pred.tcpa_s <= sh.seconds
        else:
            radius = sh.radius_m
            inside = pred.range_m <= radius if sh.half_height_ft is None else (
                pred.range_m < radius and abs(pred.rel_alt_ft) < sh.half_height_ft)
        out.append(ShellState(sh.key, True, radius, inside, pred.range_m - radius))
    return tuple(out)


def innermost(states: tuple[ShellState, ...]) -> str | None:
    """Key of the highest-rank shell the target is inside, or None if outside them all."""
    key = None
    for st in states:          # listed outermost first, so the last hit is the innermost
        if st.inside:
            key = st.key
    return key


def proximity(pred: Prediction, cfg: Config = DEFAULT) -> float:
    """Physical closeness from 0 (at or beyond the watch ring) to 1 (same point).

    Uses slant distance with the altitude difference scaled so that the threat
    volume's height counts the same as its radius. A target 600 ft above at
    zero range is then as "close" as one level at 0.5 nm.
    """
    vertical_scale = cfg.threat_hmd_m / (cfg.threat_vmd_ft * M_PER_FT)
    d = math.hypot(pred.range_m, pred.rel_alt_ft * M_PER_FT * vertical_scale)
    return max(0.0, 1.0 - d / cfg.mandatory_range_m)

"""Surveillance: which targets are tracked and evaluated on a cycle.

Run once per cycle (config.cycle_period_s, 1 Hz). Every non-stale target
within surveillance_range_mi of ownship is tracked. There is no cap on the
number of tracks, so every target within mandatory_range_mi is always tracked
and evaluated. A target whose latest report is older than the cycle time is
dead-reckoned (coasted) to the cycle time instead of being skipped, until it
goes stale. Tracks are returned closest first.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .config import DEFAULT, Config
from .prediction import extrapolate, geodetic_to_enu
from .traffic import Target, drop_stale


@dataclass(frozen=True)
class Track:
    target: Target       # state extrapolated to the cycle time
    range_m: float       # horizontal range from ownship
    report_age_s: float  # age of the report the track is built from


def build_tracks(own: Target, reports: Iterable[Target], now: float,
                 cfg: Config = DEFAULT) -> list[Track]:
    latest: dict[str, Target] = {}
    own_icao = own.icao.lower()
    for r in drop_stale(reports, now, cfg.stale_target_s):
        if r.icao.lower() != own_icao and (r.icao not in latest or r.timestamp > latest[r.icao].timestamp):
            latest[r.icao] = r

    tracks = []
    for report in latest.values():
        t = extrapolate(report, now)
        enu = geodetic_to_enu(t.lat, t.lon, t.alt_ft, own.lat, own.lon, own.alt_ft)
        rng = math.hypot(enu[0], enu[1])
        if rng <= cfg.surveillance_range_m:
            tracks.append(Track(t, rng, now - report.timestamp))
    tracks.sort(key=lambda tr: tr.range_m)
    return tracks

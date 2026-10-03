"""Threat detection and alert state. No I/O.

A threat is a target whose predicted horizontal miss is under threat_hmd_nm
AND whose predicted vertical miss is under threat_vmd_ft. A threat gets a
traffic advisory (TA) when tCPA <= ta_tcpa_s and a resolution advisory (RA)
when tCPA <= ra_tcpa_s. Thresholds are in config.py.

Once issued, an RA is held until tCPA has passed (tCPA == 0, the aircraft are
no longer converging) and the threat condition is false. This stops the RA
from dropping out mid-maneuver when the maneuver itself makes the predicted
miss distance exceed the thresholds.
"""

from __future__ import annotations

from enum import IntEnum
from typing import Iterable

from .config import DEFAULT, Config
from .prediction import Prediction


class AlertLevel(IntEnum):
    NONE = 0
    TA = 1
    RA = 2


def is_threat(pred: Prediction, cfg: Config = DEFAULT) -> bool:
    return pred.hmd_m < cfg.threat_hmd_m and pred.vmd_ft < cfg.threat_vmd_ft


def classify(pred: Prediction, cfg: Config = DEFAULT) -> AlertLevel:
    if not is_threat(pred, cfg):
        return AlertLevel.NONE
    if pred.tcpa_s <= cfg.ra_tcpa_s:
        return AlertLevel.RA
    if pred.tcpa_s <= cfg.ta_tcpa_s:
        return AlertLevel.TA
    return AlertLevel.NONE


def ra_can_end(pred: Prediction, cfg: Config = DEFAULT) -> bool:
    """True once tCPA has passed and the threat condition is false."""
    return pred.tcpa_s <= 0.0 and not is_threat(pred, cfg)


class AlertTracker:
    """Alert level per target, with RAs held until they can end."""

    def __init__(self, cfg: Config = DEFAULT):
        self.cfg = cfg
        self._levels: dict[str, AlertLevel] = {}

    def level(self, icao: str) -> AlertLevel:
        return self._levels.get(icao, AlertLevel.NONE)

    def update(self, pred: Prediction, ra_allowed: bool = True) -> tuple[AlertLevel, AlertLevel]:
        """Record this cycle's prediction. Returns (previous level, new level).

        With ra_allowed False (ownship is on the ground) the level is capped
        at TA: traffic is still called out, but no maneuver is advised.
        """
        prev = self.level(pred.icao)
        new = classify(pred, self.cfg)
        if not ra_allowed:
            new = min(new, AlertLevel.TA)
        elif prev is AlertLevel.RA and not ra_can_end(pred, self.cfg):
            new = AlertLevel.RA
        self._levels[pred.icao] = new
        return prev, new

    def clear(self, icao: str) -> AlertLevel:
        """Set a tracked target to no alert (it is on the ground). Returns the previous level."""
        prev = self.level(icao)
        self._levels[icao] = AlertLevel.NONE
        return prev

    def forget_missing(self, present: Iterable[str]) -> list[str]:
        """Drop targets that are no longer tracked. Returns those that were in RA."""
        present = set(present)
        gone = [icao for icao in self._levels if icao not in present]
        lost_ras = [icao for icao in gone if self._levels[icao] is AlertLevel.RA]
        for icao in gone:
            del self._levels[icao]
        return lost_ras

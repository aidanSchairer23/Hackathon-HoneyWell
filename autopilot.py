"""Compliance monitoring and autopilot handoff.

SIMULATION ONLY. SimulatedAutopilot changes the state of a simulated aircraft
object. Nothing in this package can send commands to a real aircraft.

After an RA is issued, HandoffMonitor watches ownship for compliance: vertical
rate moving in the commanded sense, or track moving in the commanded
direction. If the pilot has not complied within compliance_timeout_s, the RA
is handed to the autopilot, which holds it until the RA ends ("Clear of
conflict"), then releases.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from .config import DEFAULT, Config
from .prediction import wrap180
from .resolution import Maneuver
from .traffic import SimAircraft, Target


class Autopilot(ABC):
    @property
    @abstractmethod
    def engaged(self) -> bool: ...

    @abstractmethod
    def engage(self, maneuver: Maneuver) -> None:
        """Fly `maneuver` until released. Calling again changes the maneuver."""

    @abstractmethod
    def release(self) -> None: ...


class SimulatedAutopilot(Autopilot):
    """Flies RAs by commanding a SimAircraft. On release it rolls wings level
    and levels off."""

    def __init__(self, aircraft: SimAircraft, cfg: Config = DEFAULT):
        self.aircraft = aircraft
        self.cfg = cfg
        self.maneuver: Maneuver | None = None

    @property
    def engaged(self) -> bool:
        return self.maneuver is not None

    def engage(self, maneuver: Maneuver) -> None:
        self.maneuver = maneuver
        self.aircraft.fly(maneuver, self.cfg)

    def release(self) -> None:
        self.maneuver = None
        self.aircraft.wings_level_and_level_off()


def has_complied(maneuver: Maneuver, at_issue: Target, now: Target, cfg: Config = DEFAULT) -> bool:
    """Is ownship moving in the commanded direction relative to when the RA was issued?"""
    if maneuver is Maneuver.CLIMB:
        return (now.vs_fpm - at_issue.vs_fpm >= cfg.compliance_vs_delta_fpm
                or now.vs_fpm >= cfg.ra_climb_fpm)
    if maneuver is Maneuver.DESCEND:
        return (at_issue.vs_fpm - now.vs_fpm >= cfg.compliance_vs_delta_fpm
                or now.vs_fpm <= -cfg.ra_descent_fpm)
    turned = wrap180(now.track_deg - at_issue.track_deg)
    if maneuver is Maneuver.TURN_RIGHT:
        return turned >= cfg.compliance_track_delta_deg
    return turned <= -cfg.compliance_track_delta_deg


class HandoffMonitor:
    """Tracks the RA ownship should be flying and hands it to the autopilot
    if the pilot does not comply in time. With no autopilot it only records."""

    def __init__(self, autopilot: Autopilot | None, cfg: Config = DEFAULT):
        self.autopilot = autopilot
        self.cfg = cfg
        self.maneuver: Maneuver | None = None
        self._issued_at = 0.0
        self._at_issue: Target | None = None
        self.complied = False

    def ra_issued(self, maneuver: Maneuver, own: Target, now: float) -> None:
        """A new RA (or a changed one) is now in force."""
        self.maneuver = maneuver
        self._issued_at = now
        self._at_issue = own
        self.complied = False
        if self.autopilot is not None and self.autopilot.engaged:
            self.autopilot.engage(maneuver)

    def update(self, own: Target, now: float) -> bool:
        """Check compliance. Returns True on the cycle the autopilot engages."""
        if (self.maneuver is None or self.complied or self.autopilot is None
                or self.autopilot.engaged):
            return False
        if has_complied(self.maneuver, self._at_issue, own, self.cfg):
            self.complied = True
            return False
        if now - self._issued_at >= self.cfg.compliance_timeout_s - 1e-9:
            self.autopilot.engage(self.maneuver)
            return True
        return False

    def ra_ended(self) -> bool:
        """The RA is over. Releases the autopilot; returns True if it was engaged."""
        self.maneuver = None
        if self.autopilot is not None and self.autopilot.engaged:
            self.autopilot.release()
            return True
        return False

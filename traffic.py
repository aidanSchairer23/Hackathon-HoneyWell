"""Traffic sources: scripted simulation and live dump1090 data.

Nothing in this module talks to real aircraft controls. SimAircraft is a
kinematic model that only exists inside the simulation.
"""

from __future__ import annotations

import json
import logging
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import requests

from .config import DEFAULT, G_MPS2, M_PER_FT, M_PER_STATUTE_MILE, Config
from .prediction import enu_to_geodetic, turn_rate_dps, velocity_enu
from .resolution import Maneuver, commands_for

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Target:
    icao: str
    lat: float
    lon: float
    alt_ft: float
    gs_kt: float
    track_deg: float
    vs_fpm: float
    timestamp: float  # seconds, on the caller's clock
    on_ground: bool = False  # parked, taxiing or rolling: shown, never alerted on


def drop_stale(targets: Iterable[Target], now: float, max_age_s: float) -> list[Target]:
    """Keep only targets updated within max_age_s of now."""
    return [t for t in targets if now - t.timestamp <= max_age_s]


class TrafficSource(ABC):
    @abstractmethod
    def poll(self, now: float) -> list[Target]:
        """Return the latest report for every target, excluding stale ones."""


# --------------------------------------------------------------------------- #
# Live data
# --------------------------------------------------------------------------- #

def parse_aircraft_json(data: dict, now: float, ground_alt_ft: float | None = None) -> list[Target]:
    """Convert a dump1090 aircraft.json document into Targets.

    Reads hex, lat, lon, alt_baro, gs, track and baro_rate. A record without a
    position is skipped. Missing gs, track or baro_rate are treated as 0. The
    report time is now minus seen_pos (falling back to seen), so the receiver's
    clock never matters.

    An aircraft on the ground reports alt_baro as the word "ground", with no
    altitude. If ground_alt_ft (the field elevation) is given, such a record
    becomes a Target with on_ground=True at that altitude; without it the
    record is skipped, as is any other non-numeric altitude.
    """
    targets = []
    for ac in data.get("aircraft", []):
        lat, lon, alt = ac.get("lat"), ac.get("lon"), ac.get("alt_baro")
        if lat is None or lon is None:
            continue
        on_ground = alt == "ground"
        if on_ground:
            if ground_alt_ft is None:
                continue
            alt = ground_alt_ft
        elif not isinstance(alt, (int, float)) or isinstance(alt, bool):
            continue
        age = ac.get("seen_pos", ac.get("seen", 0.0)) or 0.0
        targets.append(Target(
            icao=str(ac.get("hex", "")).strip(),
            lat=float(lat),
            lon=float(lon),
            alt_ft=float(alt),
            gs_kt=float(ac.get("gs") or 0.0),
            track_deg=float(ac.get("track") or 0.0),
            vs_fpm=0.0 if on_ground else float(ac.get("baro_rate") or 0.0),
            timestamp=now - float(age),
            on_ground=on_ground,
        ))
    return targets


class Dump1090Source(TrafficSource):
    """Polls a dump1090 aircraft.json URL. A failed poll keeps serving cached
    targets until they go stale."""

    def __init__(self, url: str, cfg: Config = DEFAULT, session: requests.Session | None = None,
                 ground_alt_ft: float | None = None):
        self.url = url
        self.cfg = cfg
        self.ground_alt_ft = ground_alt_ft   # field elevation; lets on-ground aircraft be shown
        self._session = session or requests.Session()
        self._latest: dict[str, Target] = {}
        self._failing = False

    def poll(self, now: float) -> list[Target]:
        try:
            resp = self._session.get(self.url, timeout=self.cfg.dump1090_timeout_s)
            resp.raise_for_status()
            reports = parse_aircraft_json(resp.json(), now, self.ground_alt_ft)
        except (requests.RequestException, ValueError) as exc:
            if not self._failing:
                log.warning("dump1090 poll failed (%s); serving cached targets", exc)
            self._failing = True
        else:
            if self._failing:
                log.warning("dump1090 poll recovered")
            self._failing = False
            for t in reports:
                old = self._latest.get(t.icao)
                if old is None or t.timestamp >= old.timestamp:
                    self._latest[t.icao] = t
        fresh = drop_stale(self._latest.values(), now, self.cfg.stale_target_s)
        self._latest = {t.icao: t for t in fresh}
        return fresh


# --------------------------------------------------------------------------- #
# Simulation
# --------------------------------------------------------------------------- #

@dataclass
class SimAircraft:
    """Kinematic point-mass aircraft. Commands are set by the simulated pilot
    or the simulated autopilot; step() integrates them."""

    icao: str
    lat: float
    lon: float
    alt_ft: float
    gs_kt: float
    track_deg: float
    vs_fpm: float
    cmd_vs_fpm: float | None = None  # None: hold the current vertical rate
    cmd_turn: int = 0                # -1 left, 0 hold track, +1 right
    on_ground: bool = False          # parked or taxiing; set on touchdown if field_elev_ft is known
    field_elev_ft: float | None = None
    landed: bool = False             # touched down during this simulation (rolls out, then taxis)

    def fly(self, maneuver: Maneuver, cfg: Config = DEFAULT) -> None:
        if self.on_ground:
            return                   # an aircraft on the ground cannot fly an RA
        self.cmd_vs_fpm, self.cmd_turn = commands_for(maneuver, self.vs_fpm, cfg)

    def wings_level_and_level_off(self) -> None:
        self.cmd_vs_fpm, self.cmd_turn = 0.0, 0

    def step(self, dt: float, cfg: Config = DEFAULT) -> None:
        if self.on_ground:
            self.vs_fpm, self.cmd_vs_fpm, self.cmd_turn = 0.0, 0.0, 0
            if self.landed and self.gs_kt > cfg.sim_taxi_speed_kt:
                self.gs_kt = max(cfg.sim_taxi_speed_kt, self.gs_kt - cfg.sim_rollout_decel_kt_s * dt)
            move = velocity_enu(self.gs_kt, self.track_deg, 0.0) * dt
            self.lat, self.lon, _ = enu_to_geodetic(move, self.lat, self.lon, self.alt_ft)
            return
        old_vel = velocity_enu(self.gs_kt, self.track_deg, self.vs_fpm)
        if self.cmd_vs_fpm is not None:
            max_dv = cfg.vertical_accel_g * G_MPS2 * dt / M_PER_FT * 60.0
            dv = self.cmd_vs_fpm - self.vs_fpm
            self.vs_fpm += math.copysign(min(abs(dv), max_dv), dv)
        if self.cmd_turn:
            rate = turn_rate_dps(self.gs_kt, cfg.ra_bank_deg)
            self.track_deg = (self.track_deg + self.cmd_turn * rate * dt) % 360.0
        new_vel = velocity_enu(self.gs_kt, self.track_deg, self.vs_fpm)
        move = (old_vel + new_vel) / 2.0 * dt
        self.lat, self.lon, self.alt_ft = enu_to_geodetic(move, self.lat, self.lon, self.alt_ft)
        if self.field_elev_ft is not None and self.alt_ft <= self.field_elev_ft and self.vs_fpm < 0:
            self.alt_ft, self.vs_fpm = self.field_elev_ft, 0.0          # touchdown
            self.on_ground = self.landed = True

    def as_target(self, now: float) -> Target:
        return Target(self.icao, self.lat, self.lon, self.alt_ft,
                      self.gs_kt, self.track_deg, self.vs_fpm, now, self.on_ground)


@dataclass
class SimPilot:
    """Scripted pilot. 'comply' flies each RA after delay_s; 'ignore' does nothing."""

    response: str = "ignore"
    delay_s: float = 2.0
    _pending: tuple[float, Maneuver | None] | None = None

    def on_ra(self, maneuver: Maneuver, now: float) -> None:
        if self.response == "comply":
            self._pending = (now + self.delay_s, maneuver)

    def on_clear(self, now: float) -> None:
        if self.response == "comply":
            self._pending = (now + self.delay_s, None)

    def update(self, aircraft: SimAircraft, now: float, cfg: Config = DEFAULT) -> None:
        if self._pending and now >= self._pending[0] - 1e-9:
            maneuver = self._pending[1]
            self._pending = None
            if maneuver is None:
                aircraft.wings_level_and_level_off()
            else:
                aircraft.fly(maneuver, cfg)


def _aircraft_from_spec(spec: dict, origin: tuple[float, float, float] | None,
                        field_elev_ft: float | None = None) -> SimAircraft:
    if "lat" in spec:
        lat, lon = spec["lat"], spec["lon"]
    else:
        if origin is None:
            raise ValueError("ownship must be given with lat/lon")
        offset = [spec.get("east_mi", 0.0) * M_PER_STATUTE_MILE,
                  spec.get("north_mi", 0.0) * M_PER_STATUTE_MILE, 0.0]
        lat, lon, _ = enu_to_geodetic(offset, *origin)
    on_ground = bool(spec.get("on_ground", False))
    if on_ground and "alt_ft" not in spec and field_elev_ft is None:
        raise ValueError(f"{spec['icao']}: an on_ground aircraft needs alt_ft or a scenario field_elev_ft")
    return SimAircraft(
        icao=spec["icao"], lat=lat, lon=lon,
        alt_ft=spec["alt_ft"] if "alt_ft" in spec else field_elev_ft,
        gs_kt=spec["gs_kt"], track_deg=spec["track_deg"],
        vs_fpm=0.0 if on_ground else spec.get("vs_fpm", 0.0),
        on_ground=on_ground, field_elev_ft=field_elev_ft,
    )


class SimulatedSource(TrafficSource):
    """Plays a scripted scenario: an ownship, its pilot model, and intruders
    flying constant velocity. Intruders may be placed by lat/lon or by
    east_mi/north_mi offsets from ownship's starting position.

    An optional "field_elev_ft" gives the ground elevation: an aircraft that
    descends to it touches down and becomes on_ground, and descend RAs are
    inhibited close to it. An aircraft spec with "on_ground": true starts on
    the ground (alt_ft defaults to the field elevation)."""

    def __init__(self, scenario: dict, cfg: Config = DEFAULT):
        self.cfg = cfg
        self.name = scenario.get("name", "unnamed scenario")
        self.duration_s = float(scenario.get("duration_s", 120.0))
        elev = scenario.get("field_elev_ft")
        self.field_elev_ft = None if elev is None else float(elev)
        self.ownship = _aircraft_from_spec(scenario["ownship"], None, self.field_elev_ft)
        origin = (self.ownship.lat, self.ownship.lon, self.ownship.alt_ft)
        self.intruders = [_aircraft_from_spec(s, origin, self.field_elev_ft)
                          for s in scenario.get("intruders", [])]
        pilot = scenario.get("pilot", {})
        self.pilot = SimPilot(pilot.get("response", "ignore"), float(pilot.get("delay_s", 2.0)))
        self.time = 0.0

    @classmethod
    def from_file(cls, path: str | Path, cfg: Config = DEFAULT) -> SimulatedSource:
        with open(path, encoding="utf-8") as f:
            return cls(json.load(f), cfg)

    def ownship_target(self) -> Target:
        return self.ownship.as_target(self.time)

    def poll(self, now: float) -> list[Target]:
        return drop_stale((a.as_target(self.time) for a in self.intruders),
                          now, self.cfg.stale_target_s)

    def advance(self, dt: float) -> None:
        steps = max(1, round(dt / self.cfg.sim_substep_s))
        h = dt / steps
        t0 = self.time
        for i in range(steps):
            self.pilot.update(self.ownship, self.time, self.cfg)
            for ac in (self.ownship, *self.intruders):
                ac.step(h, self.cfg)
            self.time = t0 + (i + 1) * h
        self.time = t0 + dt

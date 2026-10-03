"""CLI entry point and the per-cycle pipeline.

    python -m gatcas --scenario scenarios/head_on.json [--fast]
    python -m gatcas --dump1090 URL --lat LAT --lon LON --alt FT

Each cycle: surveillance -> prediction -> alerting -> resolution -> handoff,
then one log line per tracked target and any alert events.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable

from .alerting import AlertLevel, AlertTracker
from .audio import AUTOPILOT_PHRASE, CLEAR_PHRASE, Announcer, ra_phrase, ta_phrase
from .autopilot import Autopilot, HandoffMonitor, SimulatedAutopilot
from .config import DEFAULT, Config
from .prediction import Prediction, predict
from .resolution import Maneuver, Resolution, select_ra
from .surveillance import build_tracks
from .traffic import Dump1090Source, SimulatedSource, Target, TrafficSource

Out = Callable[[str], None]


@dataclass(frozen=True)
class Event:
    time: float
    kind: str                # "TA", "RA", "AUTOPILOT", "CLEAR", "RA_LOST"
    icao: str
    phrase: str | None       # spoken text; None for log-only events
    maneuver: Maneuver | None = None
    detail: str = ""


@dataclass(frozen=True)
class TargetStatus:
    pred: Prediction
    level: AlertLevel
    resolution: Resolution | None
    on_ground: bool = False      # tracked and shown, never alerted on


@dataclass(frozen=True)
class CycleReport:
    time: float
    targets: list[TargetStatus]
    events: list[Event]
    active_ra: tuple[str, Maneuver] | None
    autopilot_engaged: bool
    own_on_ground: bool = False  # RAs are inhibited while this is True


class CasSystem:
    """One collision-avoidance unit. Call cycle() once per cycle period."""

    def __init__(self, cfg: Config = DEFAULT, autopilot: Autopilot | None = None,
                 ground_elev_ft: float | None = None):
        self.cfg = cfg
        self.autopilot = autopilot
        self.ground_elev_ft = ground_elev_ft   # None: unknown, so no descend floor is applied
        self.alerts = AlertTracker(cfg)
        self.handoff = HandoffMonitor(autopilot, cfg)
        self.resolutions: dict[str, Resolution] = {}
        self.active: tuple[str, Maneuver] | None = None

    def cycle(self, now: float, own: Target, reports: Iterable[Target]) -> CycleReport:
        cfg = self.cfg
        events: list[Event] = []
        tracks = build_tracks(own, reports, now, cfg)
        lost = set(self.alerts.forget_missing(t.target.icao for t in tracks))
        for icao in lost:
            self.resolutions.pop(icao, None)

        # Too close to the ground to be told to descend?
        allow_descend = (self.ground_elev_ft is None
                         or own.alt_ft - self.ground_elev_ft >= cfg.ra_descend_min_agl_ft)
        statuses = []
        for tr in tracks:
            pred = predict(own, tr.target)
            if tr.target.on_ground:
                # Parked and taxiing aircraft stay on the display but are never a threat.
                self.alerts.clear(pred.icao)
                self.resolutions.pop(pred.icao, None)
                statuses.append(TargetStatus(pred, AlertLevel.NONE, None, True))
                continue
            # Ownship on the ground cannot maneuver: traffic advisories only.
            prev, level = self.alerts.update(pred, ra_allowed=not own.on_ground)
            if level is AlertLevel.RA and prev is not AlertLevel.RA:
                self.resolutions[pred.icao] = select_ra(own, tr.target, cfg, allow_descend)
            elif level is not AlertLevel.RA:
                self.resolutions.pop(pred.icao, None)
            if level is AlertLevel.TA and prev is AlertLevel.NONE:
                events.append(Event(now, "TA", pred.icao, ta_phrase(pred, own.track_deg, cfg)))
            statuses.append(TargetStatus(pred, level, self.resolutions.get(pred.icao)))

        # Ownship can fly one maneuver: the RA against the threat with the smallest tCPA.
        in_ra = [s for s in statuses if s.resolution is not None]
        active = None
        if in_ra:
            s = min(in_ra, key=lambda s: s.pred.tcpa_s)
            active = (s.pred.icao, s.resolution.maneuver)

        if active is not None and (self.active is None or active[1] is not self.active[1]):
            res = self.resolutions[active[0]]
            events.append(Event(now, "RA", active[0], ra_phrase(active[1]), active[1],
                                detail=_describe_scores(res)))
            self.handoff.ra_issued(active[1], own, now)
        elif active is None and self.active is not None:
            released = self.handoff.ra_ended()
            note = "autopilot released" if released else ""
            if self.active[0] in lost:
                events.append(Event(now, "RA_LOST", self.active[0], None,
                                    detail="track lost, RA ended" + (f"; {note}" if note else "")))
            else:
                events.append(Event(now, "CLEAR", self.active[0], CLEAR_PHRASE, detail=note))
        self.active = active

        if self.handoff.update(own, now):
            events.append(Event(now, "AUTOPILOT", active[0], AUTOPILOT_PHRASE, active[1],
                                detail=f"pilot did not comply within {cfg.compliance_timeout_s:g} s"))

        return CycleReport(now, statuses, events, active,
                           self.autopilot is not None and self.autopilot.engaged, own.on_ground)


def _describe_scores(res: Resolution) -> str:
    scores = ", ".join(f"{m.value} {s:.2f}" for m, s in res.scores.items())
    rule = "; tie broken by coordination rule" if res.coordinated else ""
    basis = f"; {res.basis}" if res.basis else ""
    return f"predicted separation at CPA / RA success target: {scores}{rule}{basis}"


# --------------------------------------------------------------------------- #
# Output
# --------------------------------------------------------------------------- #

def format_target_line(t: float, s: TargetStatus) -> str:
    p = s.pred
    state = "GND" if s.on_ground else s.level.name
    if s.resolution is not None:
        state = f"RA {s.resolution.maneuver.value.upper()}"
    return (f"[{t:7.1f}s] {p.icao:<7} rng {p.range_mi:5.2f} mi  rel alt {p.rel_alt_ft:+6.0f} ft  "
            f"tCPA {p.tcpa_s:5.1f} s  HMD {p.hmd_nm:5.2f} nm  VMD {p.vmd_ft:5.0f} ft  {state}")


EVENT_TITLES = {
    "TA": "TRAFFIC ADVISORY",
    "RA": "RESOLUTION ADVISORY",
    "AUTOPILOT": "AUTOPILOT ENGAGING",
    "CLEAR": "CLEAR OF CONFLICT",
    "RA_LOST": "RA TARGET LOST",
}


def format_event(t: float, ev: Event) -> str:
    line = f"[{t:7.1f}s] *** {EVENT_TITLES[ev.kind]} {ev.icao}"
    if ev.maneuver is not None:
        line += f": {ev.maneuver.value.upper()}"
    if ev.detail:
        line += f" ({ev.detail})"
    return line


def emit(report: CycleReport, t: float, out: Out, announcer: Announcer | None) -> None:
    for s in report.targets:
        out(format_target_line(t, s))
    for ev in report.events:
        out(format_event(t, ev))
        if ev.phrase and announcer is not None:
            announcer.say(ev.phrase)


# --------------------------------------------------------------------------- #
# Run loops
# --------------------------------------------------------------------------- #

@dataclass
class RunResult:
    reports: list[CycleReport] = field(default_factory=list)

    @property
    def events(self) -> list[Event]:
        return [e for r in self.reports for e in r.events]


def run_simulation(source: SimulatedSource, *, fast: bool = False, cfg: Config = DEFAULT,
                   announcer: Announcer | None = None, out: Out = print,
                   autopilot_enabled: bool = True) -> RunResult:
    autopilot = SimulatedAutopilot(source.ownship, cfg) if autopilot_enabled else None
    cas = CasSystem(cfg, autopilot, getattr(source, "field_elev_ft", None))
    o = source.ownship
    out("SIMULATION ONLY - not connected to any aircraft controls.")
    out(f"Scenario: {source.name} ({source.duration_s:g} s, {'fast' if fast else 'real time'})")
    out(f"Ownship {o.icao}: {o.gs_kt:g} kt, track {o.track_deg:03.0f}, {o.alt_ft:.0f} ft. "
        f"Pilot model: {source.pilot.response} RAs. "
        f"Simulated autopilot: {'enabled' if autopilot else 'disabled'}.")

    period = cfg.cycle_period_s
    n_cycles = int(round(source.duration_s / period))
    result = RunResult()
    start = time.monotonic()
    for k in range(n_cycles + 1):
        now = source.time
        report = cas.cycle(now, source.ownship_target(), source.poll(now))
        emit(report, now, out, announcer)
        for ev in report.events:
            if ev.kind == "RA":
                source.pilot.on_ra(ev.maneuver, now)
            elif ev.kind in ("CLEAR", "RA_LOST"):
                source.pilot.on_clear(now)
        result.reports.append(report)
        if k == n_cycles:
            break
        if not fast:
            time.sleep(max(0.0, start + (k + 1) * period - time.monotonic()))
        source.advance(period)
    out("Scenario complete.")
    return result


def run_scenario(path: str | Path, **kwargs) -> RunResult:
    cfg = kwargs.get("cfg", DEFAULT)
    return run_simulation(SimulatedSource.from_file(path, cfg), **kwargs)


def run_live(source: TrafficSource, lat: float, lon: float, alt_ft: float, *,
             cfg: Config = DEFAULT, announcer: Announcer | None = None, out: Out = print,
             max_cycles: int | None = None, clock: Callable[[], float] = time.time,
             sleep: Callable[[float], None] = time.sleep, on_ground: bool = False,
             ground_elev_ft: float | None = None) -> RunResult:
    """Fixed ownship position, live traffic, autopilot disabled. Runs until
    interrupted (or for max_cycles). on_ground marks the fixed ownship as being
    on the ground (traffic advisories only); ground_elev_ft is the field
    elevation used for the descend floor."""
    cas = CasSystem(cfg, autopilot=None, ground_elev_ft=ground_elev_ft)
    out("LIVE MODE - advisory only, autopilot disabled.")
    out(f"Ownship fixed at {lat:.5f}, {lon:.5f}, {alt_ft:.0f} ft"
        f"{' (on the ground: traffic advisories only)' if on_ground else ''}. Press Ctrl+C to stop.")
    period = cfg.cycle_period_s
    result = RunResult()
    start = clock()
    k = 0
    while max_cycles is None or k < max_cycles:
        now = clock()
        own = Target(cfg.live_ownship_icao, lat, lon, alt_ft, 0.0, 0.0, 0.0, now, on_ground)
        report = cas.cycle(now, own, source.poll(now))
        emit(report, now - start, out, announcer)
        result.reports.append(report)
        k += 1
        sleep(max(0.0, start + k * period - clock()))
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m gatcas",
        description="ADS-B traffic collision avoidance prototype (simulation and demo only).")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--scenario", metavar="FILE", help="scenario JSON file to simulate")
    source.add_argument("--dump1090", metavar="URL", help="dump1090 aircraft.json URL to poll")
    parser.add_argument("--fast", action="store_true",
                        help="run the scenario without real-time pacing (speech off)")
    parser.add_argument("--lat", type=float, help="live mode: ownship latitude, degrees")
    parser.add_argument("--lon", type=float, help="live mode: ownship longitude, degrees")
    parser.add_argument("--alt", type=float, help="live mode: ownship pressure altitude, feet")
    parser.add_argument("--field-elev", type=float, metavar="FT",
                        help="live mode: field elevation, feet. Shows aircraft on the ground and sets the descend floor")
    parser.add_argument("--on-ground", action="store_true",
                        help="live mode: the fixed ownship is on the ground (traffic advisories only)")
    args = parser.parse_args(argv)

    if args.dump1090 and None in (args.lat, args.lon, args.alt):
        parser.error("--dump1090 requires --lat, --lon and --alt")
    if args.dump1090 and args.fast:
        parser.error("--fast only applies to --scenario")

    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    sys.stdout.reconfigure(line_buffering=True)  # flush each log line, even when piped
    announcer = Announcer(speech=not args.fast)
    try:
        if args.scenario:
            run_scenario(args.scenario, fast=args.fast, announcer=announcer)
        else:
            run_live(Dump1090Source(args.dump1090, ground_alt_ft=args.field_elev),
                     args.lat, args.lon, args.alt, announcer=announcer,
                     on_ground=args.on_ground, ground_elev_ft=args.field_elev)
    except KeyboardInterrupt:
        print("Stopped.")
        return 130
    finally:
        announcer.close()
    return 0

from dataclasses import replace
from pathlib import Path

import pytest

from gatcas.alerting import is_threat
from gatcas.autopilot import HandoffMonitor, SimulatedAutopilot, has_complied
from gatcas.config import DEFAULT, Config
from gatcas.main import run_scenario, run_simulation
from gatcas.resolution import Maneuver
from gatcas.traffic import SimAircraft, SimulatedSource
from helpers import own

SCENARIOS = Path(__file__).resolve().parent.parent / "scenarios"
CLIMB, DESCEND, LEFT, RIGHT = Maneuver.CLIMB, Maneuver.DESCEND, Maneuver.TURN_LEFT, Maneuver.TURN_RIGHT
BEST = replace(DEFAULT, ra_vertical_first=False)   # the original rule: most predicted separation


def quiet(name, **kw):
    return run_scenario(SCENARIOS / name, fast=True, out=lambda s: None, **kw)


def kinds(result):
    return [e.kind for e in result.events]


def test_has_complied_requires_motion_in_commanded_direction():
    start = own(vs_fpm=0.0, track_deg=350.0)
    assert has_complied(CLIMB, start, replace(start, vs_fpm=250.0))
    assert not has_complied(CLIMB, start, replace(start, vs_fpm=150.0))
    assert not has_complied(CLIMB, start, replace(start, vs_fpm=-400.0))
    assert has_complied(DESCEND, start, replace(start, vs_fpm=-250.0))
    assert has_complied(CLIMB, replace(start, vs_fpm=700.0), replace(start, vs_fpm=700.0))  # already climbing
    assert has_complied(RIGHT, start, replace(start, track_deg=355.0))
    assert has_complied(RIGHT, start, replace(start, track_deg=10.0))     # through north
    assert not has_complied(RIGHT, start, replace(start, track_deg=340.0))
    assert has_complied(LEFT, start, replace(start, track_deg=340.0))
    assert not has_complied(LEFT, start, replace(start, track_deg=352.0))


def test_simulated_autopilot_commands_and_releases_the_sim_aircraft():
    ac = SimAircraft("A00001", 39.5, -104.8, 6000.0, 100.0, 0.0, 0.0)
    ap = SimulatedAutopilot(ac)
    ap.engage(CLIMB)
    assert ap.engaged and ac.cmd_vs_fpm == 500.0
    ap.engage(RIGHT)
    assert ac.cmd_turn == 1
    ap.release()
    assert not ap.engaged and (ac.cmd_vs_fpm, ac.cmd_turn) == (0.0, 0)


def test_handoff_monitor_engages_exactly_at_the_timeout():
    ac = SimAircraft("A00001", 39.5, -104.8, 6000.0, 100.0, 0.0, 0.0)
    mon = HandoffMonitor(SimulatedAutopilot(ac))
    o = own()
    mon.ra_issued(CLIMB, o, now=10.0)
    assert [mon.update(o, t) for t in (10.0, 11.0, 12.0, 13.0, 14.0)] == [False] * 5
    assert mon.update(o, 15.0) is True
    assert mon.update(o, 16.0) is False          # engages once
    assert mon.ra_ended() is True and not mon.autopilot.engaged


def test_handoff_timeout_is_configurable():
    cfg = Config(compliance_timeout_s=2.0)
    ac = SimAircraft("A00001", 39.5, -104.8, 6000.0, 100.0, 0.0, 0.0)
    mon = HandoffMonitor(SimulatedAutopilot(ac, cfg), cfg)
    mon.ra_issued(RIGHT, own(), now=0.0)
    assert [mon.update(own(), t) for t in (1.0, 2.0)] == [False, True]


def test_no_autopilot_means_no_handoff():
    mon = HandoffMonitor(None)
    mon.ra_issued(CLIMB, own(), now=0.0)
    assert not any(mon.update(own(), float(t)) for t in range(30))
    assert mon.ra_ended() is False


@pytest.mark.parametrize("name", ["head_on_compliant.json", "crossing_90.json", "overtaking.json"])
@pytest.mark.parametrize("delay_s", [0.0, 1.0, 2.0, 3.0])
def test_compliant_pilot_never_triggers_autopilot_handoff(name, delay_s):
    source = SimulatedSource.from_file(SCENARIOS / name)
    source.pilot.response, source.pilot.delay_s = "comply", delay_s
    result = run_simulation(source, fast=True, out=lambda s: None)
    assert "RA" in kinds(result)            # an RA really was issued...
    assert "AUTOPILOT" not in kinds(result)  # ...and the pilot flew it, so no handoff
    assert not any(r.autopilot_engaged for r in result.reports)
    assert kinds(result)[-1] == "CLEAR"


def test_ignored_ra_hands_off_after_timeout_and_climbs():
    # Altitude first (the default): the RA is CLIMB, the autopilot takes it 5 s
    # later, and from then on the predicted vertical miss opens up.
    result = quiet("head_on.json")
    events = {e.kind: e for e in result.events}
    assert kinds(result) == ["TA", "RA", "AUTOPILOT", "CLEAR"]
    ra, ap, clear = events["RA"], events["AUTOPILOT"], events["CLEAR"]
    assert ap.time - ra.time == DEFAULT.compliance_timeout_s
    assert ap.maneuver is ra.maneuver is CLIMB

    def pred(r):
        [s] = r.targets
        return s.pred

    before = [pred(r) for r in result.reports if ra.time <= r.time <= ap.time]
    after = [pred(r) for r in result.reports if r.time > ap.time and pred(r).tcpa_s > 0]
    assert all(p.vmd_ft == pytest.approx(0.0, abs=1.0) for p in before)   # pilot did nothing
    assert all(p.vmd_ft > 200.0 for p in after)                           # the climb is in
    assert all(p.hmd_m < 1.0 for p in after)                              # no turn at all
    # Ownship alone, climbing at 500 fpm: under the 300 ft target (see test_coordination).
    passing = min((pred(r) for r in result.reports), key=lambda p: p.range_m)
    assert 230.0 <= abs(passing.rel_alt_ft) < DEFAULT.ra_success_vmd_ft
    assert all(r.autopilot_engaged for r in result.reports if ap.time <= r.time < clear.time)
    assert not result.reports[-1].autopilot_engaged


def test_ignored_ra_hands_off_after_timeout_and_resolves_the_threat():
    result = quiet("head_on.json", cfg=BEST)
    events = {e.kind: e for e in result.events}
    assert kinds(result) == ["TA", "RA", "AUTOPILOT", "CLEAR"]
    ra, ap = events["RA"], events["AUTOPILOT"]
    assert ap.time - ra.time == DEFAULT.compliance_timeout_s
    assert ap.maneuver is ra.maneuver is RIGHT

    def pred(r):
        [s] = r.targets
        return s.pred

    before = [r for r in result.reports if ra.time <= r.time <= ap.time]
    after = [r for r in result.reports if r.time > ap.time and pred(r).tcpa_s > 0]
    # Until the handoff the pilot ignores the RA: the predicted miss stays inside the thresholds.
    assert all(is_threat(pred(r)) for r in before)
    # After the handoff, still before CPA, the predicted miss first reaches the
    # RA success target, then exceeds the threat thresholds, and stays out.
    success = [r for r in after if pred(r).hmd_m >= DEFAULT.ra_success_hmd_m]
    resolved = [r for r in after if not is_threat(pred(r))]
    assert success and resolved
    assert ap.time < success[0].time <= resolved[0].time
    assert pred(resolved[0]).hmd_m >= DEFAULT.threat_hmd_m
    assert all(not is_threat(pred(r)) for r in after if r.time >= resolved[0].time)
    # The autopilot holds the RA until clear of conflict, then releases.
    clear = events["CLEAR"]
    assert all(r.autopilot_engaged for r in result.reports if ap.time <= r.time < clear.time)
    assert not result.reports[-1].autopilot_engaged


def test_same_encounter_without_autopilot_stays_a_threat_until_cpa():
    result = quiet("head_on.json", autopilot_enabled=False)
    assert "AUTOPILOT" not in kinds(result)
    ra_time = next(e.time for e in result.events if e.kind == "RA")
    for r in result.reports:
        [s] = r.targets
        if r.time >= ra_time and s.pred.tcpa_s > 0:
            assert is_threat(s.pred)


def test_head_on_autopilot_engages_then_releases_on_clear_of_conflict():
    result = quiet("head_on.json")
    assert kinds(result) == ["TA", "RA", "AUTOPILOT", "CLEAR"]
    clear = result.events[-1]
    assert clear.detail == "autopilot released"
    assert not any(r.autopilot_engaged for r in result.reports if r.time >= clear.time)

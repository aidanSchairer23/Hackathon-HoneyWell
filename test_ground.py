"""Aircraft on the ground: shown and tracked, never alerted on."""

import json
from dataclasses import replace
from pathlib import Path

import pytest

from gatcas.alerting import AlertLevel, AlertTracker
from gatcas.config import DEFAULT, MPS_PER_KT
from gatcas.main import CasSystem, format_target_line, run_live, run_scenario, run_simulation
from gatcas.prediction import predict
from gatcas.resolution import BASIS_TURN_GROUND, Maneuver
from gatcas.traffic import Dump1090Source, SimAircraft, SimulatedSource, parse_aircraft_json
from gatcas.viz import record_scenario
from gatcas.zones import shell_states
from helpers import at, own

SCENARIOS = Path(__file__).resolve().parent.parent / "scenarios"
V = 110 * MPS_PER_KT
FIELD = 5400.0


def head_on_at(tcpa_s, **kw):
    return at("B00002", north_m=2 * V * tcpa_s, track_deg=180, **kw)


# ---- the engine ------------------------------------------------------------

def test_a_grounded_target_is_tracked_but_never_alerted_on():
    cas = CasSystem()
    for k, tcpa in enumerate((60, 45, 30, 15, 5)):          # straight at us, co-altitude
        grounded = replace(head_on_at(tcpa, t=float(k)), on_ground=True)
        report = cas.cycle(float(k), own(t=float(k)), [grounded])
        [status] = report.targets
        assert status.on_ground and status.level is AlertLevel.NONE and status.resolution is None
        assert not report.events and report.active_ra is None
        assert format_target_line(float(k), status).endswith("GND")
    # The same aircraft, airborne, is an RA straight away.
    report = CasSystem().cycle(0.0, own(), [head_on_at(5)])
    assert [e.kind for e in report.events] == ["RA"]


def test_an_ra_ends_when_its_target_lands():
    cas = CasSystem()
    assert [e.kind for e in cas.cycle(0.0, own(), [head_on_at(20)]).events] == ["RA"]
    report = cas.cycle(1.0, own(t=1.0), [replace(head_on_at(19, t=1.0), on_ground=True)])
    assert [e.kind for e in report.events] == ["CLEAR"]
    assert report.active_ra is None and report.targets[0].level is AlertLevel.NONE


def test_ownship_on_the_ground_gets_traffic_advisories_only():
    cas = CasSystem()
    me = replace(own(), on_ground=True)
    report = cas.cycle(0.0, me, [head_on_at(20)])            # an RA if we were flying
    assert report.own_on_ground
    assert [e.kind for e in report.events] == ["TA"]
    assert report.targets[0].level is AlertLevel.TA and report.active_ra is None


def test_alert_tracker_caps_at_ta_and_clears():
    tracker = AlertTracker()
    pred = predict(own(), head_on_at(20))
    assert tracker.update(pred, ra_allowed=False) == (AlertLevel.NONE, AlertLevel.TA)
    assert tracker.update(pred) == (AlertLevel.TA, AlertLevel.RA)
    assert tracker.clear(pred.icao) is AlertLevel.RA
    assert tracker.level(pred.icao) is AlertLevel.NONE


def test_time_shells_do_not_apply_to_an_aircraft_on_the_ground():
    pred = predict(own(), head_on_at(20))
    flying = {s.key: s for s in shell_states(pred, 2 * V)}
    parked = {s.key: s for s in shell_states(pred, 2 * V, alertable=False)}
    assert flying["ra"].applies and flying["ra"].inside
    assert not parked["ta"].applies and not parked["ra"].applies and not parked["ra"].inside
    assert parked["watch"] == flying["watch"]               # where it physically is does not change


# ---- the descend floor -----------------------------------------------------

@pytest.mark.parametrize("agl_ft, expect_turn", [(700.0, True), (999.0, True), (1000.0, False), (2500.0, False)])
def test_descend_ra_is_replaced_by_a_turn_below_the_floor(agl_ft, expect_turn):
    cas = CasSystem(ground_elev_ft=FIELD)
    me = own(alt_ft=FIELD + agl_ft)
    above = head_on_at(30, alt_ft=FIELD + agl_ft + 150.0)   # 150 ft above: the sense is descend
    [status] = cas.cycle(0.0, me, [above]).targets
    assert status.resolution.vertical_sense is Maneuver.DESCEND
    if expect_turn:
        assert not status.resolution.maneuver.is_vertical
        assert status.resolution.basis == BASIS_TURN_GROUND
    else:
        assert status.resolution.maneuver is Maneuver.DESCEND


def test_a_climb_ra_is_never_affected_by_the_floor():
    cas = CasSystem(ground_elev_ft=FIELD)
    me = own(alt_ft=FIELD + 400.0)
    [status] = cas.cycle(0.0, me, [head_on_at(30, alt_ft=FIELD + 250.0)]).targets
    assert status.resolution.maneuver is Maneuver.CLIMB


def test_no_floor_when_the_ground_elevation_is_unknown():
    me = own(alt_ft=300.0)
    [status] = CasSystem().cycle(0.0, me, [head_on_at(30, alt_ft=450.0)]).targets
    assert status.resolution.maneuver is Maneuver.DESCEND


# ---- the simulator ---------------------------------------------------------

def test_sim_aircraft_touches_down_rolls_out_and_taxis():
    ac = SimAircraft("A00001", 39.5, -104.8, FIELD + 50.0, 70.0, 0.0, -500.0, field_elev_ft=FIELD)
    for _ in range(59):
        ac.step(0.1)
    assert not ac.on_ground and ac.alt_ft > FIELD
    for _ in range(2):
        ac.step(0.1)
    assert ac.on_ground and ac.landed and ac.alt_ft == FIELD and ac.vs_fpm == 0.0
    assert ac.as_target(0.0).on_ground
    lat = ac.lat
    for _ in range(200):                                    # 20 s: 70 kt down to taxi speed at 4 kt/s
        ac.step(0.1)
    assert ac.gs_kt == DEFAULT.sim_taxi_speed_kt and ac.alt_ft == FIELD and ac.lat > lat
    ac.fly(Maneuver.CLIMB)                                  # cannot fly an RA from the ground
    ac.step(0.1)
    assert ac.vs_fpm == 0.0 and ac.alt_ft == FIELD


def test_sim_aircraft_without_a_field_elevation_never_lands():
    ac = SimAircraft("A00001", 39.5, -104.8, 20.0, 70.0, 0.0, -500.0)
    for _ in range(100):
        ac.step(0.1)
    assert not ac.on_ground and ac.alt_ft < 0.0


def test_a_parked_aircraft_does_not_slow_down_or_move():
    ac = SimAircraft("A00001", 39.5, -104.8, FIELD, 0.0, 90.0, 0.0, on_ground=True, field_elev_ft=FIELD)
    for _ in range(50):
        ac.step(0.1)
    assert (ac.lat, ac.lon, ac.alt_ft, ac.gs_kt) == (39.5, -104.8, FIELD, 0.0)
    taxi = SimAircraft("A00002", 39.5, -104.8, FIELD, 12.0, 0.0, 0.0, on_ground=True, field_elev_ft=FIELD)
    for _ in range(50):
        taxi.step(0.1)
    assert taxi.gs_kt == 12.0 and taxi.lat > 39.5           # a taxiing aircraft keeps its own speed


def test_scenario_on_ground_aircraft_default_to_the_field_elevation():
    src = SimulatedSource.from_file(SCENARIOS / "ground_traffic.json")
    assert src.field_elev_ft == FIELD
    grounded = [a for a in src.intruders if a.on_ground]
    assert [a.icao for a in grounded] == ["A0D001", "A0D002", "A0D003"]
    assert all(a.alt_ft == FIELD and a.vs_fpm == 0.0 for a in grounded)
    with pytest.raises(ValueError):
        SimulatedSource({"ownship": {"icao": "A00001", "lat": 0, "lon": 0, "alt_ft": 0, "gs_kt": 0, "track_deg": 0},
                         "intruders": [{"icao": "A00002", "east_mi": 1, "north_mi": 1, "on_ground": True,
                                        "gs_kt": 0, "track_deg": 0}]})


# ---- the two scenarios -----------------------------------------------------

def test_landing_scenario_raises_no_advisory():
    result = run_scenario(SCENARIOS / "ground_traffic.json", fast=True, out=lambda s: None)
    assert result.events == []
    touchdown = next(r.time for r in result.reports if r.own_on_ground)
    assert 70.0 <= touchdown <= 74.0                        # 600 ft at 500 fpm
    by_icao = {s.pred.icao: s for s in result.reports[0].targets}
    assert [i for i, s in by_icao.items() if s.on_ground] == ["A0D001", "A0D003", "A0D002"]
    assert not by_icao["A0A004"].on_ground and not by_icao["A0A005"].on_ground
    # It taxis past the holding aircraft at well inside the RA target distance: still silent.
    closest = min(s.pred.range_m for r in result.reports for s in r.targets if s.pred.icao == "A0D001")
    assert closest < DEFAULT.ra_success_hmd_m


def test_the_same_landing_without_ground_flags_is_a_nuisance_ra():
    """What the ground logic prevents: a climb RA on short final against an
    aircraft holding short of the runway."""
    spec = json.loads((SCENARIOS / "ground_traffic.json").read_text())
    del spec["field_elev_ft"]
    for i in spec["intruders"]:
        if i.pop("on_ground", False):
            i["alt_ft"] = FIELD
    result = run_simulation(SimulatedSource(spec), fast=True, out=lambda s: None)
    kinds = [(e.kind, e.icao) for e in result.events]
    assert ("TA", "A0D001") in kinds and ("RA", "A0D001") in kinds and ("AUTOPILOT", "A0D001") in kinds
    assert next(e.maneuver for e in result.events if e.kind == "RA") is Maneuver.CLIMB


def test_low_level_scenario_turns_because_it_cannot_descend():
    lines = []
    result = run_scenario(SCENARIOS / "low_level_turn.json", fast=True, out=lines.append)
    assert [e.kind for e in result.events] == ["TA", "RA", "AUTOPILOT", "CLEAR"]
    ra = next(e for e in result.events if e.kind == "RA")
    assert ra.maneuver is Maneuver.TURN_RIGHT
    assert any("too low to descend" in line for line in lines)
    # 700 ft above the field the whole way: the turn never takes it lower.
    spec = json.loads((SCENARIOS / "low_level_turn.json").read_text())
    del spec["field_elev_ft"]
    free = run_simulation(SimulatedSource(spec), fast=True, out=lambda s: None)
    assert next(e.maneuver for e in free.events if e.kind == "RA") is Maneuver.DESCEND


# ---- live input ------------------------------------------------------------

AIRCRAFT_JSON = {"aircraft": [
    {"hex": "a0aaaa", "lat": 39.58, "lon": -104.85, "alt_baro": "ground", "gs": 14, "track": 90,
     "baro_rate": -64, "seen_pos": 0.4},
    {"hex": "a0bbbb", "lat": 39.59, "lon": -104.85, "alt_baro": 7500, "gs": 120, "track": 180, "seen_pos": 0.1},
    {"hex": "a0cccc", "lat": 39.59, "lon": -104.85, "alt_baro": None},
]}


def test_ground_records_are_skipped_unless_the_field_elevation_is_known():
    assert [t.icao for t in parse_aircraft_json(AIRCRAFT_JSON, now=100.0)] == ["a0bbbb"]
    ground, flying = parse_aircraft_json(AIRCRAFT_JSON, now=100.0, ground_alt_ft=FIELD)
    assert (ground.icao, ground.on_ground, ground.alt_ft, ground.vs_fpm, ground.gs_kt) == ("a0aaaa", True, FIELD, 0.0, 14.0)
    assert (flying.icao, flying.on_ground) == ("a0bbbb", False)


class FakeSession:
    def get(self, url, timeout):
        return self

    def raise_for_status(self):
        pass

    def json(self):
        return AIRCRAFT_JSON


def test_live_mode_shows_ground_traffic_and_marks_ownship_on_the_ground():
    clock = [1000.0]
    lines = []
    source = Dump1090Source("http://rx/data/aircraft.json", session=FakeSession(), ground_alt_ft=FIELD)
    result = run_live(source, 39.57, -104.85, FIELD, out=lines.append, max_cycles=3,
                      clock=lambda: clock[0], sleep=lambda s: clock.__setitem__(0, clock[0] + s),
                      on_ground=True, ground_elev_ft=FIELD)
    assert all(r.own_on_ground for r in result.reports)
    assert {s.pred.icao: s.on_ground for s in result.reports[0].targets} == {"a0aaaa": True, "a0bbbb": False}
    assert not any(e.kind == "RA" for e in result.events)
    assert any("on the ground" in line for line in lines)


# ---- the display recording -------------------------------------------------

def test_recording_carries_ground_state_for_the_display():
    rec = record_scenario(SCENARIOS / "ground_traffic.json")
    assert rec["field_elev_ft"] == FIELD
    first, last = rec["frames"][0], rec["frames"][-1]
    assert (first["own"]["on_ground"], first["own"]["agl_ft"], first["own"]["descend_ok"]) == (False, 600.0, False)
    assert (last["own"]["on_ground"], last["own"]["agl_ft"]) == (True, 0.0)
    for f in rec["frames"]:
        for t in f["targets"]:
            if t["on_ground"]:
                assert t["level"] == "NONE" and not t["threat"]
                assert not t["shells"]["ta"]["applies"] and not t["shells"]["ra"]["applies"]
    rec = record_scenario(SCENARIOS / "low_level_turn.json")
    ra = next(t for f in rec["frames"] for t in f["targets"] if t["level"] == "RA")
    assert (ra["maneuver"], ra["basis"]) == ("turn right", BASIS_TURN_GROUND)
    rec = record_scenario(SCENARIOS / "head_on.json")
    assert rec["field_elev_ft"] is None
    assert rec["frames"][0]["own"]["agl_ft"] is None and rec["frames"][0]["own"]["descend_ok"] is True
    ra = next(t for f in rec["frames"] for t in f["targets"] if t["level"] == "RA")
    assert (ra["maneuver"], ra["basis"]) == ("climb", "altitude first")

import pytest
import requests

from gatcas.config import DEFAULT
from gatcas.prediction import geodetic_to_enu, turn_rate_dps
from gatcas.resolution import Maneuver
from gatcas.traffic import (Dump1090Source, SimAircraft, SimPilot, SimulatedSource, drop_stale,
                            parse_aircraft_json)
from helpers import MI, at

AIRCRAFT_JSON = {
    "now": 1700000000.0,
    "aircraft": [
        {"hex": "a1b2c3", "lat": 39.6, "lon": -104.8, "alt_baro": 6500, "gs": 120.5,
         "track": 270.0, "baro_rate": -320, "seen_pos": 1.5, "seen": 0.4},
        {"hex": "abcdef", "alt_baro": 9000},                                     # no position
        {"hex": "ad0001", "lat": 39.5, "lon": -104.8, "alt_baro": "ground"},     # on ground
        {"hex": "ad0002", "lat": 39.4, "lon": -104.7, "alt_baro": 5000},         # no velocity
    ],
}


def test_drop_stale_drops_targets_older_than_10_s():
    ts = [at("OLD", t=0.0), at("EDGE", t=5.0), at("NEW", t=14.0)]
    assert [t.icao for t in drop_stale(ts, 15.0, DEFAULT.stale_target_s)] == ["EDGE", "NEW"]


def test_parse_aircraft_json_reads_required_fields():
    targets = parse_aircraft_json(AIRCRAFT_JSON, now=100.0)
    assert [t.icao for t in targets] == ["a1b2c3", "ad0002"]
    t = targets[0]
    assert (t.lat, t.lon, t.alt_ft, t.gs_kt, t.track_deg, t.vs_fpm) == (39.6, -104.8, 6500, 120.5, 270.0, -320)
    assert t.timestamp == pytest.approx(98.5)  # now - seen_pos
    assert (targets[1].gs_kt, targets[1].track_deg, targets[1].vs_fpm) == (0.0, 0.0, 0.0)


class FakeResponse:
    def __init__(self, payload):
        self.payload = payload

    def raise_for_status(self):
        pass

    def json(self):
        return self.payload


class FakeSession:
    def __init__(self):
        self.payload = AIRCRAFT_JSON
        self.fail = False
        self.calls = []

    def get(self, url, timeout):
        self.calls.append((url, timeout))
        if self.fail:
            raise requests.ConnectionError("receiver offline")
        return FakeResponse(self.payload)


def test_dump1090_source_caches_through_failures_then_drops_after_10_s():
    session = FakeSession()
    src = Dump1090Source("http://rx/data/aircraft.json", session=session)
    assert {t.icao for t in src.poll(100.0)} == {"a1b2c3", "ad0002"}
    assert session.calls[0] == ("http://rx/data/aircraft.json", DEFAULT.dump1090_timeout_s)

    session.fail = True
    assert {t.icao for t in src.poll(105.0)} == {"a1b2c3", "ad0002"}   # cached
    # a1b2c3's last position was 1.5 s old at t=100, so it goes stale first.
    assert {t.icao for t in src.poll(109.0)} == {"ad0002"}
    assert src.poll(110.5) == []


def test_sim_aircraft_climb_reaches_commanded_rate_quickly():
    ac = SimAircraft("A", 39.5, -104.8, 6000.0, 100.0, 0.0, 0.0)
    ac.fly(Maneuver.CLIMB)
    for _ in range(20):
        ac.step(0.1)
    assert ac.vs_fpm == pytest.approx(500.0)
    assert 6000.0 + 500 / 60 * 2 - 10 < ac.alt_ft < 6000.0 + 500 / 60 * 2


def test_sim_aircraft_turns_at_bank_limited_rate():
    ac = SimAircraft("A", 39.5, -104.8, 6000.0, 100.0, 0.0, 0.0)
    ac.fly(Maneuver.TURN_LEFT)
    for _ in range(100):
        ac.step(0.1)
    assert ac.track_deg == pytest.approx(360.0 - 10 * turn_rate_dps(100.0, DEFAULT.ra_bank_deg))
    assert turn_rate_dps(100.0, 25.0) == pytest.approx(5.09, abs=0.01)  # 25 degree bank at 100 kt


def test_sim_pilot_complies_after_delay_and_ignoring_pilot_does_nothing():
    ac = SimAircraft("A", 39.5, -104.8, 6000.0, 100.0, 0.0, 0.0)
    pilot = SimPilot("comply", delay_s=2.0)
    pilot.on_ra(Maneuver.DESCEND, now=10.0)
    pilot.update(ac, 11.9)
    assert ac.cmd_vs_fpm is None
    pilot.update(ac, 12.0)
    assert ac.cmd_vs_fpm == -1000.0

    ac2 = SimAircraft("B", 39.5, -104.8, 6000.0, 100.0, 0.0, 0.0)
    lazy = SimPilot("ignore")
    lazy.on_ra(Maneuver.CLIMB, now=0.0)
    lazy.update(ac2, 100.0)
    assert (ac2.cmd_vs_fpm, ac2.cmd_turn) == (None, 0)


def test_simulated_source_places_intruders_by_offset_and_advances():
    src = SimulatedSource({
        "name": "t", "duration_s": 30,
        "ownship": {"icao": "A00001", "lat": 39.5, "lon": -104.8, "alt_ft": 6000,
                    "gs_kt": 110, "track_deg": 0},
        "intruders": [{"icao": "A00002", "east_mi": 1.0, "north_mi": 2.0, "alt_ft": 6500,
                       "gs_kt": 110, "track_deg": 180}],
    })
    own0 = src.ownship_target()
    [intr] = src.poll(0.0)
    enu = geodetic_to_enu(intr.lat, intr.lon, intr.alt_ft, own0.lat, own0.lon, own0.alt_ft)
    assert enu == pytest.approx([1 * MI, 2 * MI, 500 * 0.3048], abs=1e-6)

    src.advance(1.0)
    assert src.time == pytest.approx(1.0)
    [intr] = src.poll(1.0)
    assert intr.timestamp == pytest.approx(1.0)

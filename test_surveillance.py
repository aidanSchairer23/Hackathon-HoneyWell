import random

import pytest

from gatcas.surveillance import build_tracks
from helpers import MI, at, own


def test_tracks_targets_within_10_miles_only_closest_first():
    reports = [at("FAR", north_m=10.2 * MI, gs_kt=0), at("MID", east_m=6 * MI, gs_kt=0),
               at("NEAR", north_m=-1 * MI, gs_kt=0), at("EDGE", north_m=9.9 * MI, gs_kt=0)]
    tracks = build_tracks(own(gs_kt=0), reports, now=0.0)
    assert [t.target.icao for t in tracks] == ["NEAR", "MID", "EDGE"]


def test_ownship_own_transponder_is_not_tracked():
    o = own(icao="A0BEEF")
    echo = at("a0beef", north_m=10.0)   # dump1090 reports hex in lower case
    tracks = build_tracks(o, [o, echo, at("OTHER", north_m=MI)], now=0.0)
    assert [t.target.icao for t in tracks] == ["OTHER"]


def test_targets_within_3_miles_are_evaluated_every_cycle_while_coasting():
    """A close target with no fresh report is coasted every cycle until stale at 10 s."""
    o = own(gs_kt=0)
    report = at("CLOSE", north_m=2.9 * MI, gs_kt=120, track_deg=180, t=0.0)
    for now in range(0, 11):
        [track] = build_tracks(o, [report], now=float(now))
        assert track.report_age_s == pytest.approx(now)
        assert track.target.timestamp == now
        assert track.range_m == pytest.approx(2.9 * MI - 120 * 1852 / 3600 * now, abs=0.1)
    assert build_tracks(o, [report], now=10.5) == []


def test_every_target_within_3_miles_tracked_even_in_dense_traffic():
    rng = random.Random(1)
    close = [at(f"C{i:04d}", east_m=rng.uniform(-2, 2) * MI, north_m=rng.uniform(-2, 2) * MI,
                t=-rng.uniform(0, 9)) for i in range(300)]
    far = [at(f"F{i:04d}", east_m=rng.uniform(4, 7) * MI, north_m=rng.uniform(4, 7) * MI)
           for i in range(300)]
    tracks = build_tracks(own(), close + far, now=0.0)
    assert {t.icao for t in close} <= {t.target.icao for t in tracks}


def test_latest_report_wins_when_duplicated():
    old = at("DUP", north_m=2 * MI, gs_kt=0, t=0.0)
    new = at("DUP", north_m=1 * MI, gs_kt=0, t=1.0)
    [track] = build_tracks(own(gs_kt=0), [new, old], now=1.0)
    assert track.range_m == pytest.approx(1 * MI)

import math

import pytest

from gatcas.config import MPS_PER_KT
from gatcas.prediction import (WGS84_A, WGS84_E2, enu_to_geodetic, extrapolate, geodetic_to_enu,
                               predict, turn_rate_dps, velocity_enu)
from helpers import MI, NM, ORIGIN, at, own

V110 = 110 * MPS_PER_KT


def ecef(lat, lon):
    la, lo = math.radians(lat), math.radians(lon)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(la) ** 2)
    return (n * math.cos(la) * math.cos(lo), n * math.cos(la) * math.sin(lo),
            n * (1 - WGS84_E2) * math.sin(la))


def test_enu_round_trip():
    enu = geodetic_to_enu(39.6, -104.7, 7200.0, *ORIGIN)
    lat, lon, alt = enu_to_geodetic(enu, *ORIGIN)
    assert (lat, lon, alt) == pytest.approx((39.6, -104.7, 7200.0), abs=1e-9)
    assert enu[2] == pytest.approx(1200 * 0.3048)


@pytest.mark.parametrize("bearing", range(0, 360, 30))
def test_enu_distance_matches_wgs84_within_25_m_at_10_miles(bearing):
    d = 10 * MI
    lat, lon, _ = enu_to_geodetic([d * math.sin(math.radians(bearing)),
                                   d * math.cos(math.radians(bearing)), 0.0], *ORIGIN)
    true = math.dist(ecef(ORIGIN[0], ORIGIN[1]), ecef(lat, lon))  # chord = arc to < 1 cm here
    assert true == pytest.approx(d, abs=25.0)


def test_velocity_and_turn_rate():
    v = velocity_enu(100.0, 90.0, 600.0)
    assert v[0] == pytest.approx(100 * MPS_PER_KT)
    assert v[1] == pytest.approx(0.0, abs=1e-9)
    assert v[2] == pytest.approx(600 * 0.3048 / 60)
    # A 15 degree bank at 100 kt is roughly a 3 deg/s turn.
    assert turn_rate_dps(100.0, 15.0) == pytest.approx(2.93, abs=0.02)


def test_extrapolate_constant_velocity():
    t = at(north_m=0.0, gs_kt=120.0, track_deg=0.0, vs_fpm=500.0, t=0.0)
    later = extrapolate(t, 60.0)
    enu = geodetic_to_enu(later.lat, later.lon, later.alt_ft, t.lat, t.lon, t.alt_ft)
    assert enu[1] == pytest.approx(2 * NM, rel=1e-6)
    assert later.alt_ft == pytest.approx(t.alt_ft + 500.0)
    assert later.timestamp == 60.0


def test_head_on():
    p = predict(own(track_deg=0), at(north_m=3 * MI, track_deg=180))
    assert p.tcpa_s == pytest.approx(3 * MI / (2 * V110), rel=1e-6)
    assert p.hmd_m == pytest.approx(0.0, abs=1e-6)
    assert p.vmd_ft == pytest.approx(0.0, abs=1e-9)
    assert p.bearing_deg == pytest.approx(0.0, abs=1e-9)
    # CPA midpoint is half way between them.
    mid = geodetic_to_enu(p.cpa_lat, p.cpa_lon, p.cpa_alt_ft, *ORIGIN)
    assert mid == pytest.approx([0.0, 1.5 * MI, 0.0], abs=1e-3)


def test_crossing_90_degrees():
    # Both reach the point 1 mile north of ownship at the same instant.
    d = 1 * MI
    p = predict(own(track_deg=0), at(east_m=d, north_m=d, track_deg=270))
    assert p.tcpa_s == pytest.approx(d / V110, rel=1e-6)
    assert p.hmd_m == pytest.approx(0.0, abs=1e-3)
    assert p.bearing_deg == pytest.approx(45.0)


def test_overtaking():
    p = predict(own(gs_kt=110), at(north_m=-1 * MI, gs_kt=160, track_deg=0, alt_ft=6200))
    assert p.tcpa_s == pytest.approx(MI / (50 * MPS_PER_KT), rel=1e-6)
    assert p.hmd_m == pytest.approx(0.0, abs=1e-6)
    assert p.vmd_ft == pytest.approx(200.0)
    assert p.rel_alt_ft == pytest.approx(200.0)


def test_diverging_traffic_has_zero_tcpa_and_current_range_as_miss():
    p = predict(own(gs_kt=110), at(north_m=1 * MI, gs_kt=150, track_deg=0))
    assert p.tcpa_s == 0.0
    assert p.hmd_m == pytest.approx(1 * MI, rel=1e-9)


def test_parallel_pass_miss_distance():
    p = predict(own(track_deg=0), at(east_m=0.8 * NM, north_m=4 * MI, track_deg=180))
    assert p.hmd_nm == pytest.approx(0.8, rel=1e-6)
    assert p.tcpa_s == pytest.approx(4 * MI / (2 * V110), rel=1e-6)


def test_vertical_miss_uses_vertical_rates():
    # Intruder 1000 ft above, descending 1000 ft/min, CPA in 30 s -> 500 ft above at CPA.
    p = predict(own(track_deg=0), at(north_m=2 * V110 * 30, track_deg=180,
                                     alt_ft=7000, vs_fpm=-1000))
    assert p.tcpa_s == pytest.approx(30.0, rel=1e-6)
    assert p.vmd_ft == pytest.approx(500.0, rel=1e-6)

import dataclasses
import math
import random

import pytest

from gatcas.alerting import AlertLevel, classify, is_threat
from gatcas.config import DEFAULT, M_PER_NM, M_PER_STATUTE_MILE
from gatcas.prediction import predict, relative_state
from gatcas.zones import innermost, proximity, shell_states, shells, time_shell_radius_m

from helpers import MI, NM, at, own


def rel_speed(o, t):
    _, own_vel, intr_vel = relative_state(o, t)
    v = intr_vel - own_vel
    return math.hypot(v[0], v[1])


def states_for(o, t, cfg=DEFAULT):
    return {s.key: s for s in shell_states(predict(o, t), rel_speed(o, t), cfg)}


def test_shells_are_built_from_config_outermost_first():
    sh = shells()
    assert [s.key for s in sh] == ["surveillance", "watch", "ta", "ra", "threat", "core"]
    assert [s.rank for s in sh] == [0, 1, 2, 3, 4, 5]
    by = {s.key: s for s in sh}
    assert by["surveillance"].radius_m == pytest.approx(10 * M_PER_STATUTE_MILE)
    assert by["watch"].radius_m == pytest.approx(3 * M_PER_STATUTE_MILE)
    assert by["threat"].radius_m == pytest.approx(0.5 * M_PER_NM) and by["threat"].half_height_ft == 600
    assert by["core"].radius_m == pytest.approx(0.25 * M_PER_NM) and by["core"].half_height_ft == 300
    assert (by["ta"].kind, by["ta"].seconds) == ("time", 50) and (by["ra"].kind, by["ra"].seconds) == ("time", 35)
    # every field a shell names really exists on Config
    names = {f.name for f in dataclasses.fields(DEFAULT)}
    assert all(field in names for s in sh for field in s.source)


def test_shells_follow_a_changed_config():
    cfg = dataclasses.replace(DEFAULT, threat_hmd_nm=1.0, threat_vmd_ft=800.0, ra_tcpa_s=25.0, mandatory_range_mi=5.0)
    by = {s.key: s for s in shells(cfg)}
    assert by["threat"].radius_m == pytest.approx(M_PER_NM) and by["threat"].half_height_ft == 800
    assert by["ra"].seconds == 25 and by["ra"].half_height_ft == 800
    assert by["watch"].radius_m == pytest.approx(5 * M_PER_STATUTE_MILE)


def test_time_ring_radius_for_a_head_on():
    # 110 kt each way = 220 kt closure = 113.18 m/s; 35 s of that is 3961 m (2.46 mi)
    o, t = own(), at(north_m=4.0 * MI, track_deg=180)
    st = states_for(o, t)
    assert st["ra"].radius_m == pytest.approx(220 * 1852 / 3600 * 35, rel=1e-6)
    assert st["ta"].radius_m == pytest.approx(220 * 1852 / 3600 * 50, rel=1e-6)
    assert not st["ra"].inside and not st["ta"].inside      # 4 mi out: 56.9 s to go
    assert innermost(tuple(st.values())) == "surveillance"


def test_time_ring_is_exactly_the_alerting_rule():
    """For any threat, being inside the TA / RA ring by distance is the same
    statement as alerting.classify() returning TA / RA. Random geometries."""
    rng = random.Random(7)
    threats = 0
    for _ in range(10000):
        o = own(gs_kt=rng.uniform(70, 150), track_deg=rng.uniform(0, 360), vs_fpm=rng.uniform(-500, 500))
        t = at(east_m=rng.uniform(-6, 6) * MI, north_m=rng.uniform(-6, 6) * MI,
               alt_ft=6000 + rng.uniform(-700, 700), gs_kt=rng.uniform(70, 150),
               track_deg=rng.uniform(0, 360), vs_fpm=rng.uniform(-500, 500))
        pred = predict(o, t)
        st = {s.key: s for s in shell_states(pred, rel_speed(o, t))}
        level = classify(pred)
        if not is_threat(pred):
            assert not st["ta"].applies and not st["ra"].applies
            assert level is AlertLevel.NONE
            continue
        threats += 1
        assert st["ta"].inside == (level >= AlertLevel.TA)
        assert st["ra"].inside == (level is AlertLevel.RA)
        # the drawn radius agrees with the time test, to within rounding at the edge
        for key in ("ta", "ra"):
            by_distance = pred.range_m <= st[key].radius_m
            if abs(st[key].margin_m) > 0.01:
                assert by_distance == st[key].inside
    assert threats > 200


def test_time_ring_shrinks_with_closing_speed():
    assert time_shell_radius_m(35, 0.0, 113.0) == pytest.approx(3955.0)
    assert time_shell_radius_m(35, 0.0, 30.9) < 0.3 * time_shell_radius_m(35, 0.0, 113.0)   # 60 kt overtake
    assert time_shell_radius_m(35, 400.0, 0.0) == pytest.approx(400.0)                      # no closure: just the miss


def test_boundaries_are_discs_not_spheres():
    o = own()
    # directly overhead by 1,800 ft: inside the threat ring seen from above, outside the volume
    high = states_for(o, at(east_m=0.1 * NM, alt_ft=7800, track_deg=90))
    assert high["watch"].inside and not high["threat"].inside and not high["core"].inside
    assert high["threat"].margin_m < 0                         # horizontally inside the ring
    # same horizontal position, 200 ft above: inside both volumes
    near = states_for(o, at(east_m=0.1 * NM, alt_ft=6200, track_deg=90))
    assert near["threat"].inside and near["core"].inside
    assert innermost(tuple(near.values())) == "core"
    # 0.4 nm away and 400 ft above: threat volume yes, RA target volume no
    mid = states_for(o, at(east_m=0.4 * NM, alt_ft=6400, track_deg=90))
    assert mid["threat"].inside and not mid["core"].inside


def test_proximity_runs_from_zero_at_the_watch_ring_to_one_at_the_same_point():
    o = own()
    assert proximity(predict(o, at(north_m=5 * MI))) == 0.0
    assert proximity(predict(o, at(north_m=3 * MI))) == pytest.approx(0.0, abs=1e-9)
    assert proximity(predict(o, at(north_m=1.5 * MI))) == pytest.approx(0.5, abs=1e-6)
    assert proximity(predict(o, at(north_m=0.0))) == pytest.approx(1.0)
    # 600 ft above at zero range counts the same as level at 0.5 nm
    assert proximity(predict(o, at(alt_ft=6600))) == pytest.approx(proximity(predict(o, at(north_m=0.5 * NM))), abs=1e-6)

from dataclasses import replace

from gatcas.alerting import AlertLevel, AlertTracker, classify, is_threat
from gatcas.config import MPS_PER_KT, Config
from gatcas.prediction import predict
from helpers import MI, NM, at, own

V = 110 * MPS_PER_KT
NONE, TA, RA = AlertLevel.NONE, AlertLevel.TA, AlertLevel.RA


def head_on(tcpa, **kw):
    return predict(own(track_deg=0), at(north_m=2 * V * tcpa, track_deg=180, **kw))


def crossing(tcpa):
    d = V * tcpa  # both reach the same point at the same time
    return predict(own(track_deg=0), at(east_m=-d, north_m=d, track_deg=90))


def overtaking(tcpa):
    closure = 50 * MPS_PER_KT
    return predict(own(gs_kt=110), at(north_m=-closure * tcpa, gs_kt=160, track_deg=0))


def test_head_on_ta_at_50_s_then_ra_at_35_s():
    assert [classify(head_on(t)) for t in (70, 51, 49.9, 40, 35.1, 34.9, 5)] == [NONE, NONE, TA, TA, TA, RA, RA]


def test_crossing_90_degrees_ta_then_ra():
    assert [classify(crossing(t)) for t in (55, 45, 34)] == [NONE, TA, RA]


def test_overtaking_ta_then_ra():
    p = overtaking(40)
    assert is_threat(p)
    assert [classify(overtaking(t)) for t in (60, 48, 30)] == [NONE, TA, RA]


def test_ta_and_ra_times_are_configurable():
    cfg = Config(ta_tcpa_s=40.0, ra_tcpa_s=25.0)
    assert [classify(head_on(t), cfg) for t in (45, 39.9, 30, 24.9)] == [NONE, TA, TA, RA]


def test_diverging_traffic_no_alert():
    for north in (0.6 * NM, 1 * MI, 3 * MI):
        p = predict(own(gs_kt=110), at(north_m=north, gs_kt=150, track_deg=0))
        assert p.tcpa_s == 0.0
        assert classify(p) is NONE
    behind = predict(own(track_deg=0), at(north_m=-1 * MI, track_deg=180))
    assert classify(behind) is NONE


def test_parallel_pass_outside_thresholds_no_alert():
    # Opposite-direction parallel tracks 0.6 nm apart: never inside 0.5 nm.
    for north_mi in (4, 2, 1, 0.3, 0.0):
        p = predict(own(track_deg=0), at(east_m=0.6 * NM, north_m=north_mi * MI, track_deg=180))
        assert classify(p) is NONE
    # Collision course horizontally but 700 ft above: outside the vertical threshold.
    assert classify(head_on(10, alt_ft=6700)) is NONE


def test_threshold_edges_are_strict():
    p = head_on(10)
    assert is_threat(replace(p, hmd_m=0.5 * NM - 0.01))
    assert not is_threat(replace(p, hmd_m=0.5 * NM))
    assert not is_threat(replace(p, vmd_ft=600.0))


def test_ra_is_held_until_tcpa_passed_and_threat_false():
    tracker = AlertTracker()
    p = head_on(20)
    assert tracker.update(p) == (NONE, RA)
    # The maneuver resolves the threat before CPA: RA is still held.
    resolved = replace(p, tcpa_s=8.0, vmd_ft=650.0)
    assert tracker.update(resolved) == (RA, RA)
    # Passed but still inside the threat volume: held.
    assert tracker.update(replace(p, tcpa_s=0.0, hmd_m=0.2 * NM)) == (RA, RA)
    # Passed and outside: RA ends.
    assert tracker.update(replace(p, tcpa_s=0.0, hmd_m=0.6 * NM)) == (RA, NONE)


def test_forget_missing_reports_lost_ras():
    tracker = AlertTracker()
    tracker.update(head_on(20))                    # ABC123 -> RA
    tracker.update(replace(head_on(45), icao="T2"))  # T2 -> TA
    assert tracker.forget_missing(["T2"]) == ["ABC123"]
    assert tracker.level("ABC123") is NONE
    assert tracker.level("T2") is TA

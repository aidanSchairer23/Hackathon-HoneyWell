import math
from dataclasses import replace

import numpy as np
import pytest

from gatcas.config import DEFAULT, MPS_PER_KT, Config
from gatcas.prediction import turn_rate_dps, velocity_enu
from gatcas.resolution import (BASIS_ALTITUDE, BASIS_BEST, BASIS_TURN_GROUND, BASIS_TURN_SHORTFALL,
                               Maneuver, commands_for, ownship_displacement, select_ra,
                               turn_direction, turn_tie_break, vertical_sense)
from helpers import at, own

V = 110 * MPS_PER_KT
CLIMB, DESCEND, LEFT, RIGHT = Maneuver.CLIMB, Maneuver.DESCEND, Maneuver.TURN_LEFT, Maneuver.TURN_RIGHT
# The original selection rule: whichever maneuver predicts the most separation.
# The default is now altitude first; the turn logic is still there and still tested.
BEST = replace(DEFAULT, ra_vertical_first=False)


def head_on(tcpa, own_icao="A00001", intr_icao="B00002", **kw):
    return own(icao=own_icao), at(intr_icao, north_m=2 * V * tcpa, track_deg=180, **kw)


def test_default_limits():
    assert (DEFAULT.ra_climb_fpm, DEFAULT.ra_descent_fpm, DEFAULT.ra_bank_deg) == (500.0, 1000.0, 25.0)
    assert (DEFAULT.ra_success_hmd_nm, DEFAULT.ra_success_vmd_ft) == (0.25, 300.0)
    assert (DEFAULT.threat_hmd_nm, DEFAULT.threat_vmd_ft) == (0.5, 600.0)


def test_commands_respect_separate_climb_and_descent_limits():
    assert commands_for(CLIMB, 0.0) == (500.0, 0)
    assert commands_for(CLIMB, 900.0) == (900.0, 0)       # never reduces an existing climb
    assert commands_for(DESCEND, 200.0) == (-1000.0, 0)
    assert commands_for(DESCEND, -1500.0) == (-1500.0, 0)  # never reduces an existing descent
    assert commands_for(RIGHT, 300.0) == (None, 1)        # turns hold vertical rate
    assert commands_for(LEFT, 0.0) == (None, -1)
    assert commands_for(DESCEND, 0.0, Config(ra_descent_fpm=700.0)) == (-700.0, 0)


def test_vertical_model_accelerates_then_holds_commanded_rate():
    vel = velocity_enu(100, 0, 0)
    t = np.array([0.0, 10.0, 60.0])
    climb_ft = ownship_displacement(CLIMB, vel, t)[:, 2] / 0.3048
    descend_ft = ownship_displacement(DESCEND, vel, t)[:, 2] / 0.3048
    assert climb_ft[0] == 0.0
    assert 500 / 6 - 5 < climb_ft[1] < 500 / 6       # ~83 ft in 10 s, minus the ramp
    assert climb_ft[2] - climb_ft[1] == pytest.approx(500 * 50 / 60, rel=1e-9)
    assert descend_ft[2] - descend_ft[1] == pytest.approx(-1000 * 50 / 60, rel=1e-9)


def test_turn_model_is_a_bank_limited_circle():
    vel = velocity_enu(100, 0, 0)
    half_circle_s = 180.0 / turn_rate_dps(100, DEFAULT.ra_bank_deg)
    right = ownship_displacement(RIGHT, vel, np.array([half_circle_s]))[0]
    left = ownship_displacement(LEFT, vel, np.array([half_circle_s]))[0]
    radius = (100 * MPS_PER_KT) ** 2 / (9.80665 * math.tan(math.radians(DEFAULT.ra_bank_deg)))
    assert right[:2] == pytest.approx([2 * radius, 0.0], abs=1e-6)
    assert left[:2] == pytest.approx([-2 * radius, 0.0], abs=1e-6)


def test_scores_are_relative_to_the_ra_success_target():
    # Co-altitude head-on at RA onset (tCPA 35 s): a 500 fpm climb gives under
    # 300 ft, a 1000 fpm descent gives over 300 ft, a 25 degree turn clears 0.25 nm.
    res = select_ra(*head_on(35), BEST)
    assert res.scores[CLIMB] < 1.0 < res.scores[DESCEND]
    assert res.scores[RIGHT] > 1.0 and res.maneuver is RIGHT and res.basis == BASIS_BEST
    doubled = select_ra(*head_on(35), Config(ra_success_vmd_ft=600.0))
    assert doubled.scores[DESCEND] == pytest.approx(res.scores[DESCEND] / 2, rel=1e-6)


def test_vertical_sense_is_away_from_intruder():
    assert select_ra(*head_on(25, alt_ft=5500)).vertical_sense is CLIMB     # intruder 500 ft below
    assert select_ra(*head_on(25, alt_ft=6400)).vertical_sense is DESCEND   # intruder 400 ft above


def test_unequal_rates_do_not_break_the_vertical_tie():
    # Co-altitude, detected 6 s before CPA. At its own limit descend scores far
    # higher than climb, but the sense is chosen at equal rates, so it is a
    # tie and the ICAO rule decides.
    low = select_ra(*head_on(6, own_icao="A00001", intr_icao="B00002"))
    high = select_ra(*head_on(6, own_icao="B00002", intr_icao="A00001"))
    assert low.scores[DESCEND] - low.scores[CLIMB] > DEFAULT.ra_tie_tolerance
    assert (low.vertical_sense, high.vertical_sense) == (CLIMB, DESCEND)
    assert high.maneuver is DESCEND and high.coordinated
    assert low.maneuver is not DESCEND   # never the same sense as the other aircraft


def test_turn_away_from_laterally_offset_head_on():
    o, _ = head_on(35)
    assert select_ra(o, at("B00002", east_m=100, north_m=2 * V * 35, track_deg=180), BEST).maneuver is LEFT
    assert select_ra(o, at("B00002", east_m=-100, north_m=2 * V * 35, track_deg=180), BEST).maneuver is RIGHT


def test_crossing_turns_away_from_intruder_side():
    d = V * 35
    assert select_ra(own(), at("B00002", east_m=d, north_m=d, track_deg=270), BEST).maneuver is LEFT
    assert select_ra(own(), at("B00002", east_m=-d, north_m=d, track_deg=90), BEST).maneuver is RIGHT


def test_symmetric_head_on_turn_tie_means_turn_right():
    res = select_ra(*head_on(35), BEST)
    assert res.scores[LEFT] == pytest.approx(res.scores[RIGHT], abs=1e-9)
    assert res.maneuver is RIGHT and res.coordinated


def test_vertical_sense_tie_rule():
    o, i = own(icao="a00001"), at("A00002")
    tied = {CLIMB: 0.80, DESCEND: 0.78}
    assert vertical_sense(tied, o, i) == (CLIMB, True)
    assert vertical_sense(tied, i, o) == (DESCEND, True)
    assert vertical_sense(tied, own(icao="~00ff01"), at("100000")) == (CLIMB, True)  # non-ICAO prefix ignored
    assert vertical_sense({CLIMB: 0.2, DESCEND: 0.9}, o, i) == (DESCEND, False)


@pytest.mark.parametrize("own_trk, own_gs, intr_trk, intr_gs, own_icao, intr_icao, expected", [
    (0, 110, 180, 110, "A00001", "A00002", RIGHT),   # opposing: both turn right
    (0, 110, 100, 90, "A00002", "A00001", RIGHT),    # 100 degrees apart counts as opposing
    (0, 100, 0, 160, "A00001", "A00002", LEFT),      # overtaken (slower) turns left
    (0, 160, 0, 100, "A00002", "A00001", RIGHT),     # overtaker (faster) turns right
    (350, 100, 40, 120, "A00001", "A00002", LEFT),   # converging 50 degrees, slower turns left
    (0, 110, 90, 140, "A00001", "A00002", LEFT),     # exactly 90 degrees counts as "within 90"
    (0, 110, 30, 110, "A00001", "A00002", RIGHT),    # equal speed: lower ICAO turns right
    (0, 110, 30, 110, "A00002", "A00001", LEFT),     # equal speed: higher ICAO turns left
    (0, 110.0, 30, 110.9, "A00002", "A00001", LEFT),  # within ra_speed_tie_kt counts as equal
])
def test_turn_tie_break(own_trk, own_gs, intr_trk, intr_gs, own_icao, intr_icao, expected):
    o = own(icao=own_icao, track_deg=own_trk, gs_kt=own_gs)
    i = at(intr_icao, track_deg=intr_trk, gs_kt=intr_gs)
    assert turn_tie_break(o, i) is expected


def test_turn_direction_uses_scores_unless_tied():
    o, i = own(gs_kt=100), at("A00002", gs_kt=160)          # overtaken by a faster aircraft
    assert turn_direction({LEFT: 0.62, RIGHT: 0.60}, o, i) == (LEFT, True)
    assert turn_direction({LEFT: 0.60, RIGHT: 0.62}, i, o) == (RIGHT, True)
    assert turn_direction({LEFT: 0.3, RIGHT: 0.9}, o, i) == (RIGHT, False)
    assert turn_direction({LEFT: 0.9, RIGHT: 0.3}, i, o) == (LEFT, False)


def test_overtake_from_directly_behind_follows_right_of_way():
    # With equal climb and descent limits a turn wins, and the overtake is a turn tie.
    cfg = replace(BEST, ra_descent_fpm=500.0)
    slow, fast = own(gs_kt=100), at("A00002", north_m=-25 * 60 * MPS_PER_KT, gs_kt=160)
    assert select_ra(slow, fast, cfg).maneuver is LEFT
    assert select_ra(fast, slow, cfg).maneuver is RIGHT


# ---- altitude first (the default) ------------------------------------------

def test_altitude_first_is_the_default():
    assert DEFAULT.ra_vertical_first is True
    assert (DEFAULT.ra_turn_fallback_score, DEFAULT.ra_descend_min_agl_ft) == (0.5, 1000.0)


def test_altitude_first_picks_the_vertical_sense_even_when_a_turn_scores_higher():
    # Co-altitude head-on at RA onset. A turn predicts far more separation than
    # either vertical maneuver, and the RA is still a change of altitude.
    low = select_ra(*head_on(35, own_icao="A00001", intr_icao="B00002"))
    high = select_ra(*head_on(35, own_icao="B00002", intr_icao="A00001"))
    assert low.scores[RIGHT] > low.scores[DESCEND] > low.scores[CLIMB]
    assert (low.maneuver, high.maneuver) == (CLIMB, DESCEND)     # complementary, by ICAO address
    assert low.basis == high.basis == BASIS_ALTITUDE
    assert low.coordinated and high.coordinated


@pytest.mark.parametrize("east_m, track_deg", [(100, 180), (-100, 180), (V * 35, 270), (-V * 35, 90)])
def test_altitude_first_never_turns_at_ra_onset(east_m, track_deg):
    # The offset head-ons and crossings that turn under the old rule.
    north = 2 * V * 35 if track_deg == 180 else V * 35
    res = select_ra(own(), at("B00002", east_m=east_m, north_m=north, track_deg=track_deg))
    assert res.maneuver.is_vertical and res.basis == BASIS_ALTITUDE


def test_altitude_first_moves_away_from_the_intruders_altitude():
    assert select_ra(*head_on(35, alt_ft=5800)).maneuver is CLIMB     # intruder 200 ft below
    assert select_ra(*head_on(35, alt_ft=6200)).maneuver is DESCEND   # intruder 200 ft above


def test_turn_is_the_fallback_when_altitude_falls_well_short():
    # Detected 6 s before CPA: a 500 fpm climb gains about 25 ft, far under
    # half the target, and a turn does better. Only then is a turn advised.
    res = select_ra(*head_on(6, own_icao="A00001", intr_icao="B00002"))
    assert res.vertical_sense is CLIMB
    assert res.scores[CLIMB] < DEFAULT.ra_turn_fallback_score
    assert res.scores[RIGHT] > res.scores[CLIMB]
    assert res.maneuver is RIGHT and res.basis == BASIS_TURN_SHORTFALL


def test_turn_fallback_threshold_is_tunable():
    o, i = head_on(35)
    # 1.0: turn whenever altitude alone is predicted to miss the target (the lone climber).
    strict = select_ra(o, i, replace(DEFAULT, ra_turn_fallback_score=1.0))
    assert strict.scores[CLIMB] < 1.0 and strict.maneuver is RIGHT
    # 0.0: never turn because of a shortfall.
    never = select_ra(*head_on(6), replace(DEFAULT, ra_turn_fallback_score=0.0))
    assert never.maneuver is CLIMB and never.basis == BASIS_ALTITUDE


def test_no_turn_when_altitude_is_short_but_still_the_better_choice():
    # Intruder directly above, 120 ft higher, same track and speed. Every
    # option scores 0.4 (under the fallback threshold), no turn does better,
    # so the RA stays vertical.
    res = select_ra(own(), at("B00002", alt_ft=6120.0))
    assert res.scores[DESCEND] < DEFAULT.ra_turn_fallback_score
    assert res.maneuver is DESCEND and res.basis == BASIS_ALTITUDE


@pytest.mark.parametrize("own_icao, intr_icao", [("A00001", "B00002"), ("B00002", "A00001")])
def test_vertical_tie_with_an_altitude_offset_moves_away_not_by_icao(own_icao, intr_icao):
    # Stacked on the same track at the same speed (two aircraft on one final).
    # The scores tie, and the ICAO rule alone would send the lower-address
    # aircraft UP into traffic 150 ft above it.
    below, above = own(icao=own_icao), at(intr_icao, alt_ft=6150.0)
    assert select_ra(below, above).maneuver is DESCEND
    assert select_ra(above, below).maneuver is CLIMB
    assert vertical_sense({CLIMB: 0.5, DESCEND: 0.5}, below, above) == (DESCEND, True)
    # Inside the 15 ft band it is a co-altitude tie and the ICAO rule applies.
    near = at(intr_icao, alt_ft=6010.0)
    assert vertical_sense({CLIMB: 0.5, DESCEND: 0.5}, below, near)[0] is (CLIMB if own_icao < intr_icao else DESCEND)


def test_too_low_to_descend_turns_instead():
    o, i = head_on(35, alt_ft=6200)                    # intruder above: the sense is descend
    assert select_ra(o, i).maneuver is DESCEND
    low = select_ra(o, i, allow_descend=False)
    assert low.vertical_sense is DESCEND and not low.maneuver.is_vertical
    assert low.basis == BASIS_TURN_GROUND
    # A climb is never affected by the floor.
    o, i = head_on(35, alt_ft=5800)
    assert select_ra(o, i, allow_descend=False).maneuver is CLIMB

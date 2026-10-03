"""Two aircraft, each running its own CasSystem against the other, no data link."""

import math
from dataclasses import replace

import pytest

from gatcas.autopilot import SimulatedAutopilot
from gatcas.config import DEFAULT, M_PER_FT, M_PER_NM, MPS_PER_KT
from gatcas.main import CasSystem
from gatcas.prediction import enu_to_geodetic, geodetic_to_enu
from gatcas.resolution import Maneuver
from gatcas.traffic import SimAircraft, SimPilot
from helpers import MI, ORIGIN

CLIMB, DESCEND, LEFT, RIGHT = Maneuver.CLIMB, Maneuver.DESCEND, Maneuver.TURN_LEFT, Maneuver.TURN_RIGHT
V110 = 110 * MPS_PER_KT
# The original selection rule (most predicted separation), under which turns are common.
BEST = replace(DEFAULT, ra_vertical_first=False)


def aircraft(icao, east_m, north_m, alt_ft, gs_kt, track_deg):
    lat, lon, _ = enu_to_geodetic([east_m, north_m, 0.0], *ORIGIN)
    return SimAircraft(icao, lat, lon, alt_ft, gs_kt, track_deg, 0.0)


def fly_pair(make, equipped=(True, True), comply_delay_s=None, duration_s=100, cfg=DEFAULT):
    """Fly two aircraft against each other. Pilots ignore RAs (so the autopilot
    flies them) unless comply_delay_s is given. Returns each aircraft's RAs and
    the horizontal (m) / vertical (ft) separation at the actual CPA."""
    ras, cpa, _ = fly_pair_full(make, equipped, comply_delay_s, duration_s, cfg)
    return ras, cpa


def fly_pair_full(make, equipped=(True, True), comply_delay_s=None, duration_s=100, cfg=DEFAULT):
    """fly_pair, plus the smallest altitude difference (ft) seen while the two
    were inside the RA success radius of each other. That is the number that
    matters when the aircraft are stacked and there is no single moment of CPA."""
    a, b = make()
    min_vertical = math.inf
    systems = [CasSystem(cfg, SimulatedAutopilot(x, cfg)) if eq else None
               for x, eq in zip((a, b), equipped)]
    pilots = [SimPilot("ignore") if comply_delay_s is None else SimPilot("comply", comply_delay_s)
              for _ in range(2)]
    ras = ([], [])
    cpa = (math.inf, 0.0)
    for k in range(duration_s):
        now = float(k)
        for i, (me, other) in enumerate(((a, b), (b, a))):
            if systems[i] is None:
                continue
            report = systems[i].cycle(now, me.as_target(now), [other.as_target(now)])
            for e in report.events:
                if e.kind == "RA":
                    ras[i].append(e.maneuver)
                    pilots[i].on_ra(e.maneuver, now)
        for step in range(10):
            pilots[0].update(a, now + step / 10, cfg)
            pilots[1].update(b, now + step / 10, cfg)
            a.step(0.1, cfg)
            b.step(0.1, cfg)
            d = geodetic_to_enu(b.lat, b.lon, b.alt_ft, a.lat, a.lon, a.alt_ft)
            if math.hypot(d[0], d[1]) < cpa[0]:
                cpa = (math.hypot(d[0], d[1]), abs(d[2]) / M_PER_FT)
            if math.hypot(d[0], d[1]) < cfg.ra_success_hmd_m:
                min_vertical = min(min_vertical, abs(d[2]) / M_PER_FT)
    return ras, cpa, min_vertical


def vs_target(cpa):
    """Separation at CPA as a fraction of the RA success target (>= 1 means met)."""
    return max(cpa[0] / DEFAULT.ra_success_hmd_m, cpa[1] / DEFAULT.ra_success_vmd_ft)


def no_shared_vertical_sense(ra_a, ra_b):
    return not (ra_a == ra_b and ra_a[0].is_vertical)


def head_on():
    return (aircraft("A00001", 0, 0, 7500, 110, 0),
            aircraft("A00002", 0, 4.5 * MI, 7500, 110, 180))


def crossing():
    d = V110 * 60
    return (aircraft("A00001", 0, 0, 7500, 110, 0),
            aircraft("A00002", d, d, 7500, 110, 270))


def test_head_on_both_turn_right_and_separation_improves():
    (ras_a, ras_b), both = fly_pair(head_on, cfg=BEST)
    assert ras_a == [RIGHT] and ras_b == [RIGHT]
    _, one = fly_pair(head_on, equipped=(True, False), cfg=BEST)
    _, none = fly_pair(head_on, equipped=(False, False), cfg=BEST)
    assert none[0] < 0.01 * M_PER_NM
    assert vs_target(one) >= 1.0
    assert both[0] > 1.25 * one[0]   # the two turns add up rather than cancel


def test_crossing_turns_are_complementary():
    (ras_a, ras_b), both = fly_pair(crossing, cfg=BEST)
    assert ras_a == [LEFT] and ras_b == [RIGHT]   # each turns away from the other
    _, one = fly_pair(crossing, equipped=(True, False), cfg=BEST)
    assert vs_target(both) >= 1.0
    assert both[0] > 1.1 * one[0]


@pytest.mark.parametrize("overtaker_has_lower_icao", [False, True])
def test_co_altitude_overtake_is_complementary(overtaker_has_lower_icao):
    """Previously both aircraft turned right (0.01 nm), then both descended (0 ft)."""
    slow_icao, fast_icao = ("B00009", "A00002") if overtaker_has_lower_icao else ("A00001", "A00002")

    def overtake():
        return (aircraft(slow_icao, 0, 0, 7500, 100, 0),
                aircraft(fast_icao, 0, -1.1 * MI, 7500, 160, 0))

    (ras_slow, ras_fast), both = fly_pair(overtake, cfg=BEST)
    assert len(ras_slow) == len(ras_fast) == 1
    assert no_shared_vertical_sense(ras_slow, ras_fast)
    # Whichever aircraft turns follows right of way: the slower left, the faster right.
    assert ras_slow[0] in (LEFT, CLIMB, DESCEND) and ras_fast[0] in (RIGHT, CLIMB, DESCEND)
    assert LEFT in ras_slow or RIGHT in ras_fast
    assert vs_target(both) >= 1.0


def test_overtake_turn_tie_end_to_end_slower_left_faster_right():
    # Equal climb and descent limits, so both aircraft turn and the turn tie rule decides.
    cfg = replace(BEST, ra_descent_fpm=500.0)

    def overtake():
        return (aircraft("A00001", 0, 0, 7500, 100, 0),
                aircraft("A00002", 0, -1.1 * MI, 7500, 160, 0))

    (ras_slow, ras_fast), both = fly_pair(overtake, cfg=cfg)
    assert (ras_slow, ras_fast) == ([LEFT], [RIGHT])
    _, one = fly_pair(overtake, equipped=(True, False), cfg=cfg)
    assert vs_target(both) >= 1.0
    assert both[0] > 1.25 * one[0]


@pytest.mark.parametrize("detected_s", [4, 6, 8, 10])
@pytest.mark.parametrize("low_icao_is_a", [True, False])
def test_late_detection_choices_are_complementary(detected_s, low_icao_is_a):
    # Co-altitude head-on detected only seconds before CPA. No maneuver can
    # reach the RA success target this late (see README), so this checks only
    # that the two RAs are complementary.
    icao_a, icao_b = ("A00001", "B00002") if low_icao_is_a else ("B00002", "A00001")

    def late_head_on():
        return (aircraft(icao_a, 0, 0, 7500, 110, 0),
                aircraft(icao_b, 0, 2 * V110 * detected_s, 7500, 110, 180))

    (ras_a, ras_b), both = fly_pair(late_head_on, comply_delay_s=0.0, duration_s=30)
    assert len(ras_a) == len(ras_b) == 1
    assert no_shared_vertical_sense(ras_a, ras_b)
    low, high = (ras_a[0], ras_b[0]) if low_icao_is_a else (ras_b[0], ras_a[0])
    assert low in (CLIMB, RIGHT) and high in (DESCEND, RIGHT)   # ICAO rule; head-on turns go right
    _, one = fly_pair(late_head_on, equipped=(True, False), comply_delay_s=0.0, duration_s=30)
    assert vs_target(both) >= 0.95 * vs_target(one)   # the two RAs do not cancel


def test_vertical_geometry_beats_icao_order_when_not_tied():
    # Lower aircraft has the LOWER address, so the tie rule alone would make it
    # climb. With a 100 ft offset there is no tie: lower descends, upper climbs.
    def offset():
        return (aircraft("A00001", 0, 0, 7500, 110, 0),
                aircraft("A00002", 0, 2 * V110 * 8, 7600, 110, 180))

    (ras_a, ras_b), both = fly_pair(offset, comply_delay_s=0.0, duration_s=30)
    assert (ras_a, ras_b) == ([DESCEND], [CLIMB])
    _, one = fly_pair(offset, equipped=(True, False), comply_delay_s=0.0, duration_s=30)
    assert both[1] > one[1] > 100


# ---- altitude first (the default) ------------------------------------------

def overtake():
    return (aircraft("A00001", 0, 0, 7500, 100, 0),
            aircraft("A00002", 0, -1.1 * MI, 7500, 160, 0))


@pytest.mark.parametrize("make", [head_on, crossing, overtake])
def test_altitude_first_both_equipped_one_climbs_one_descends(make):
    # Co-altitude, no data link: the lower ICAO address climbs, the higher descends.
    (ras_a, ras_b), both = fly_pair(make)
    assert (ras_a, ras_b) == ([CLIMB], [DESCEND])
    assert both[1] >= 700.0                      # 713 to 803 ft measured; target is 300 ft
    assert both[1] >= DEFAULT.threat_vmd_ft      # also clear of the threat volume


@pytest.mark.parametrize("make", [head_on, crossing, overtake])
def test_altitude_first_lone_descender_meets_the_target(make):
    # Only the higher-address aircraft is equipped, so it descends at 1000 fpm alone.
    (_, ras_b), cpa = fly_pair(make, equipped=(False, True))
    assert ras_b == [DESCEND]
    assert cpa[1] >= DEFAULT.ra_success_vmd_ft   # 483 to 533 ft measured


@pytest.mark.parametrize("make", [head_on, crossing, overtake])
def test_altitude_first_lone_climber_falls_short_of_the_target(make):
    """KNOWN SHORTFALL, kept as a test so it cannot be forgotten.

    Only the lower-address aircraft is equipped, so it climbs at 500 fpm alone
    after the 5 s autopilot handoff. That gives 241 to 271 ft at CPA, under the
    300 ft RA success target. See the README for the two one-line fixes.
    """
    (ras_a, _), cpa = fly_pair(make, equipped=(True, False))
    assert ras_a == [CLIMB]
    assert 230.0 <= cpa[1] < DEFAULT.ra_success_vmd_ft
    assert cpa[0] < DEFAULT.ra_success_hmd_m     # and nothing horizontal makes up for it


@pytest.mark.parametrize("fix", [
    {"ta_tcpa_s": 58.0, "ra_tcpa_s": 43.0},      # advise 8 s earlier
    {"ra_climb_fpm": 700.0},                     # or climb harder
])
@pytest.mark.parametrize("make", [head_on, crossing, overtake])
def test_either_fix_brings_the_lone_climber_to_the_target(make, fix):
    cfg = replace(DEFAULT, **fix)
    (ras_a, _), cpa = fly_pair(make, equipped=(True, False), cfg=cfg)
    assert ras_a == [CLIMB]
    assert cpa[1] >= DEFAULT.ra_success_vmd_ft


def test_altitude_first_with_an_altitude_offset_ignores_icao_order():
    # 200 ft apart: the upper aircraft climbs and the lower descends whatever their addresses.
    def offset():
        return (aircraft("A00001", 0, 0, 7500, 110, 0),
                aircraft("A00002", 0, 4.5 * MI, 7700, 110, 180))

    (ras_a, ras_b), both = fly_pair(offset)
    assert (ras_a, ras_b) == ([DESCEND], [CLIMB])
    assert both[1] >= 700.0


@pytest.mark.parametrize("upper_has_lower_icao", [False, True])
def test_stacked_on_the_same_final_the_upper_climbs_and_the_lower_descends(upper_has_lower_icao):
    """One aircraft descending onto another on the same track at the same
    speed. Horizontal distance never changes, so every maneuver scores the
    same and the sense is a tie. The ICAO rule alone would send one of the two
    toward the other half the time; the altitude offset decides instead."""
    upper_icao, lower_icao = ("A00001", "A00002") if upper_has_lower_icao else ("A00002", "A00001")

    def stacked():
        lower = aircraft(lower_icao, 0, 0, 7500, 80, 0)
        upper = aircraft(upper_icao, 0, 0, 7750, 80, 0)
        upper.vs_fpm = -500.0
        return lower, upper

    _, _, untouched = fly_pair_full(stacked, equipped=(False, False), duration_s=60)
    assert untouched < 5.0                                  # they meet 30 s in
    (ras_lower, ras_upper), _, both = fly_pair_full(stacked, duration_s=60)
    assert (ras_lower[0], ras_upper[0]) == (DESCEND, CLIMB)
    assert both > 190.0                                     # never closer than at the handoff
    for equipped in ((True, False), (False, True)):
        _, _, one = fly_pair_full(stacked, equipped=equipped, duration_s=60)
        assert one > 190.0

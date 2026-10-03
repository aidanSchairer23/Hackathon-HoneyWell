"""Resolution advisory selection. Pure functions, no I/O.

Each candidate maneuver (climb, descend, turn left, turn right) is flown
forward over [0, tCPA + ra_eval_margin_s] with a kinematic model of ownship
while the intruder holds constant velocity. CPA is the sampled time of minimum
horizontal distance on those trajectories, and the candidate is scored by its
separation there, normalized by the RA success target (which is separate
from, and smaller than, the threat definition):

    score = max(horizontal_miss / ra_success_hmd, vertical_miss / ra_success_vmd)

A score of 1.0 or more means the RA is predicted to succeed.

Selection:
  1. Vertical sense. Climb and descend are compared at the SAME rate (the lower
     of ra_climb_fpm and ra_descent_fpm), so unequal climb and descent limits
     cannot make both aircraft in a symmetric encounter pick the same sense.
  2. Turn direction, from the turn scores.
  3. Which of the two is flown depends on ra_vertical_first:
     * True (default), altitude first: the chosen sense is flown. A turn is
       the last resort, used only when
         - the sense is "descend" but ownship is too close to the ground to
           descend (climbing instead could put it into the other aircraft's
           climb, so it turns), or
         - the sense is predicted to reach less than ra_turn_fallback_score of
           the RA success target and the turn is predicted to do better.
     * False, most separation: the chosen sense, scored at its own rate limit,
       is compared with the chosen turn and the higher score is flown (an
       exact tie goes to vertical).

Ties (within ra_tie_tolerance) are broken by the coordination rule so two
aircraft running this logic pick complementary maneuvers without a data link:
  * vertical tie, aircraft at different altitudes now (by more than
    ra_tie_tolerance x ra_success_vmd_ft, 15 ft by default): each moves away
    from the other's altitude, so neither crosses it;
  * vertical tie, co-altitude: the lower ICAO address climbs, the higher descends;
  * turn tie, opposing tracks (more than 90 degrees apart): both turn right;
  * turn tie, tracks within 90 degrees (overtaking or converging), following
    right-of-way practice: the faster aircraft turns right and the slower turns
    left; if ground speeds are equal (within ra_speed_tie_kt), the lower ICAO
    address turns right and the higher turns left.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

from .config import DEFAULT, G_MPS2, M_PER_FT, Config
from .prediction import relative_state, time_to_cpa, wrap180

if TYPE_CHECKING:
    from .traffic import Target


class Maneuver(Enum):
    CLIMB = "climb"
    DESCEND = "descend"
    TURN_LEFT = "turn left"
    TURN_RIGHT = "turn right"

    @property
    def is_vertical(self) -> bool:
        return self in (Maneuver.CLIMB, Maneuver.DESCEND)


def commands_for(maneuver: Maneuver, vs_fpm: float, cfg: Config = DEFAULT) -> tuple[float | None, int]:
    """(commanded vertical rate in ft/min or None to hold, turn direction) for a maneuver.

    A climb never reduces an existing climb rate and a descend never reduces an
    existing descent rate. A turn holds the current vertical rate.
    """
    if maneuver is Maneuver.CLIMB:
        return max(vs_fpm, cfg.ra_climb_fpm), 0
    if maneuver is Maneuver.DESCEND:
        return min(vs_fpm, -cfg.ra_descent_fpm), 0
    return None, (1 if maneuver is Maneuver.TURN_RIGHT else -1)


def ownship_displacement(maneuver: Maneuver, own_vel: np.ndarray, times: np.ndarray,
                         cfg: Config = DEFAULT) -> np.ndarray:
    """Ownship ENU displacement in meters at each time while flying a maneuver.

    Vertical maneuvers accelerate at vertical_accel_g to the commanded rate.
    Turns roll in instantly to ra_bank_deg and turn at the coordinated rate.
    """
    t = np.asarray(times, dtype=float)
    ve, vn, vu = (float(x) for x in own_vel)
    cmd_vs_fpm, turn = commands_for(maneuver, vu / M_PER_FT * 60.0, cfg)

    if maneuver.is_vertical:
        target = cmd_vs_fpm * M_PER_FT / 60.0
        accel = cfg.vertical_accel_g * G_MPS2
        dv = target - vu
        t_ramp = abs(dv) / accel
        tr = np.minimum(t, t_ramp)
        up = vu * tr + 0.5 * math.copysign(accel, dv) * tr**2 + target * (t - tr)
        return np.column_stack([ve * t, vn * t, up])

    speed = math.hypot(ve, vn)
    omega = G_MPS2 * math.tan(math.radians(cfg.ra_bank_deg)) / speed if speed >= 1.0 else 0.0
    if omega == 0.0:
        east, north = ve * t, vn * t
    else:
        psi0 = math.atan2(ve, vn)
        psi = psi0 + turn * omega * t
        radius = speed / (turn * omega)
        east = radius * (math.cos(psi0) - np.cos(psi))
        north = radius * (np.sin(psi) - math.sin(psi0))
    return np.column_stack([east, north, vu * t])


def predicted_separation(maneuver: Maneuver, rel_pos: np.ndarray, own_vel: np.ndarray,
                         intr_vel: np.ndarray, horizon_s: float, cfg: Config = DEFAULT) -> float:
    """Separation at CPA, as a fraction of the RA success target, if ownship flies `maneuver`."""
    times = np.arange(0.0, horizon_s + cfg.ra_eval_step_s / 2, cfg.ra_eval_step_s)
    own = ownship_displacement(maneuver, own_vel, times, cfg)
    intr = rel_pos + np.outer(times, intr_vel)
    d = intr - own
    h = np.hypot(d[:, 0], d[:, 1])
    k = int(np.argmin(h))
    return max(h[k] / cfg.ra_success_hmd_m, abs(d[k, 2]) / cfg.ra_success_vmd_m)


def _icao_key(icao: str) -> str:
    """Normalize so string order equals numeric order of the 24-bit address."""
    return icao.lstrip("~").upper().zfill(6)


def turn_tie_break(own: Target, intruder: Target, cfg: Config = DEFAULT) -> Maneuver:
    """Which way to turn when turning left and right score the same."""
    if abs(wrap180(own.track_deg - intruder.track_deg)) > 90.0:
        return Maneuver.TURN_RIGHT  # opposing tracks: both turn right
    if abs(own.gs_kt - intruder.gs_kt) > cfg.ra_speed_tie_kt:
        faster = own.gs_kt > intruder.gs_kt
    else:
        faster = _icao_key(own.icao) < _icao_key(intruder.icao)  # lower address acts as "faster"
    return Maneuver.TURN_RIGHT if faster else Maneuver.TURN_LEFT


def vertical_sense(sense_scores: dict[Maneuver, float], own: Target, intruder: Target,
                   cfg: Config = DEFAULT) -> tuple[Maneuver, bool]:
    """Climb or descend, from scores computed at equal rates. Returns (sense, tie)."""
    climb, descend = sense_scores[Maneuver.CLIMB], sense_scores[Maneuver.DESCEND]
    if abs(climb - descend) <= cfg.ra_tie_tolerance:
        # Same predicted result either way. If the aircraft are at different
        # altitudes now, do not cross the other one's altitude: move away from
        # it. (Both aircraft see the same offset with opposite signs, so the
        # two choices are complementary.) Only a co-altitude tie goes to the
        # ICAO rule. Without this, an aircraft directly below another on the
        # same track and speed could be told to climb into it.
        offset_ft = intruder.alt_ft - own.alt_ft
        if abs(offset_ft) > cfg.ra_tie_tolerance * cfg.ra_success_vmd_ft:
            return (Maneuver.DESCEND if offset_ft > 0 else Maneuver.CLIMB), True
        lower = _icao_key(own.icao) < _icao_key(intruder.icao)
        return (Maneuver.CLIMB if lower else Maneuver.DESCEND), True
    return (Maneuver.CLIMB if climb > descend else Maneuver.DESCEND), False


def turn_direction(scores: dict[Maneuver, float], own: Target, intruder: Target,
                   cfg: Config = DEFAULT) -> tuple[Maneuver, bool]:
    """Turn left or right. Returns (direction, tie)."""
    left, right = scores[Maneuver.TURN_LEFT], scores[Maneuver.TURN_RIGHT]
    if abs(left - right) <= cfg.ra_tie_tolerance:
        return turn_tie_break(own, intruder, cfg), True
    return (Maneuver.TURN_RIGHT if right > left else Maneuver.TURN_LEFT), False


@dataclass(frozen=True)
class Resolution:
    maneuver: Maneuver
    scores: dict[Maneuver, float]  # each option at its own rate limit
    vertical_sense: Maneuver       # sense chosen at equal rates, flown or not
    coordinated: bool              # True when the flown maneuver came from a tie rule
    basis: str = ""                # why this kind of maneuver: see BASIS_* below


BASIS_ALTITUDE = "altitude first"
BASIS_TURN_SHORTFALL = "turn: altitude change predicted to fall short"
BASIS_TURN_GROUND = "turn: too low to descend"
BASIS_BEST = "most predicted separation"


def select_ra(own: Target, intruder: Target, cfg: Config = DEFAULT,
              allow_descend: bool = True) -> Resolution:
    """Select one RA against one threat.

    allow_descend is False when ownship is too close to the ground for a
    descend RA (see ra_descend_min_agl_ft).
    """
    rel_pos, own_vel, intr_vel = relative_state(own, intruder)
    horizon = time_to_cpa(rel_pos, intr_vel - own_vel) + cfg.ra_eval_margin_s
    scores = {m: predicted_separation(m, rel_pos, own_vel, intr_vel, horizon, cfg) for m in Maneuver}

    rate = min(cfg.ra_climb_fpm, cfg.ra_descent_fpm)
    equal = replace(cfg, ra_climb_fpm=rate, ra_descent_fpm=rate)
    sense_scores = {m: predicted_separation(m, rel_pos, own_vel, intr_vel, horizon, equal)
                    for m in (Maneuver.CLIMB, Maneuver.DESCEND)}

    sense, v_tie = vertical_sense(sense_scores, own, intruder, cfg)
    turn, t_tie = turn_direction(scores, own, intruder, cfg)

    if sense is Maneuver.DESCEND and not allow_descend:
        # The coordination rule gave us "descend" and we cannot. Climbing
        # instead could match the other aircraft's climb, so turn.
        return Resolution(turn, scores, sense, t_tie, BASIS_TURN_GROUND)
    if cfg.ra_vertical_first:
        if scores[sense] >= cfg.ra_turn_fallback_score or scores[sense] >= scores[turn]:
            return Resolution(sense, scores, sense, v_tie, BASIS_ALTITUDE)
        return Resolution(turn, scores, sense, t_tie, BASIS_TURN_SHORTFALL)
    if scores[sense] >= scores[turn]:
        return Resolution(sense, scores, sense, v_tie, BASIS_BEST)
    return Resolution(turn, scores, sense, t_tie, BASIS_BEST)

"""Pure geometry and conflict prediction. No I/O.

Frame: a local east-north-up (ENU) tangent plane centered on ownship. East and
north come from a local projection using WGS-84 radii of curvature at
ownship's latitude. Up is the altitude difference from ownship, which ignores
earth curvature on purpose: aircraft hold pressure altitude, not a tangent
plane, and TCAS-style logic compares altitudes directly.

Closest point of approach (CPA) is the time of minimum horizontal distance with
both aircraft at constant velocity. Vertical miss is the altitude difference
at that time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import numpy as np

from .config import G_MPS2, M_PER_FT, M_PER_NM, M_PER_STATUTE_MILE, MPS_PER_KT

if TYPE_CHECKING:
    from .traffic import Target

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def _radii_of_curvature(lat_deg: float) -> tuple[float, float]:
    """Meridian (north-south) and prime-vertical (east-west) radii, in meters."""
    s = math.sin(math.radians(lat_deg))
    w = math.sqrt(1.0 - WGS84_E2 * s * s)
    return WGS84_A * (1.0 - WGS84_E2) / w**3, WGS84_A / w


def wrap180(deg: float) -> float:
    """Wrap an angle to the interval [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def geodetic_to_enu(lat: float, lon: float, alt_ft: float,
                    ref_lat: float, ref_lon: float, ref_alt_ft: float) -> np.ndarray:
    """Position of (lat, lon, alt_ft) in meters in the ENU frame at the reference point."""
    m_radius, n_radius = _radii_of_curvature(ref_lat)
    east = math.radians(wrap180(lon - ref_lon)) * n_radius * math.cos(math.radians(ref_lat))
    north = math.radians(lat - ref_lat) * m_radius
    up = (alt_ft - ref_alt_ft) * M_PER_FT
    return np.array([east, north, up])


def enu_to_geodetic(enu: np.ndarray, ref_lat: float, ref_lon: float,
                    ref_alt_ft: float) -> tuple[float, float, float]:
    """Inverse of geodetic_to_enu: returns (lat, lon, alt_ft)."""
    m_radius, n_radius = _radii_of_curvature(ref_lat)
    lat = ref_lat + math.degrees(enu[1] / m_radius)
    lon = ref_lon + math.degrees(enu[0] / (n_radius * math.cos(math.radians(ref_lat))))
    return lat, wrap180(lon), ref_alt_ft + enu[2] / M_PER_FT


def velocity_enu(gs_kt: float, track_deg: float, vs_fpm: float) -> np.ndarray:
    """Velocity in m/s as (east, north, up)."""
    speed = gs_kt * MPS_PER_KT
    trk = math.radians(track_deg)
    return np.array([speed * math.sin(trk), speed * math.cos(trk), vs_fpm * M_PER_FT / 60.0])


def turn_rate_dps(gs_kt: float, bank_deg: float) -> float:
    """Coordinated-turn rate in deg/s for a given ground speed and bank angle.

    Ground speed stands in for true airspeed (no wind model).
    """
    speed = gs_kt * MPS_PER_KT
    if speed < 1.0:
        return 0.0
    return math.degrees(G_MPS2 * math.tan(math.radians(bank_deg)) / speed)


def bearing_deg(enu: np.ndarray) -> float:
    """True bearing in [0, 360) of an ENU vector."""
    return math.degrees(math.atan2(enu[0], enu[1])) % 360.0


def time_to_cpa(rel_pos: np.ndarray, rel_vel: np.ndarray) -> float:
    """Time to horizontal closest point of approach, clamped to >= 0.

    rel_pos and rel_vel are intruder-minus-ownship; only east and north are used.
    """
    p = rel_pos[:2]
    v = rel_vel[:2]
    v2 = float(v @ v)
    if v2 < 1e-9:
        return 0.0
    return max(0.0, -float(p @ v) / v2)


def extrapolate(target: Target, now: float) -> Target:
    """Dead-reckon a target at constant velocity to time `now`."""
    dt = now - target.timestamp
    if dt == 0.0:
        return target
    move = velocity_enu(target.gs_kt, target.track_deg, target.vs_fpm) * dt
    lat, lon, alt = enu_to_geodetic(move, target.lat, target.lon, target.alt_ft)
    return replace(target, lat=lat, lon=lon, alt_ft=alt, timestamp=now)


@dataclass(frozen=True)
class Prediction:
    icao: str
    range_m: float        # current horizontal range
    bearing_deg: float    # true bearing from ownship to the intruder
    rel_alt_ft: float     # intruder altitude minus ownship altitude (+ = above)
    tcpa_s: float         # time to CPA, >= 0
    hmd_m: float          # horizontal miss distance at CPA
    vmd_ft: float         # vertical miss distance at CPA (absolute)
    cpa_lat: float        # midpoint of the two aircraft at CPA
    cpa_lon: float
    cpa_alt_ft: float

    @property
    def range_mi(self) -> float:
        return self.range_m / M_PER_STATUTE_MILE

    @property
    def hmd_nm(self) -> float:
        return self.hmd_m / M_PER_NM


def relative_state(own: Target, intruder: Target) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Intruder position relative to ownship, plus both velocities, in the ownship ENU frame."""
    rel_pos = geodetic_to_enu(intruder.lat, intruder.lon, intruder.alt_ft,
                              own.lat, own.lon, own.alt_ft)
    own_vel = velocity_enu(own.gs_kt, own.track_deg, own.vs_fpm)
    intr_vel = velocity_enu(intruder.gs_kt, intruder.track_deg, intruder.vs_fpm)
    return rel_pos, own_vel, intr_vel


def predict(own: Target, intruder: Target) -> Prediction:
    """Constant-velocity CPA prediction. Both targets must describe the same instant."""
    rel_pos, own_vel, intr_vel = relative_state(own, intruder)
    rel_vel = intr_vel - own_vel
    tcpa = time_to_cpa(rel_pos, rel_vel)
    miss = rel_pos + rel_vel * tcpa
    own_cpa = own_vel * tcpa
    intr_cpa = rel_pos + intr_vel * tcpa
    cpa_lat, cpa_lon, cpa_alt = enu_to_geodetic((own_cpa + intr_cpa) / 2.0,
                                                own.lat, own.lon, own.alt_ft)
    return Prediction(
        icao=intruder.icao,
        range_m=float(math.hypot(rel_pos[0], rel_pos[1])),
        bearing_deg=bearing_deg(rel_pos),
        rel_alt_ft=intruder.alt_ft - own.alt_ft,
        tcpa_s=tcpa,
        hmd_m=float(math.hypot(miss[0], miss[1])),
        vmd_ft=abs(float(miss[2])) / M_PER_FT,
        cpa_lat=cpa_lat,
        cpa_lon=cpa_lon,
        cpa_alt_ft=cpa_alt,
    )

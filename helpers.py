"""Builders for targets placed relative to a fixed origin."""

from gatcas.config import M_PER_NM, M_PER_STATUTE_MILE
from gatcas.prediction import enu_to_geodetic
from gatcas.traffic import Target

ORIGIN = (39.5, -104.8, 6000.0)
MI = M_PER_STATUTE_MILE
NM = M_PER_NM


def at(icao="ABC123", east_m=0.0, north_m=0.0, alt_ft=6000.0, gs_kt=110.0,
       track_deg=0.0, vs_fpm=0.0, t=0.0) -> Target:
    lat, lon, _ = enu_to_geodetic([east_m, north_m, 0.0], *ORIGIN)
    return Target(icao, lat, lon, alt_ft, gs_kt, track_deg, vs_fpm, t)


def own(**kw) -> Target:
    kw.setdefault("icao", "A00001")
    return at(**kw)

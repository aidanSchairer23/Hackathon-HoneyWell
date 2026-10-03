"""Every tunable threshold and parameter for GATCAS lives here.

Values marked TUNABLE PLACEHOLDER are starting points chosen for a hackathon
prototype. They are NOT validated, certified, or derived from flight test.

Unit convention: ranges are in statute miles in this file and are converted to
meters internally; altitude is in feet, speed in knots, vertical rate in ft/min.
The exceptions are the horizontal-miss threshold and the horizontal RA success
target, which are specified in nautical miles and kept in nautical miles here.
"""

from dataclasses import dataclass

# Exact unit conversions. These are definitions, not tunables.
M_PER_STATUTE_MILE = 1609.344
M_PER_NM = 1852.0
M_PER_FT = 0.3048
MPS_PER_KT = 1852.0 / 3600.0
G_MPS2 = 9.80665


@dataclass(frozen=True)
class Config:
    # --- Surveillance (surveillance.py, traffic.py) ---------------------------
    # TUNABLE PLACEHOLDER: track every target inside this horizontal range.
    surveillance_range_mi: float = 10.0
    # TUNABLE PLACEHOLDER: every target inside this range is tracked and
    # evaluated on every cycle, coasting on its last report if needed.
    mandatory_range_mi: float = 3.0
    # TUNABLE PLACEHOLDER: drop a target that has had no update for this long.
    stale_target_s: float = 10.0
    # TUNABLE PLACEHOLDER: processing cycle period (1 Hz).
    cycle_period_s: float = 1.0

    # --- Threat detection (alerting.py) ---------------------------------------
    # TUNABLE PLACEHOLDER: a threat has predicted horizontal miss below this...
    threat_hmd_nm: float = 0.5
    # TUNABLE PLACEHOLDER: ...AND predicted vertical miss below this.
    threat_vmd_ft: float = 600.0
    # TUNABLE PLACEHOLDER: traffic advisory when a threat's tCPA is at or below this.
    ta_tcpa_s: float = 50.0
    # TUNABLE PLACEHOLDER: resolution advisory when a threat's tCPA is at or below this.
    ra_tcpa_s: float = 35.0

    # --- RA success target (resolution.py) ------------------------------------
    # Separate from, and smaller than, the threat definition above: the threat
    # thresholds decide WHEN to alert; these decide what an RA must achieve.
    # TUNABLE PLACEHOLDER: an RA succeeds if predicted horizontal miss at CPA reaches this...
    ra_success_hmd_nm: float = 0.25
    # TUNABLE PLACEHOLDER: ...OR predicted vertical miss at CPA reaches this.
    ra_success_vmd_ft: float = 300.0

    # --- Resolution (resolution.py) and simulated aircraft dynamics -----------
    # TUNABLE PLACEHOLDER: climb rate a climb RA commands (light piston).
    ra_climb_fpm: float = 500.0
    # TUNABLE PLACEHOLDER: descent rate a descend RA commands (light piston).
    ra_descent_fpm: float = 1000.0
    # TUNABLE PLACEHOLDER: bank angle a turn RA commands (light piston).
    ra_bank_deg: float = 25.0
    # TUNABLE PLACEHOLDER: vertical acceleration used to reach the commanded
    # rate (0.25 g is the TCAS II design assumption).
    vertical_accel_g: float = 0.25
    # TUNABLE PLACEHOLDER: each RA option is evaluated from now to tCPA + this.
    ra_eval_margin_s: float = 10.0
    # TUNABLE PLACEHOLDER: time step used when sampling an RA option's trajectory.
    ra_eval_step_s: float = 0.5
    # TUNABLE PLACEHOLDER: two options whose normalized separations differ by
    # no more than this count as a tie, and the coordination rule decides.
    ra_tie_tolerance: float = 0.05
    # TUNABLE PLACEHOLDER: ground speeds within this many knots count as equal
    # in the turn tie rule (ADS-B velocity resolution is about 1 kt).
    ra_speed_tie_kt: float = 1.0

    # --- RA priority (resolution.py) --------------------------------------------
    # TUNABLE PLACEHOLDER: True = altitude first. The RA is the chosen climb or
    # descend sense, and a turn is used only as a last resort (see the next
    # value and ra_descend_min_agl_ft). False = fly whichever option is
    # predicted to give the most separation, turns included.
    ra_vertical_first: bool = True
    # TUNABLE PLACEHOLDER: with altitude first, a turn replaces the climb or
    # descent only when that climb or descent is predicted to reach less than
    # this fraction of the RA success target AND the turn is predicted to do
    # better. 1.0 means "turn whenever the altitude change would miss the
    # target"; lower values hold on to the altitude change longer.
    ra_turn_fallback_score: float = 0.5

    # --- Ground (main.py, resolution.py) -----------------------------------------
    # TUNABLE PLACEHOLDER: no descend RA is issued when ownship is less than
    # this far above the ground. Used only when a ground elevation is known
    # (scenario "field_elev_ft", or --field-elev in live mode).
    ra_descend_min_agl_ft: float = 1000.0

    # --- Compliance monitoring and autopilot handoff (autopilot.py) -----------
    # TUNABLE PLACEHOLDER: hand the RA to the autopilot if the pilot has not
    # complied this long after the RA was issued.
    compliance_timeout_s: float = 5.0
    # TUNABLE PLACEHOLDER: vertical-rate change in the commanded sense that
    # counts as complying with a climb or descend RA.
    compliance_vs_delta_fpm: float = 200.0
    # TUNABLE PLACEHOLDER: track change in the commanded direction that counts
    # as complying with a turn RA.
    compliance_track_delta_deg: float = 3.0

    # --- Audio (audio.py) ------------------------------------------------------
    # TUNABLE PLACEHOLDER: traffic within this many feet is called "level".
    ta_level_band_ft: float = 300.0

    # --- Live data (traffic.py, main.py) ---------------------------------------
    # TUNABLE PLACEHOLDER: HTTP timeout when polling dump1090. Kept well under
    # the cycle period so a receiver outage cannot stall the 1 Hz cycle.
    dump1090_timeout_s: float = 0.5
    # ICAO address used for the fixed ownship in live mode. Not a threshold, but
    # it decides the ICAO coordination tie-break, so it is set here.
    live_ownship_icao: str = "000000"

    # --- Simulation (traffic.py) -----------------------------------------------
    # TUNABLE PLACEHOLDER: integration step for simulated aircraft.
    sim_substep_s: float = 0.1
    # TUNABLE PLACEHOLDER: after a simulated aircraft touches down it slows at
    # this rate to this taxi speed.
    sim_rollout_decel_kt_s: float = 4.0
    sim_taxi_speed_kt: float = 15.0

    # Derived values in SI units, used internally.
    @property
    def surveillance_range_m(self) -> float:
        return self.surveillance_range_mi * M_PER_STATUTE_MILE

    @property
    def mandatory_range_m(self) -> float:
        return self.mandatory_range_mi * M_PER_STATUTE_MILE

    @property
    def threat_hmd_m(self) -> float:
        return self.threat_hmd_nm * M_PER_NM

    @property
    def threat_vmd_m(self) -> float:
        return self.threat_vmd_ft * M_PER_FT

    @property
    def ra_success_hmd_m(self) -> float:
        return self.ra_success_hmd_nm * M_PER_NM

    @property
    def ra_success_vmd_m(self) -> float:
        return self.ra_success_vmd_ft * M_PER_FT


DEFAULT = Config()

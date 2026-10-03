# GATCAS: ADS-B collision avoidance prototype for general aviation

A hackathon prototype that reads ADS-B traffic, predicts conflicts, gives spoken
traffic and resolution advisories (TA/RA), and hands an ignored RA to a
**simulated** autopilot. An RA is a change of altitude (climb or descend); a
turn is the last resort. A phone-sized traffic scope shows all of it.

> **Simulation and demo only.** Nothing in this package connects to real aircraft
> controls. `SimulatedAutopilot` changes the state of a software aircraft model
> inside the simulation, and live mode always runs with the autopilot disabled.
> The thresholds are placeholders, not validated or certified values.

## Setup

Python 3.11 or newer.

```bash
python -m venv .venv
.venv/Scripts/python -m pip install -r requirements.txt   # Windows
# .venv/bin/python -m pip install -r requirements.txt     # macOS / Linux
```

Dependencies: `numpy`, `pyttsx3`, `requests`, `pytest`. On Windows, pyttsx3 pulls
in `comtypes` and `pywin32` for SAPI5 speech.

## Running scenarios

```bash
python -m gatcas --scenario scenarios/head_on.json          # real time, spoken alerts
python -m gatcas --scenario scenarios/head_on.json --fast   # as fast as possible, printed alerts only
```

| Scenario | Geometry | Pilot | Expected events |
|---|---|---|---|
| `head_on.json` | Reciprocal track, co-altitude, 110 kt each | ignores RA | TA, RA climb, autopilot engaging, clear of conflict |
| `head_on_compliant.json` | Same | complies after 2 s | TA, RA climb, clear of conflict |
| `head_on_500ft_below.json` | Reciprocal track, intruder 500 ft below | ignores RA | TA, RA climb, autopilot engaging, clear of conflict |
| `crossing_90.json` | Intruder crossing from the right | complies after 2 s | TA, RA climb, clear of conflict |
| `overtaking.json` | 160 kt intruder from behind, 100 ft above | complies after 2 s | TA, RA descend, clear of conflict |
| `diverging.json` | Three aircraft moving away | (none) | none |
| `parallel_pass.json` | Reciprocal track 0.7 nm abeam | (none) | none |
| `mixed_traffic.json` | Five aircraft from 1,800 ft above to 1,400 ft below; one descends through ownship's altitude from the right | complies after 2 s | TA, RA descend, clear of conflict |
| `ground_traffic.json` | Final approach and landing at a 5,400 ft field: one aircraft holding short, one taxiing, one parked, pattern traffic and a departure | ignores RA | none (without the ground flags: TA, RA climb and autopilot on short final) |
| `low_level_turn.json` | Head-on traffic 150 ft above, ownship 700 ft above the field | ignores RA | TA, RA turn right ("too low to descend"), autopilot engaging, clear of conflict |

In real-time mode, alerts are spoken with pyttsx3 on a background thread and are
also printed. If pyttsx3 cannot start or speaking fails (for example, no audio
device), alerts are printed only. `--fast` turns speech off, because spoken
phrases would lag far behind the simulation.

### Log format

Each 1 Hz cycle prints one line per tracked target:

```
[   29.0s] A00002  rng  2.46 mi  rel alt     +0 ft  tCPA  35.0 s  HMD  0.00 nm  VMD     0 ft  RA CLIMB
```

These are: time, ICAO address, horizontal range, intruder altitude minus ownship
altitude, time to CPA, predicted horizontal and vertical miss at CPA, and the
alert state (`NONE`, `TA`, `RA <maneuver>`, or `GND` for an aircraft on the
ground). Event lines start with `***` and are followed by the spoken phrase:

```
[   29.0s] *** RESOLUTION ADVISORY A00002: CLIMB (predicted separation at CPA / RA success target: climb 0.96, descend 1.89, turn left 4.66, turn right 4.66; tie broken by coordination rule; altitude first)
    AUDIO (printed, no speech): Climb, climb
```

The numbers after an RA are each option's predicted separation at CPA, divided
by the RA success target (1.0 or more means the RA is predicted to succeed).
The last phrase says why that kind of maneuver was chosen: `altitude first`,
`turn: too low to descend`, `turn: altitude change predicted to fall short`,
or `most predicted separation` when `ra_vertical_first` is off.

### Scenario file format

```json
{
  "name": "Head-on, co-altitude, pilot ignores the RA",
  "duration_s": 100,
  "ownship": {"icao": "A00001", "lat": 39.57, "lon": -104.85, "alt_ft": 7500,
              "gs_kt": 110, "track_deg": 0, "vs_fpm": 0},
  "pilot": {"response": "ignore"},
  "intruders": [
    {"icao": "A00002", "east_mi": 0.0, "north_mi": 4.5, "alt_ft": 7500,
     "gs_kt": 110, "track_deg": 180, "vs_fpm": 0}
  ]
}
```

- `pilot.response` is `"comply"` (fly the RA after `delay_s`, default 2 s) or `"ignore"`.
- Place intruders with `lat`/`lon`, or with `east_mi`/`north_mi` offsets from
  ownship's starting position.
- Intruders fly at constant velocity.
- Optional `"field_elev_ft"` at the top level is the ground elevation. An
  aircraft that descends to it touches down, becomes "on the ground", rolls
  out and taxis. It also sets the descend floor (see Assumptions).
- An aircraft with `"on_ground": true` starts on the ground. Its `alt_ft`
  defaults to the field elevation; `gs_kt` 0 is parked, anything else taxis.

## Running against dump1090 (live data)

```bash
python -m gatcas --dump1090 http://raspberrypi:8080/data/aircraft.json --lat 39.57 --lon -104.85 --alt 7500
```

- The URL is the receiver's `aircraft.json`. Common locations:
  - dump1090-fa: `http://<host>:8080/data/aircraft.json` or `http://<host>/skyaware/data/aircraft.json`
  - readsb/tar1090: `http://<host>/tar1090/data/aircraft.json`
- `--lat`/`--lon` (degrees) and `--alt` (pressure altitude, feet) set a fixed ownship.
  Ownship has zero velocity, so clock positions are relative to true north.
- The autopilot is always disabled in live mode. You still get TAs, RAs and
  "Clear of conflict".
- Fields read per aircraft: `hex`, `lat`, `lon`, `alt_baro`, `gs`, `track`,
  `baro_rate`, plus `seen_pos` (or `seen`) to time-stamp the report.
- `--field-elev FT` gives the field elevation. An aircraft on the ground
  reports `alt_baro` as the word `"ground"`; with the elevation known it is
  shown at that altitude and marked `GND`, and it is never alerted on. Without
  `--field-elev` those records are skipped. The same number sets the descend
  floor.
- `--on-ground` says the fixed ownship is itself on the ground (a receiver on
  the ramp): traffic advisories only, no RAs.
- Stop with Ctrl+C.

## Traffic scope (visual display)

```bash
python -m gatcas.viz                                   # every scenario in ./scenarios, opens gatcas_scope.html
python -m gatcas.viz --scenario scenarios/head_on.json
python -m gatcas.viz --dump1090 http://raspberrypi:8080/data/aircraft.json --lat 39.57 --lon -104.85 --alt 7500
python -m gatcas.viz --serve --lan                     # open it on a phone on the same Wi-Fi
```

Scenario mode runs each scenario through the normal simulation loop, saves one
frame per 1 Hz cycle, and writes **one self-contained HTML file** that replays
them. It needs no server and no extra packages; the file opens by double-click
and can be handed to anyone. Live mode serves the same page on
`http://127.0.0.1:8765/` and streams a frame per cycle. As in the CLI, live
mode is advisory only. `--mute` prints alerts instead of speaking them, and
`--field-elev` / `--on-ground` work as in the CLI.

### On a phone

The page is laid out for a phone held upright: the picture (scope, side view,
color key) takes about 70 % of the screen and the traffic alert the other
30 %. Legend, Traffic, Events, Rings and Setup open as a sheet from the row of
buttons at the bottom, and a sheet closes by itself when a new advisory
arrives. On a wide screen the same column is shown with every panel open
beside it.

To open it on a phone:

1. Put the phone and the computer on the same Wi-Fi.
2. Run `python -m gatcas.viz --serve --lan` (recorded scenarios) or add `--lan`
   to the `--dump1090` command (live). It prints the address to open, for
   example `http://192.168.1.20:8765/`.
3. In the phone's browser, use Add to Home Screen (iPhone: the Share button;
   Android: the browser menu). It then opens full screen like an app.

Without `--lan` the server answers this computer only. `--lan` lets any
device on the local network open the page (it is read-only; nothing can be
changed through it). The exported `gatcas_scope.html` can also be sent to a
phone as a file.

The display never decides anything. Every alert, RA and number on it comes from
the same `CasSystem.cycle()` the CLI runs; `tests/test_viz.py` checks that a
recorded run and a plain run produce identical events and values.

What is on the page:

- **Scope.** Top-down, centred on ownship, track up or north up. The range
  scales itself to the most urgent aircraft, or can be fixed.
- **Boundaries as rings.** Each threshold in `config.py` is a `Shell` object in
  `gatcas/zones.py` with a radius, a height and a rank:

  | Shell | Radius | Height | From |
  |---|---|---|---|
  | Surveillance | 10 mi | any | `surveillance_range_mi` |
  | Watch | 3 mi | any | `mandatory_range_mi` |
  | TA ring | 50 s to closest approach | ±600 ft | `ta_tcpa_s` |
  | RA ring | 35 s to closest approach | ±600 ft | `ra_tcpa_s` |
  | Threat volume | 0.5 nm | ±600 ft | `threat_hmd_nm`, `threat_vmd_ft` |
  | RA target | 0.25 nm | ±300 ft | `ra_success_hmd_nm`, `ra_success_vmd_ft` |

  The boundaries are flat discs, not spheres: the threat volume is 3,038 ft
  wide and 600 ft tall. From above a disc is a ring; in the side view it is a box.
- **Time rings.** TA and RA are times, not distances, so their rings are drawn
  at `sqrt(HMD² + (relative speed × T)²)`, the range at which that aircraft's
  time to closest approach equals T. They grow with closing speed (2.46 mi for
  the RA ring in a 220 kt head-on, 0.67 mi in a 60 kt overtake) and are exact:
  a threat is inside the ring if and only if `alerting.classify()` returns that
  level (`tests/test_zones.py`, 10,000 random geometries). They are drawn for
  the most urgent threat, dashed until crossed.
- **Ripples.** Each aircraft carries one ripple per boundary it is inside. They
  turn amber for a TA and red for an RA, and move faster as it gets closer.
- **Altitude as color.** Five bands relative to ownship: more than 600 ft
  above, 300-600 ft above, within 300 ft, 300-600 ft below, more than 600 ft
  below. The breaks are `ta_level_band_ft` and `threat_vmd_ft`. Violet is
  above, teal is below, white is level, and the color deepens away from your
  altitude. Amber and red are kept for alerts only. Every aircraft also shows
  the difference in hundreds of feet (`+07`), so altitude never depends on
  color alone.
- **Side view.** Range against altitude difference, with the same boundaries
  as boxes. An aircraft directly overhead but 1,800 ft up is inside the ring in
  the top view and visibly outside the box here.
  It also carries the altitude advice as a red arrow at ownship, and, when
  the field elevation is known, the ground as a hatched band and the descend
  floor as a dotted line.
- **Aircraft on the ground.** Hollow grey symbol on a short ground bar, tagged
  `GND`, with no altitude color and no ripples. Ownship shows `ON GROUND`
  after it lands, and the banner says "On ground: traffic advisories only".
- **Color key.** Always visible under the side view; tap it for the full legend.
- **Traffic alert.** A banner filled amber for a TA and red for an RA, with the
  maneuver as a word and an arrow (`CLIMB`, `DESCEND`, or `TURN RIGHT` with
  the reason). Under it: the most urgent (or selected) aircraft, its range,
  altitude difference, time to closest approach and predicted miss, a six-step
  ladder of the boundaries it is inside, and one line saying why this advice
  was chosen or which boundary comes next. The miss turns amber while an RA is
  active and the prediction is still inside the RA target.
- **Sheets.** Traffic table, event log (tap an event to jump to it), boundary
  table, and Setup (speed, range, track up or north up, voice). Tap an
  aircraft or a table row to follow it. Voice speaks the phrases in the
  browser and, on phones that support it, vibrates on an RA.

Between two 1 Hz cycles the picture is interpolated so motion is smooth. Alert
state, RA and boundary membership are never interpolated.

## Tests

```bash
python -m pytest
```

| Acceptance criterion | Test |
|---|---|
| Head-on, 90 degree crossing, overtaking, diverging (no alert), parallel pass (no alert) | `tests/test_prediction.py`, `tests/test_alerting.py`, `tests/test_main.py::test_scenario_event_sequences` |
| Compliant pilot never triggers autopilot handoff | `tests/test_autopilot.py::test_compliant_pilot_never_triggers_autopilot_handoff` (3 scenarios × 4 response delays) |
| Ignored RA hands off after the timeout and the autopilot climbs | `tests/test_autopilot.py::test_ignored_ra_hands_off_after_timeout_and_climbs` |
| The same with `ra_vertical_first` off: the turn takes the predicted miss past the RA success target and the threat thresholds | `tests/test_autopilot.py::test_ignored_ra_hands_off_after_timeout_and_resolves_the_threat` |
| Two aircraft running this logic select complementary RAs | `tests/test_coordination.py` |
| Altitude first: one climbs, one descends; the measured separations, including the lone-climber shortfall and its two fixes | `tests/test_coordination.py` (the `altitude_first` tests), `tests/test_resolution.py` |
| Aircraft on the ground are tracked, shown and never alerted on; ownship on the ground gets TAs only; no descend RA close to the ground | `tests/test_ground.py` |
| Boundary objects follow `Config`; the TA and RA rings equal the alerting rule | `tests/test_zones.py` |
| Recording a scenario changes nothing; export, phone serving and live server work | `tests/test_viz.py` |
| `--scenario scenarios/head_on.json --fast` logs TA, RA, autopilot, clear in order | `tests/test_main.py::test_cli_head_on_fast_logs_ta_ra_autopilot_clear_in_order` |

`tests/test_coordination.py` flies two equipped aircraft against each other:
head-on, 90° crossing, co-altitude overtake (both ICAO orderings, plus an
equal-rate variant where both turn), late detection (4-10 s, both ICAO
orderings), a vertical offset that overrides the ICAO rule, and two aircraft
stacked on the same final. The turn tests run with `ra_vertical_first=False`.

## Package layout

| Module | Role |
|---|---|
| `gatcas/config.py` | Every tunable threshold (see below) |
| `gatcas/traffic.py` | `Target`, `TrafficSource`, `SimulatedSource` (scenario playback, aircraft and pilot models), `Dump1090Source` |
| `gatcas/surveillance.py` | Picks targets to track and evaluate each cycle |
| `gatcas/prediction.py` | ENU frame, constant-velocity CPA, miss distances, CPA location (pure) |
| `gatcas/alerting.py` | Threat test, TA/RA classification, RA hold logic (pure plus a small state tracker) |
| `gatcas/resolution.py` | RA option model, scoring, coordination tie rules (pure) |
| `gatcas/audio.py` | Phrases (pure) and the pyttsx3 announcer |
| `gatcas/autopilot.py` | `Autopilot`, `SimulatedAutopilot`, compliance check, handoff monitor |
| `gatcas/main.py` | Per-cycle pipeline (`CasSystem`), simulation and live loops, CLI |
| `gatcas/zones.py` | The boundaries as `Shell` objects built from `Config`; which ones a target is inside (pure) |
| `gatcas/viz.py` | Records scenarios, writes the standalone scope page, serves live mode |
| `gatcas/web/scope.html` | The scope page (no build step, no dependencies beyond a browser) |

Each cycle runs surveillance → prediction → alerting → resolution → handoff,
then logging and announcements.

## Tunable thresholds

All live in `gatcas/config.py` as fields of `Config`, each marked `TUNABLE PLACEHOLDER`.

| Name | Default | Meaning |
|---|---|---|
| `surveillance_range_mi` | 10.0 mi | Track every target inside this horizontal range |
| `mandatory_range_mi` | 3.0 mi | Targets inside this range are tracked and evaluated every cycle |
| `stale_target_s` | 10.0 s | Drop a target with no update for this long |
| `cycle_period_s` | 1.0 s | Processing cycle (1 Hz) |
| `threat_hmd_nm` | 0.5 nm | Threat if predicted horizontal miss is below this... |
| `threat_vmd_ft` | 600 ft | ...and predicted vertical miss is below this |
| `ta_tcpa_s` | 50 s | Traffic advisory when a threat's tCPA ≤ this |
| `ra_tcpa_s` | 35 s | Resolution advisory when a threat's tCPA ≤ this |
| `ra_success_hmd_nm` | 0.25 nm | RA succeeds if predicted horizontal miss at CPA reaches this... |
| `ra_success_vmd_ft` | 300 ft | ...or predicted vertical miss at CPA reaches this |
| `ra_climb_fpm` | 500 ft/min | Climb rate a climb RA commands |
| `ra_descent_fpm` | 1000 ft/min | Descent rate a descend RA commands |
| `ra_bank_deg` | 25° | Bank angle a turn RA commands |
| `vertical_accel_g` | 0.25 g | Vertical acceleration to reach the commanded rate |
| `ra_eval_margin_s` | 10 s | RA options are evaluated out to tCPA + this |
| `ra_eval_step_s` | 0.5 s | Sampling step for RA option trajectories |
| `ra_tie_tolerance` | 0.05 | Normalized separations this close count as a tie |
| `ra_speed_tie_kt` | 1 kt | Ground speeds this close count as equal in the turn tie rule |
| `ra_vertical_first` | `True` | Altitude first: the RA is climb or descend, and a turn is the last resort. `False` restores "whichever predicts the most separation" |
| `ra_turn_fallback_score` | 0.5 | Altitude first only: turn instead when the altitude change is predicted to reach less than this fraction of the RA success target and a turn does better. 0 = never turn for a shortfall, 1.0 = turn whenever altitude alone is predicted to miss the target |
| `ra_descend_min_agl_ft` | 1000 ft | No descend RA below this height above the ground (only when the ground elevation is known) |
| `compliance_timeout_s` | 5 s | Hand the RA to the autopilot after this long without compliance |
| `compliance_vs_delta_fpm` | 200 ft/min | Vertical-rate change that counts as complying with climb/descend |
| `compliance_track_delta_deg` | 3° | Track change that counts as complying with a turn |
| `ta_level_band_ft` | 300 ft | Traffic within ±this is called "level" |
| `dump1090_timeout_s` | 0.5 s | HTTP timeout per poll (kept under the cycle period) |
| `sim_substep_s` | 0.1 s | Integration step for simulated aircraft |
| `sim_rollout_decel_kt_s` | 4 kt/s | Simulated landing roll: deceleration after touchdown |
| `sim_taxi_speed_kt` | 15 kt | Simulated landing roll: speed it slows to |
| `live_ownship_icao` | `000000` | Ownship address in live mode (used by the ICAO tie rule) |

The threat thresholds decide *when* to alert. The RA success target, which is
deliberately smaller, decides what an RA must achieve. "Clear of conflict" still
uses the threat thresholds.

Units follow the spec: ranges in statute miles, converted to meters internally;
altitude in feet; speed in knots; vertical rate in ft/min. The horizontal-miss
threshold and the horizontal success target stay in nautical miles because they
are specified that way.

## Assumptions

These cover points the spec does not settle.

**Geometry and prediction**
1. The ENU frame is a local tangent-plane projection using WGS-84 radii of
   curvature at ownship's latitude. "Up" is the pressure-altitude difference
   (no earth curvature): aircraft hold pressure altitude, so altitudes are
   compared directly. Against exact WGS-84 distances the horizontal error is
   at most 2 m at 3 mi and 21 m at 10 mi (latitudes 0-70°).
2. Every range is horizontal, not slant: the 10 mi and 3 mi rings, logged
   range, and spoken TA distance.
3. CPA is the time of minimum *horizontal* distance at constant velocity. The
   vertical miss is the altitude difference at that time. With no horizontal
   closure, tCPA = 0 and the miss distances are the current separation.
4. "Predicted CPA location" is the midpoint between the two aircraft at tCPA.
5. Ground track stands in for heading (clock position, turn compliance), and
   ground speed stands in for airspeed in the turn-rate formula (no wind).
6. Altitudes are pressure altitudes (`alt_baro`). There is no QNH or geometric
   correction.

**Surveillance**
7. There is no cap on the number of tracks, so every target inside 3 mi is
   always evaluated. A target whose newest report is older than the cycle time
   is dead-reckoned to the cycle time until it goes stale at 10 s. Tracks are
   processed closest first.
8. A report with the ownship's own ICAO address is ignored.

**Alerting**
9. Threat thresholds are strict (`<`); TA/RA tCPA thresholds are inclusive (`<=`).
10. Once issued, an RA is held until tCPA has passed and the threat condition is
    false. "tCPA has passed" means the current tCPA is 0 (no longer converging).
    So an RA does not drop out halfway through the maneuver that is resolving it.
11. A TA is announced once, on the transition from no alert. TAs have no
    hysteresis. A target that jumps straight to RA gets no TA announcement.

**Resolution**
12. Each RA option is flown forward from now, with no assumed pilot delay. Climb
    ramps at 0.25 g to +500 ft/min and descend to -1000 ft/min; a climb never
    reduces an existing climb rate (and likewise for descent). Turns roll in
    instantly to a 25° bank and hold the current vertical rate. The intruder
    holds constant velocity.
13. "Largest predicted separation at CPA" is measured as
    `max(horizontal miss / 0.25 nm, vertical miss / 300 ft)` at the closest
    horizontal approach of the maneuvered trajectories, that is, relative to
    the RA success target. A value of 1.0 or more means the RA is predicted to
    succeed.
14. Selection runs in three steps, and a "tie" means two scores within 0.05:
    - **Vertical sense.** Climb and descend are compared at the *same* rate (the
      lower of the two limits, 500 ft/min). Without this, the 1000 ft/min
      descent always outscores the 500 ft/min climb at co-altitude, the tie rule
      never fires, and both aircraft descend together. Vertical tie: the lower
      ICAO address climbs and the higher descends (24-bit addresses compared
      numerically, with the `~` non-ICAO prefix stripped).
    - **Turn direction.** Turn tie with tracks more than 90° apart (opposing):
      both turn right. Turn tie with tracks within 90° (inclusive; overtaking or
      converging), following right of way: the faster aircraft turns right and
      the slower turns left. If ground speeds are within 1 kt, the lower ICAO
      address turns right and the higher turns left.
      A vertical tie between aircraft that are at different altitudes now (by
      more than 15 ft, the tie tolerance times the 300 ft target) is not given
      to the ICAO rule: each moves away from the other's altitude, so neither
      crosses it. This matters when two aircraft are stacked on the same track
      at the same speed (the same final approach): every option then scores
      the same, and the ICAO rule alone could send the lower one up.
    - **Altitude first (default, `ra_vertical_first`).** The chosen vertical
      sense is flown. A turn is the last resort, used in two cases only:
      the sense is "descend" but ownship is below the descend floor
      (assumption 26); or the sense is predicted to reach less than half the
      RA success target (`ra_turn_fallback_score`) *and* the turn is predicted
      to do better, which in practice means the aircraft that would climb,
      with traffic first seen less than about 20 s before CPA. The log and
      the scope say which case applied.
    - **Most separation (`ra_vertical_first=False`).** The original rule: the
      chosen sense, scored at its own rate limit, is compared with the chosen
      turn, and the higher score is flown (an exact tie goes to vertical).
      Because the two aircraft score their options differently, they can fly
      different kinds of maneuver. In a co-altitude overtake, for example, one
      turns while the other descends. They still never share a vertical sense.
15. The tie rules apply to every intruder, because without a data link you
    cannot know whether the other aircraft is equipped.
16. The RA sense is chosen once, when the RA is issued. There are no reversals
    or strengthenings.
17. With several threats, one RA is chosen per threat (shown on each target's
    log line). Ownship announces and flies the RA for the threat with the
    smallest tCPA. "Clear of conflict" is announced only when no RA remains.

**Compliance and autopilot**
18. Compliance is measured against ownship's state when the RA was issued.
    Climb/descend: vertical rate moved at least 200 ft/min in the commanded
    sense, or already at the commanded rate (500 ft/min up or 1000 ft/min down). Turn: track moved at least
    3° in the commanded direction. Once met, compliance is not re-checked.
19. Handoff happens on the first 1 Hz cycle at least 5 s after the RA. If the
    active RA changes, the timer restarts. If the autopilot is already engaged,
    it is re-commanded instead.
20. On release, the simulated autopilot rolls wings level and levels off.
21. If an RA target's track is lost (stale), the RA ends and the autopilot is
    released. This is logged as `RA TARGET LOST`, with no "Clear of conflict"
    phrase.

**Audio**
22. Spoken TA distance is in statute miles, rounded to a whole number
    ("less than 1 mile" below 0.5 mi). "Level" means within ±300 ft.

**Live data**
23. Records without a position, or with `alt_baro` not numeric, are skipped.
    `"ground"` is the exception when the field elevation is given
    (assumption 25). A missing `gs`, `track` or `baro_rate` is treated
    as 0. Report time is local time minus `seen_pos` (or `seen`), so the
    receiver's clock does not matter. A failed poll keeps serving cached
    targets until they go stale.

**Simulation**
24. Aircraft are kinematic point masses integrated at 0.1 s. Intruders fly
    constant velocity. The simulated pilot reacts at 0.1 s granularity. With a
    field elevation, an aircraft that descends to it lands: vertical rate
    zero, 4 kt/s deceleration to 15 kt, then it keeps taxiing straight ahead.
    There is no takeoff model.

**On the ground**
25. "On the ground" is a flag on a target, not something the engine works out
    from altitude. It comes from the scenario file, from touchdown in the
    simulator, or from dump1090 reporting `alt_baro` as `"ground"`. Such a
    record carries no altitude, so it is placed at the field elevation.
    An aircraft on the ground is tracked, logged (`GND`) and drawn, and never
    gets a TA or an RA; an RA against an aircraft that then lands ends with
    "Clear of conflict". While ownship is on the ground, airborne threats are
    capped at a traffic advisory: there is no maneuver to advise.
26. Descend floor. When the ground elevation is known (scenario
    `field_elev_ft`, live `--field-elev`), a descend RA is not issued below
    1,000 ft above it. If the vertical sense is "descend" there, the RA is a
    turn instead of a climb, because the sense came from the coordination
    rule and climbing could match the other aircraft's climb. With no
    elevation given there is no floor. The elevation is one number, not a
    terrain model.

## Known limitations

These come from the specified thresholds and rules interacting. The numbers
were measured with this code.

0. **Altitude first: one aircraft climbing alone falls short of the 300 ft
   target.** Co-altitude, RA at tCPA 35 s, vertical separation at CPA:

   | Who is equipped | Autopilot after 5 s | Pilot after 2 s | 300 ft target |
   |---|---|---|---|
   | Both (one climbs, one descends) | 713 to 728 ft | 788 to 803 ft | met |
   | Only the aircraft that descends (1000 ft/min) | 473 to 483 ft | 523 to 533 ft | met |
   | Only the aircraft that climbs (500 ft/min) | 241 to 246 ft | 266 to 271 ft | **short** |

   (Ranges cover head-on, 90° crossing and overtake.) `head_on.json` is the
   third row: ownship has the lower address, climbs alone, and passes 246 ft
   above the other aircraft. Two one-line changes in `config.py` each close
   the gap, and `tests/test_coordination.py` checks both:

   | Change | Lone climber, autopilot after 5 s | Pilot after 2 s |
   |---|---|---|
   | `ra_tcpa_s = 43` (with `ta_tcpa_s = 58`): advise 8 s earlier | 307 to 312 ft | 332 to 337 ft |
   | `ra_climb_fpm = 700`: climb harder | 335 to 342 ft | 370 to 377 ft |

   A third option, `ra_turn_fallback_score = 1.0`, makes the lone climber turn
   instead (0.27 to 0.91 nm), which gives up altitude-only advice in exactly
   the headline case. Altitude first also gives up separation when traffic is
   first seen 16 to 30 s before CPA and only one aircraft is equipped: a
   descent alone reaches 249 to 283 ft at 16 to 18 s, and a climb alone 162 to
   246 ft at 20 to 30 s, where a turn would have met the target.
   With vertical-only RAs the two aircraft must pick opposite senses, and at
   co-altitude that rests entirely on the ICAO rule (see limitation 4). This
   is the first thing a peer link should confirm.
1. **An RA needs about 12 s of warning to reach the success target.** (This
   table was measured with `ra_vertical_first=False`, where late RAs are
   turns. Under altitude first the same late detections still turn when the
   altitude change scores under half the target.) RAs
   normally start at tCPA 35 s, so this matters when a target is first
   acquired late (data dropout, pop-up traffic) or when the geometry changes
   late. Measured in a co-altitude head-on at 110 kt each, separation at CPA:

   | Warning before CPA | Both equipped, respond at once | One equipped, responds at once |
   |---|---|---|
   | 4 s | 0.00 nm / 78 ft, missed | 0.00 nm / 29 ft, missed |
   | 6 s | 0.04 nm / 81 ft, missed | 0.04 nm / 0 ft, missed |
   | 8 s | 0.08 nm / 114 ft, missed | 0.08 nm / 0 ft, missed |
   | 10 s | 0.12 nm / 144 ft, missed | 0.12 nm / 0 ft, missed |
   | 11 s | 0.26 nm, met (just) | 0.14 nm, missed |
   | 12 s | 0.30 nm, met | 0.17 nm, missed |
   | 14 s | 0.39 nm, met | 0.23 nm, missed |
   | 15 s | 0.43 nm, met | 0.26 nm, met |

   With both aircraft responding at once, the target is first met at 11 s, by
   0.01 nm. One equipped aircraft needs about 15 s. When the RA reaches the
   autopilot through the 5 s handoff, the response starts 5 s later and the
   minimum grows by the same amount: 16 s with both equipped (0.26 nm; 15 s
   gives 0.22 nm) and 20 s with one (0.26 nm; 19 s gives 0.23 nm).
2. **A climb RA alone cannot reach the vertical target from co-altitude.**
   500 ft/min for 35 s is about 290 ft, short of 300 ft. A climb succeeds only
   with an existing vertical offset. The 1000 ft/min descent reaches about
   580 ft.
3. **Alerting and success use different volumes.** An encounter inside the
   600 ft threat volume gets an RA even if, unmaneuvered, it already meets the
   300 ft success target. `head_on_500ft_below.json` is such a case (500 ft apart).
4. Near the tie tolerance (0.05) or the speed tie band (1 kt), two aircraft
   seeing slightly different data could disagree about whether a tie exists:
   one applies the tie rule, the other follows the geometry. Without a data
   link this cannot be fully avoided.
5. tCPA is horizontal, so two aircraft that are horizontally co-located and
   converging only vertically alert only once inside the threat volume.
6. Constant-velocity prediction does not know a turn will continue, so after a
   turn RA the logged HMD grows gradually rather than jumping.
7. By the specified threat definition, a target inside 0.5 nm and 600 ft is a
   threat even when diverging (tCPA = 0), so it gets an RA.

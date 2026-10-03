import re
import subprocess
import sys
from pathlib import Path

import pytest

from gatcas.main import main, run_live, run_scenario
from gatcas.traffic import Dump1090Source

ROOT = Path(__file__).resolve().parent.parent
SCENARIOS = ROOT / "scenarios"
TARGET_LINE = re.compile(r"^\[\s*([\d.]+)s\] (\S+)\s+rng ")


def run(name, **kw):
    lines = []
    result = run_scenario(SCENARIOS / name, fast=True, out=lines.append, **kw)
    return result, lines


@pytest.mark.parametrize("name, expected", [
    ("head_on.json", ["TA", "RA", "AUTOPILOT", "CLEAR"]),
    ("head_on_500ft_below.json", ["TA", "RA", "AUTOPILOT", "CLEAR"]),
    ("head_on_compliant.json", ["TA", "RA", "CLEAR"]),
    ("crossing_90.json", ["TA", "RA", "CLEAR"]),
    ("overtaking.json", ["TA", "RA", "CLEAR"]),
    ("diverging.json", []),
    ("parallel_pass.json", []),
])
def test_scenario_event_sequences(name, expected):
    result, _ = run(name)
    assert [e.kind for e in result.events] == expected


@pytest.mark.parametrize("name", ["diverging.json", "parallel_pass.json"])
def test_no_alert_scenarios_never_leave_alert_state_none(name):
    result, _ = run(name)
    assert all(s.level.name == "NONE" for r in result.reports for s in r.targets)
    assert all(r.targets for r in result.reports)   # traffic was tracked the whole time


def test_one_log_line_per_tracked_target_per_cycle():
    result, lines = run("diverging.json")
    per_cycle = {}
    for line in lines:
        m = TARGET_LINE.match(line)
        if m:
            per_cycle.setdefault(float(m.group(1)), []).append(m.group(2))
    assert len(per_cycle) == len(result.reports) == 61
    for report in result.reports:
        assert sorted(per_cycle[report.time]) == sorted(s.pred.icao for s in report.targets)


def test_cli_head_on_fast_logs_ta_ra_autopilot_clear_in_order():
    proc = subprocess.run(
        [sys.executable, "-m", "gatcas", "--scenario", "scenarios/head_on.json", "--fast"],
        cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert proc.returncode == 0, proc.stderr
    out = proc.stdout
    marks = ["*** TRAFFIC ADVISORY", "*** RESOLUTION ADVISORY", "*** AUTOPILOT ENGAGING",
             "*** CLEAR OF CONFLICT"]
    positions = [out.index(m) for m in marks]
    assert positions == sorted(positions)
    assert "RESOLUTION ADVISORY A00002: CLIMB" in out and "altitude first" in out
    for phrase in ["Traffic, 12 o'clock, level, 4 miles.", "Climb, climb", "Autopilot engaging",
                   "Clear of conflict"]:
        assert phrase in out
    assert out.rstrip().endswith("Scenario complete.")


def test_cli_requires_position_for_dump1090():
    with pytest.raises(SystemExit) as exc:
        main(["--dump1090", "http://localhost:8080/data/aircraft.json", "--lat", "39.5"])
    assert exc.value.code == 2


class FakeSession:
    def get(self, url, timeout):
        return self

    def raise_for_status(self):
        pass

    def json(self):
        # A target 0.8 mi north, inbound at 120 kt, same altitude as ownship.
        return {"aircraft": [{"hex": "a0beef", "lat": 39.58158, "lon": -104.85, "alt_baro": 7500,
                              "gs": 120, "track": 180, "baro_rate": 0, "seen_pos": 0.2}]}


def test_live_mode_alerts_with_autopilot_disabled():
    clock = [1000.0]
    lines = []
    result = run_live(Dump1090Source("http://rx/data/aircraft.json", session=FakeSession()),
                      39.57, -104.85, 7500.0, out=lines.append, max_cycles=8,
                      clock=lambda: clock[0], sleep=lambda s: clock.__setitem__(0, clock[0] + s))
    assert len(result.reports) == 8
    assert [e.kind for e in result.events] == ["RA"]
    assert not any(r.autopilot_engaged for r in result.reports)
    assert any("autopilot disabled" in line for line in lines)
    assert sum(1 for line in lines if TARGET_LINE.match(line)) == 8

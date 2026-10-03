import json
import threading
import urllib.request
from pathlib import Path

import pytest

from gatcas.config import DEFAULT
from gatcas.main import run_scenario
from gatcas.traffic import Target, TrafficSource
from gatcas.viz import (DATA_PLACEHOLDER, LiveFeed, build_payload, export_html, lan_address, main,
                        make_server, record_scenario, render_fragment, render_html)

ROOT = Path(__file__).resolve().parent.parent
SCENARIOS = ROOT / "scenarios"
ORIGINAL = ["crossing_90.json", "diverging.json", "head_on.json", "head_on_500ft_below.json",
            "head_on_compliant.json", "overtaking.json", "parallel_pass.json"]


@pytest.mark.parametrize("name", ORIGINAL + ["mixed_traffic.json", "ground_traffic.json", "low_level_turn.json"])
def test_recording_matches_the_plain_run_exactly(name):
    """Recording must not change what the engine does: same events at the same
    times, and the same numbers on every target in every cycle."""
    ref = run_scenario(SCENARIOS / name, fast=True, out=lambda line: None)
    rec = record_scenario(SCENARIOS / name)
    assert len(rec["frames"]) == len(ref.reports)
    got = [(f["t"], e["kind"], e["icao"], e["maneuver"]) for f in rec["frames"] for e in f["events"]]
    want = [(e.time, e.kind, e.icao, e.maneuver.value if e.maneuver else None) for e in ref.events]
    assert got == want
    for frame, report in zip(rec["frames"], ref.reports):
        assert frame["autopilot"] == report.autopilot_engaged
        assert [t["icao"] for t in frame["targets"]] == [s.pred.icao for s in report.targets]
        for t, s in zip(frame["targets"], report.targets):
            assert t["level"] == s.level.name
            assert t["range_m"] == pytest.approx(s.pred.range_m, abs=0.06)
            assert t["tcpa_s"] == pytest.approx(s.pred.tcpa_s, abs=0.006)
            assert t["rel_alt_ft"] == pytest.approx(s.pred.rel_alt_ft, abs=0.06)
            assert t["maneuver"] == (s.resolution.maneuver.value if s.resolution else None)
            assert t["on_ground"] == s.on_ground
        assert frame["own"]["on_ground"] == report.own_on_ground


def test_frame_geometry_is_consistent():
    rec = record_scenario(SCENARIOS / "mixed_traffic.json")
    for f in rec["frames"]:
        for t in f["targets"]:
            assert (t["e_m"] ** 2 + t["n_m"] ** 2) ** 0.5 == pytest.approx(t["range_m"], abs=0.2)
            assert (t["miss_e_m"] ** 2 + t["miss_n_m"] ** 2) ** 0.5 == pytest.approx(t["hmd_m"], abs=0.2)
            assert set(t["shells"]) == {"surveillance", "watch", "ta", "ra", "threat", "core"}
            assert 0.0 <= t["proximity"] <= 1.0
            # an alert shown on the scope always has its ring lit, except while an RA is being held
            if t["level"] == "TA":
                assert t["shells"]["ta"]["inside"]
            if t["level"] == "RA" and t["threat"]:
                assert t["shells"]["ra"]["inside"]


def test_ra_fires_on_the_cycle_the_target_crosses_the_ra_ring():
    rec = record_scenario(SCENARIOS / "head_on.json")
    ra_time = next(f["t"] for f in rec["frames"] for e in f["events"] if e["kind"] == "RA")
    before = next(f for f in rec["frames"] if f["t"] == ra_time - 1)["targets"][0]
    at_ra = next(f for f in rec["frames"] if f["t"] == ra_time)["targets"][0]
    assert before["shells"]["ra"]["margin_m"] > 0 and not before["shells"]["ra"]["inside"]
    assert at_ra["shells"]["ra"]["margin_m"] <= 0 and at_ra["shells"]["ra"]["inside"]
    assert at_ra["shells"]["ra"]["radius_m"] == pytest.approx(220 * 1852 / 3600 * 35, abs=1.0)


def test_export_writes_one_self_contained_page(tmp_path):
    out = export_html([SCENARIOS / "head_on.json", SCENARIOS / "diverging.json"], tmp_path / "scope.html")
    page = out.read_text(encoding="utf-8")
    assert page.lstrip().lower().startswith("<!doctype html>")
    assert DATA_PLACEHOLDER not in page
    start = page.index("const DATA = ") + len("const DATA = ")
    payload = json.loads(page[start:page.index(";\n", start)].replace("<\\/", "</"))
    assert [r["file"] for r in payload["recordings"]] == ["head_on.json", "diverging.json"]
    assert payload["config"]["threat_hmd_nm"] == DEFAULT.threat_hmd_nm
    assert [s["key"] for s in payload["shells"]] == ["surveillance", "watch", "ta", "ra", "threat", "core"]
    assert payload["live"] is None
    assert "</script>" not in page[start:page.index(";\n", start)]


def test_render_html_escapes_script_end_tags():
    payload = build_payload([{"file": "x", "name": "</script><b>", "description": "", "duration_s": 0,
                              "pilot": {}, "frames": []}])
    page = render_html(payload)
    assert page.count("</script>") == 1


def test_cli_writes_the_file(tmp_path, capsys):
    out = tmp_path / "s.html"
    assert main(["--scenario", str(SCENARIOS / "overtaking.json"), "--out", str(out), "--no-open"]) == 0
    assert out.stat().st_size > 50_000
    assert "Recorded 1 scenario" in capsys.readouterr().out


class _Scripted(TrafficSource):
    """An aircraft 1.6 mi north, flying south at 120 kt toward a fixed ownship (about 42 s out)."""

    def __init__(self, t0):
        self.t0 = t0

    def poll(self, now):
        north_m = 1.6 * 1609.344 - 120 * 1852 / 3600 * (now - self.t0)
        return [Target("abc123", 39.57 + north_m / 111_050.0, -104.85, 7500.0, 120.0, 180.0, 0.0, now)]


def test_live_feed_and_server():
    clock = {"t": 1000.0}
    lines = []
    feed = LiveFeed(_Scripted(1000.0), 39.57, -104.85, 7500.0, keep=5, clock=lambda: clock["t"], out=lines.append)
    for _ in range(8):
        feed.step()
        clock["t"] += 1.0
    assert feed.seq == 8 and len(feed.frames) == 5                 # only the newest are kept
    last = feed.frames[-1]
    assert last["own"]["icao"] == DEFAULT.live_ownship_icao and last["autopilot"] is False
    assert last["targets"][0]["icao"] == "abc123" and last["targets"][0]["level"] in ("TA", "RA")
    assert any("TRAFFIC ADVISORY" in line for line in lines)
    assert [f["t"] for f in feed.since(6)["frames"]] == [6.0, 7.0]
    assert len(feed.since(0)["frames"]) == 5 and feed.since(8)["frames"] == []

    server = make_server(feed, port=0)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/") as r:
            page = r.read().decode()
        assert '"live":{"lat":39.57' in page and DATA_PLACEHOLDER not in page
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/live.json?since=7") as r:
            body = json.load(r)
        assert body["seq"] == 8 and len(body["frames"]) == 1
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(f"http://127.0.0.1:{port}/etc/passwd")
    finally:
        server.shutdown()


def test_standalone_page_is_set_up_for_a_phone():
    page = render_html(build_payload([record_scenario(SCENARIOS / "head_on.json")]))
    head = page[:page.index("</head>")]
    assert 'name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"' in head
    assert 'name="apple-mobile-web-app-capable" content="yes"' in head and 'name="theme-color"' in head
    # The fragment (for a host that supplies its own document) carries no wrapper of its own.
    fragment = render_fragment(build_payload([]))
    assert fragment.lstrip().startswith("<title>") and "<html" not in fragment and "<body" not in fragment


def test_recorded_page_can_be_served_to_a_phone(tmp_path):
    out = export_html([SCENARIOS / "ground_traffic.json"], tmp_path / "scope.html")
    server = make_server(page=out.read_text(encoding="utf-8"), port=0)
    assert server.server_address[0] == "127.0.0.1"          # this computer only unless --lan is given
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/") as r:
            assert r.read().decode() == out.read_text(encoding="utf-8")
        with pytest.raises(urllib.error.HTTPError):          # a recorded page has no live feed
            urllib.request.urlopen(f"http://127.0.0.1:{port}/live.json")
    finally:
        server.shutdown()
        server.server_close()
    with pytest.raises(ValueError):
        make_server()
    assert lan_address() is None or lan_address().count(".") == 3


def test_live_feed_passes_ground_settings_to_the_page():
    feed = LiveFeed(_Scripted(1000.0), 39.57, -104.85, 5400.0, clock=lambda: 1000.0, out=lambda s: None,
                    on_ground=True, ground_elev_ft=5400.0)
    frame = feed.step()
    assert frame["own"]["on_ground"] and frame["own"]["agl_ft"] == 0.0
    assert all(t["level"] in ("NONE", "TA") for t in frame["targets"])
    server = make_server(feed, port=0)
    try:
        threading.Thread(target=server.serve_forever, daemon=True).start()
        with urllib.request.urlopen(f"http://127.0.0.1:{server.server_address[1]}/") as r:
            page = r.read().decode()
        assert '"on_ground":true' in page and '"field_elev_ft":5400.0' in page
    finally:
        server.shutdown()
        server.server_close()

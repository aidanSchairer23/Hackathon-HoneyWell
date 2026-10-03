"""Traffic scope: record what the engine sees and show it in a browser.

    python -m gatcas.viz                                   # every scenario in ./scenarios
    python -m gatcas.viz --scenario scenarios/head_on.json
    python -m gatcas.viz --dump1090 URL --lat LAT --lon LON --alt FT
    python -m gatcas.viz --serve --lan                     # open it on a phone on the same Wi-Fi

Scenario mode runs each scenario through the normal simulation loop
(main.run_simulation, unchanged), saves one frame per 1 Hz cycle, and writes a
single self-contained HTML file that replays them. No server is needed; the
file opens by double-click.

Live mode polls dump1090 exactly as `python -m gatcas --dump1090` does, serves
the same page on localhost, and streams a frame per cycle to it. As in the CLI,
live mode is advisory only: there is no autopilot.

The page is laid out for a phone held upright: the picture takes about 70 % of
the screen and the traffic alert the rest. --serve hosts the recorded page over
HTTP instead of opening a file, and --lan makes either server reachable from
other devices on the local network (the default is this computer only).

This module only observes. It never changes an alert, an RA or the aircraft.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Iterable

from .alerting import is_threat
from .audio import Announcer
from .config import DEFAULT, Config
from .main import CasSystem, CycleReport, emit, run_simulation
from .prediction import geodetic_to_enu, relative_state
from .surveillance import build_tracks
from .traffic import Dump1090Source, SimulatedSource, Target, TrafficSource
from .zones import innermost, proximity, shell_states, shells

TEMPLATE = Path(__file__).resolve().parent / "web" / "scope.html"
DATA_PLACEHOLDER = "/*__GATCAS_DATA__*/null"
FORMAT_VERSION = 1


def _r(x: float, digits: int = 1) -> float:
    return round(float(x), digits)


def snapshot(cfg: Config, now: float, own: Target, reports: Iterable[Target],
             report: CycleReport, ground_elev_ft: float | None = None) -> dict:
    """One display frame for one cycle: ownship, every tracked target with its
    prediction, alert state and shell states, plus the cycle's events.

    Tracks are rebuilt with surveillance.build_tracks from the same inputs the
    engine was given, so the target states shown are the ones it evaluated.
    ground_elev_ft is the field elevation the engine was given, if any; it is
    used only to show height above the ground and the descend floor.
    """
    tracks = {tr.target.icao: tr for tr in build_tracks(own, reports, now, cfg)}
    targets = []
    for st in report.targets:
        pred = st.pred
        tr = tracks[pred.icao]
        tgt = tr.target
        rel_pos, own_vel, intr_vel = relative_state(own, tgt)
        rel_vel = intr_vel - own_vel
        rel_speed = math.hypot(rel_vel[0], rel_vel[1])
        # Range rate: negative while closing.
        range_rate = (float(rel_pos[0] * rel_vel[0] + rel_pos[1] * rel_vel[1]) / pred.range_m
                      if pred.range_m > 1e-6 else 0.0)
        miss = rel_pos + rel_vel * pred.tcpa_s      # where the target will be, relative to us, at CPA
        states = shell_states(pred, rel_speed, cfg, alertable=not st.on_ground)
        targets.append({
            "icao": pred.icao,
            "e_m": _r(rel_pos[0]), "n_m": _r(rel_pos[1]),
            "alt_ft": _r(tgt.alt_ft), "gs_kt": _r(tgt.gs_kt), "track_deg": _r(tgt.track_deg),
            "vs_fpm": _r(tgt.vs_fpm, 0),
            "range_m": _r(pred.range_m), "bearing_deg": _r(pred.bearing_deg),
            "rel_alt_ft": _r(pred.rel_alt_ft),
            "tcpa_s": _r(pred.tcpa_s, 2), "hmd_m": _r(pred.hmd_m), "vmd_ft": _r(pred.vmd_ft),
            "miss_e_m": _r(miss[0]), "miss_n_m": _r(miss[1]),
            "rel_speed_mps": _r(rel_speed, 2), "range_rate_mps": _r(range_rate, 2),
            "threat": bool(is_threat(pred, cfg)) and not st.on_ground,
            "on_ground": bool(st.on_ground),
            "level": st.level.name,
            "maneuver": st.resolution.maneuver.value if st.resolution else None,
            "scores": ({m.value: _r(s, 2) for m, s in st.resolution.scores.items()}
                       if st.resolution else None),
            "coordinated": bool(st.resolution.coordinated) if st.resolution else None,
            "basis": st.resolution.basis if st.resolution else None,
            "report_age_s": _r(tr.report_age_s, 2),
            "proximity": _r(proximity(pred, cfg), 3),
            "innermost": innermost(states),
            "shells": {s.key: {"applies": bool(s.applies), "inside": bool(s.inside),
                               "radius_m": None if s.radius_m is None else _r(s.radius_m),
                               "margin_m": None if s.margin_m is None else _r(s.margin_m)}
                       for s in states},
        })
    return {
        "t": _r(now, 2),
        "own": {"icao": own.icao, "lat": round(own.lat, 6), "lon": round(own.lon, 6),
                "alt_ft": _r(own.alt_ft), "gs_kt": _r(own.gs_kt),
                "track_deg": _r(own.track_deg, 2), "vs_fpm": _r(own.vs_fpm, 0),
                "on_ground": bool(own.on_ground),
                "agl_ft": None if ground_elev_ft is None else _r(own.alt_ft - ground_elev_ft),
                # Mirrors main.CasSystem.cycle: below this height a descend RA is not issued.
                "descend_ok": bool(ground_elev_ft is None
                                   or own.alt_ft - ground_elev_ft >= cfg.ra_descend_min_agl_ft)},
        "active_ra": ({"icao": report.active_ra[0], "maneuver": report.active_ra[1].value}
                      if report.active_ra else None),
        "autopilot": bool(report.autopilot_engaged),
        "targets": targets,
        "events": [{"kind": e.kind, "icao": e.icao, "phrase": e.phrase,
                    "maneuver": e.maneuver.value if e.maneuver else None, "detail": e.detail}
                   for e in report.events],
    }


class _RecordingSource:
    """Wraps a SimulatedSource and notes the inputs of every cycle.

    run_simulation calls ownship_target() and then poll() once per cycle; this
    proxy remembers both and passes everything else straight through, so the
    simulation runs exactly as it does from the CLI.
    """

    def __init__(self, source: SimulatedSource):
        self._source = source
        self.inputs: list[tuple[float, Target, list[Target]]] = []
        self._own: Target | None = None

    def __getattr__(self, name):
        return getattr(self._source, name)

    def ownship_target(self) -> Target:
        self._own = self._source.ownship_target()
        return self._own

    def poll(self, now: float) -> list[Target]:
        reports = self._source.poll(now)
        self.inputs.append((now, self._own, reports))
        return reports


def record_scenario(path: str | Path, cfg: Config = DEFAULT) -> dict:
    """Run one scenario through the unmodified simulation loop and return its frames."""
    path = Path(path)
    with open(path, encoding="utf-8") as f:
        spec = json.load(f)
    recorder = _RecordingSource(SimulatedSource(spec, cfg))
    result = run_simulation(recorder, fast=True, cfg=cfg, announcer=None, out=lambda line: None)
    if len(recorder.inputs) != len(result.reports):
        raise RuntimeError("recorded cycle inputs do not line up with the cycle reports")
    elev = recorder.field_elev_ft                   # what run_simulation gave the engine
    frames = [snapshot(cfg, now, own, reports, report, elev)
              for (now, own, reports), report in zip(recorder.inputs, result.reports)]
    pilot = spec.get("pilot", {})
    return {
        "file": path.name,
        "name": spec.get("name", path.stem),
        "description": spec.get("description", ""),
        "duration_s": float(spec.get("duration_s", 120.0)),
        "field_elev_ft": elev,
        "pilot": {"response": pilot.get("response", "ignore"), "delay_s": float(pilot.get("delay_s", 2.0))},
        "frames": frames,
    }


def build_payload(recordings: list[dict], cfg: Config = DEFAULT, live: dict | None = None) -> dict:
    """Everything the page needs: thresholds, shell objects and the recordings."""
    return {
        "version": FORMAT_VERSION,
        "config": dataclasses.asdict(cfg),
        "shells": [dataclasses.asdict(s) for s in shells(cfg)],
        "recordings": recordings,
        "live": live,
    }


def render_fragment(payload: dict) -> str:
    """The scope page content (title, style, markup, script) with the payload
    embedded, without the <html>/<head>/<body> wrapper. Use this when the host
    supplies its own document skeleton."""
    template = TEMPLATE.read_text(encoding="utf-8")
    if DATA_PLACEHOLDER not in template:
        raise RuntimeError(f"{TEMPLATE} has no data placeholder")
    # "</" must not appear inside a <script> block; "<\\/" is the same JSON string.
    data = json.dumps(payload, separators=(",", ":")).replace("</", "<\\/")
    return template.replace(DATA_PLACEHOLDER, data)


def render_html(payload: dict) -> str:
    """The scope page as one self-contained document that opens by double-click."""
    return ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
            '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
            # Lets "Add to Home Screen" open it full screen, like an app.
            '<meta name="mobile-web-app-capable" content="yes">\n'
            '<meta name="apple-mobile-web-app-capable" content="yes">\n'
            '<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">\n'
            '<meta name="apple-mobile-web-app-title" content="GATCAS">\n'
            '<meta name="theme-color" content="#0b0e12">\n'
            "</head>\n<body>\n" + render_fragment(payload) + "\n</body>\n</html>\n")


def export_html(scenario_paths: Iterable[str | Path], out: str | Path, cfg: Config = DEFAULT,
                fragment: bool = False) -> Path:
    recordings = [record_scenario(p, cfg) for p in scenario_paths]
    out = Path(out)
    render = render_fragment if fragment else render_html
    out.write_text(render(build_payload(recordings, cfg)), encoding="utf-8")
    return out


# --------------------------------------------------------------------------- #
# Live mode
# --------------------------------------------------------------------------- #

class LiveFeed:
    """Runs the engine against a live source and keeps the most recent frames."""

    def __init__(self, source: TrafficSource, lat: float, lon: float, alt_ft: float,
                 cfg: Config = DEFAULT, keep: int = 900, announcer: Announcer | None = None,
                 clock=time.time, out=print, on_ground: bool = False,
                 ground_elev_ft: float | None = None):
        self.source, self.cfg = source, cfg
        self.lat, self.lon, self.alt_ft = lat, lon, alt_ft
        self.keep, self.announcer, self.clock, self.out = keep, announcer, clock, out
        self.on_ground, self.ground_elev_ft = on_ground, ground_elev_ft
        # Advisory only, as in main.run_live.
        self.cas = CasSystem(cfg, autopilot=None, ground_elev_ft=ground_elev_ft)
        self.frames: list[dict] = []
        self.seq = 0                                   # number of frames produced so far
        self._lock = threading.Lock()
        self._start: float | None = None

    def step(self) -> dict:
        """One cycle. Mirrors the body of main.run_live's loop."""
        now = self.clock()
        if self._start is None:
            self._start = now
        own = Target(self.cfg.live_ownship_icao, self.lat, self.lon, self.alt_ft, 0.0, 0.0, 0.0, now,
                     self.on_ground)
        reports = self.source.poll(now)
        report = self.cas.cycle(now, own, reports)
        emit(report, now - self._start, self.out, self.announcer)
        frame = snapshot(self.cfg, now, own, reports, report, self.ground_elev_ft)
        frame["t"] = _r(now - self._start, 2)
        with self._lock:
            self.frames.append(frame)
            del self.frames[:-self.keep]
            self.seq += 1
        return frame

    def since(self, seq: int) -> dict:
        """Frames produced after sequence number `seq` (at most `keep` of them)."""
        with self._lock:
            first = self.seq - len(self.frames)
            start = max(seq, first)
            return {"seq": self.seq, "frames": self.frames[start - first:]}

    def run(self, stop: threading.Event) -> None:
        period = self.cfg.cycle_period_s
        k = 0
        start = self.clock()
        while not stop.is_set():
            self.step()
            k += 1
            stop.wait(max(0.0, start + k * period - self.clock()))


def lan_address() -> str | None:
    """This computer's address on the local network, or None if it has none."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))     # UDP: nothing is sent, it only picks the outgoing interface
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def make_server(feed: LiveFeed | None = None, host: str = "127.0.0.1", port: int = 8765,
                page: str | None = None) -> ThreadingHTTPServer:
    """HTTP server for the scope page.

    With a feed it serves the live page and /live.json. With page (a recorded
    document from render_html) it serves that page only. host defaults to this
    computer; pass "0.0.0.0" to let a phone on the same network open it.
    """
    if (feed is None) == (page is None):
        raise ValueError("pass a live feed or a recorded page, not both")
    if feed is not None:
        page = render_html(build_payload([], feed.cfg, live={
            "lat": feed.lat, "lon": feed.lon, "alt_ft": feed.alt_ft, "period_s": feed.cfg.cycle_period_s,
            "on_ground": feed.on_ground, "field_elev_ft": feed.ground_elev_ft,
        }))
    page = page.encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.split("?")[0] in ("/", "/index.html"):
                body, ctype = page, "text/html; charset=utf-8"
            elif feed is not None and self.path.startswith("/live.json"):
                try:
                    seq = int(self.path.split("since=")[1]) if "since=" in self.path else 0
                except ValueError:
                    seq = 0
                body, ctype = json.dumps(feed.since(seq), separators=(",", ":")).encode(), "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):   # keep the engine's log readable
            pass

    return ThreadingHTTPServer((host, port), Handler)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m gatcas.viz",
        description="GATCAS traffic scope (simulation and demo only).")
    parser.add_argument("--scenario", metavar="FILE", action="append",
                        help="scenario JSON to record (repeatable; default: every file in ./scenarios)")
    parser.add_argument("--out", metavar="FILE", default="gatcas_scope.html", help="HTML file to write")
    parser.add_argument("--no-open", action="store_true", help="do not open the browser")
    parser.add_argument("--fragment", action="store_true",
                        help="write page content only, for a host that adds its own <html> wrapper")
    parser.add_argument("--dump1090", metavar="URL", help="live mode: dump1090 aircraft.json URL")
    parser.add_argument("--lat", type=float, help="live mode: ownship latitude, degrees")
    parser.add_argument("--lon", type=float, help="live mode: ownship longitude, degrees")
    parser.add_argument("--alt", type=float, help="live mode: ownship pressure altitude, feet")
    parser.add_argument("--field-elev", type=float, metavar="FT",
                        help="live mode: field elevation, feet. Shows aircraft on the ground and sets the descend floor")
    parser.add_argument("--on-ground", action="store_true",
                        help="live mode: the fixed ownship is on the ground (traffic advisories only)")
    parser.add_argument("--serve", action="store_true",
                        help="scenario mode: serve the recorded page over HTTP instead of opening the file")
    parser.add_argument("--lan", action="store_true",
                        help="let other devices on this network (a phone) open the page; default is this computer only")
    parser.add_argument("--port", type=int, default=8765, help="port for --serve and live mode")
    parser.add_argument("--mute", action="store_true", help="live mode: print alerts instead of speaking them")
    args = parser.parse_args(argv)
    host = "0.0.0.0" if args.lan else "127.0.0.1"

    if args.dump1090:
        if None in (args.lat, args.lon, args.alt):
            parser.error("--dump1090 requires --lat, --lon and --alt")
        if args.scenario:
            parser.error("--scenario and --dump1090 cannot be combined")
        announcer = Announcer(speech=not args.mute)     # spoken alerts, as in `python -m gatcas --dump1090`
        feed = LiveFeed(Dump1090Source(args.dump1090, ground_alt_ft=args.field_elev),
                        args.lat, args.lon, args.alt, announcer=announcer,
                        on_ground=args.on_ground, ground_elev_ft=args.field_elev)
        server = make_server(feed, host=host, port=args.port)
        stop = threading.Event()
        threading.Thread(target=server.serve_forever, daemon=True).start()
        url = f"http://127.0.0.1:{args.port}/"
        print(f"LIVE MODE - advisory only, autopilot disabled. Scope at {url}  (Ctrl+C to stop)")
        _print_phone_url(args)
        if not args.no_open:
            webbrowser.open(url)
        try:
            feed.run(stop)
        except KeyboardInterrupt:
            print("Stopped.")
        finally:
            stop.set()
            server.shutdown()
            announcer.close()
        return 0

    paths = [Path(p) for p in args.scenario] if args.scenario else sorted(Path("scenarios").glob("*.json"))
    if not paths:
        parser.error("no scenario files found; pass --scenario FILE")
    out = export_html(paths, args.out, fragment=args.fragment)
    print(f"Recorded {len(paths)} scenario(s) to {out} ({out.stat().st_size / 1024:.0f} KB).")
    if args.serve or args.lan:
        if args.fragment:
            parser.error("--fragment writes page content only and cannot be served")
        server = make_server(page=out.read_text(encoding="utf-8"), host=host, port=args.port)
        url = f"http://127.0.0.1:{args.port}/"
        print(f"Serving the scope at {url}  (Ctrl+C to stop)")
        _print_phone_url(args)
        if not args.no_open:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("Stopped.")
        finally:
            server.server_close()
        return 0
    if not args.no_open and not args.fragment:
        webbrowser.open(out.resolve().as_uri())
    return 0


def _print_phone_url(args) -> None:
    if not args.lan:
        return
    ip = lan_address()
    if ip:
        print(f"On a phone on the same Wi-Fi, open http://{ip}:{args.port}/ "
              "(then use the browser's Add to Home Screen to keep it as an app).")
    else:
        print("--lan: no local network address found; connect this computer to Wi-Fi first.")


if __name__ == "__main__":
    raise SystemExit(main())

"""Alert phrases and spoken output.

Phrase building is pure. Announcer speaks with pyttsx3 on a background
thread so a long phrase never stalls the 1 Hz cycle, and prints every phrase
as well. If speech is disabled, pyttsx3 cannot start, or speaking fails (for
example, no audio device), it falls back to printing only.
"""

from __future__ import annotations

import logging
import queue
import threading
from typing import Callable

from .config import DEFAULT, Config
from .prediction import Prediction
from .resolution import Maneuver

log = logging.getLogger(__name__)

RA_PHRASES = {
    Maneuver.CLIMB: "Climb, climb",
    Maneuver.DESCEND: "Descend, descend",
    Maneuver.TURN_LEFT: "Turn left",
    Maneuver.TURN_RIGHT: "Turn right",
}
AUTOPILOT_PHRASE = "Autopilot engaging"
CLEAR_PHRASE = "Clear of conflict"


def clock_position(own_track_deg: float, bearing_deg: float) -> int:
    """Clock position (1-12) of a bearing relative to ownship's track."""
    hour = round(((bearing_deg - own_track_deg) % 360.0) / 30.0) % 12
    return 12 if hour == 0 else hour


def altitude_word(rel_alt_ft: float, level_band_ft: float) -> str:
    if rel_alt_ft > level_band_ft:
        return "high"
    if rel_alt_ft < -level_band_ft:
        return "low"
    return "level"


def distance_words(range_mi: float) -> str:
    miles = round(range_mi)
    if miles < 1:
        return "less than 1 mile"
    return "1 mile" if miles == 1 else f"{miles} miles"


def ta_phrase(pred: Prediction, own_track_deg: float, cfg: Config = DEFAULT) -> str:
    """'Traffic, [clock position], [high/low/level], [distance] miles.'"""
    clock = clock_position(own_track_deg, pred.bearing_deg)
    return (f"Traffic, {clock} o'clock, {altitude_word(pred.rel_alt_ft, cfg.ta_level_band_ft)}, "
            f"{distance_words(pred.range_mi)}.")


def ra_phrase(maneuver: Maneuver) -> str:
    return RA_PHRASES[maneuver]


class Announcer:
    def __init__(self, speech: bool = True, out: Callable[[str], None] = print,
                 init_timeout_s: float = 5.0):
        self.out = out
        self.speech_available = False
        self._queue: queue.Queue[str | None] = queue.Queue()
        self._thread: threading.Thread | None = None
        if speech:
            ready = threading.Event()
            self._thread = threading.Thread(target=self._worker, args=(ready,), daemon=True)
            self._thread.start()
            ready.wait(init_timeout_s)
            if not self.speech_available:
                self.out("Audio unavailable: alerts will be printed only.")

    def say(self, phrase: str) -> None:
        if self.speech_available:
            self.out(f"    AUDIO: {phrase}")
            self._queue.put(phrase)
        else:
            self.out(f"    AUDIO (printed, no speech): {phrase}")

    def close(self, timeout_s: float = 15.0) -> None:
        """Let queued speech finish, then stop the worker."""
        if self._thread is not None:
            self._queue.put(None)
            self._thread.join(timeout_s)

    def _worker(self, ready: threading.Event) -> None:
        try:
            import pyttsx3
            engine = pyttsx3.init()
        except Exception as exc:  # no engine, no driver, no device
            log.debug("pyttsx3 unavailable: %r", exc)
            ready.set()
            return
        self.speech_available = True
        ready.set()
        while (phrase := self._queue.get()) is not None:
            try:
                engine.say(phrase)
                engine.runAndWait()
            except Exception as exc:
                self.speech_available = False
                self.out(f"    AUDIO (speech failed: {exc!r}; printed only): {phrase}")

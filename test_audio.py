import sys

from gatcas.audio import (Announcer, altitude_word, clock_position, distance_words, ra_phrase,
                          ta_phrase)
from gatcas.prediction import predict
from gatcas.resolution import Maneuver
from helpers import MI, at, own


def test_clock_position_is_relative_to_track():
    assert clock_position(0, 0) == 12
    assert clock_position(0, 90) == 3
    assert clock_position(90, 0) == 9
    assert clock_position(350, 10) == 1      # 20 degrees right rounds to 1 o'clock
    assert clock_position(0, 344) == 11
    assert clock_position(0, 346) == 12


def test_altitude_and_distance_words():
    assert [altitude_word(a, 300) for a in (500, 300, 0, -300, -301)] == ["high", "level", "level", "level", "low"]
    assert [distance_words(d) for d in (0.4, 0.6, 1.4, 2.6)] == ["less than 1 mile", "1 mile", "1 mile", "3 miles"]


def test_ta_phrase():
    p = predict(own(track_deg=90), at(east_m=3 * MI, alt_ft=6500, track_deg=270))
    assert ta_phrase(p, own_track_deg=90) == "Traffic, 12 o'clock, high, 3 miles."
    p = predict(own(track_deg=0), at(east_m=-2 * MI, alt_ft=6100, track_deg=90))
    assert ta_phrase(p, own_track_deg=0) == "Traffic, 9 o'clock, level, 2 miles."


def test_ra_phrases():
    assert ra_phrase(Maneuver.CLIMB) == "Climb, climb"
    assert ra_phrase(Maneuver.DESCEND) == "Descend, descend"
    assert ra_phrase(Maneuver.TURN_RIGHT) == "Turn right"
    assert ra_phrase(Maneuver.TURN_LEFT) == "Turn left"


def test_console_fallback_when_speech_disabled():
    lines = []
    a = Announcer(speech=False, out=lines.append)
    a.say("Climb, climb")
    a.close()
    assert lines == ["    AUDIO (printed, no speech): Climb, climb"]


def test_console_fallback_when_pyttsx3_cannot_start(monkeypatch):
    class Broken:
        @staticmethod
        def init():
            raise OSError("no audio device")
    monkeypatch.setitem(sys.modules, "pyttsx3", Broken)
    lines = []
    a = Announcer(speech=True, out=lines.append)
    a.say("Turn right")
    a.close()
    assert not a.speech_available
    assert lines == ["Audio unavailable: alerts will be printed only.",
                     "    AUDIO (printed, no speech): Turn right"]

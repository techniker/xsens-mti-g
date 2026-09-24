#!/usr/bin/env python3
"""Vario audio envelope tests — no hardware or audio device required.

A click or crackle in synthesised audio is an abrupt step in the waveform.
These tests measure the size of that step at every point where a tone starts
or stops and assert it stays small, i.e. that an amplitude envelope actually
ramps the tone in and out.

Run: python test_vario_audio.py
"""
import sys
sys.path.insert(0, '.')

from PyQt6.QtCore import QCoreApplication

_app = QCoreApplication([])          # VarioAudio builds a QTimer

import vario_audio as V

FULL_SCALE = 32000
# The gate closes over FADE_MS, so the last sample before silence cannot
# exceed one gate step of full scale, plus rounding.
FADE_SAMPLES = int(V.FADE_MS / 1000.0 * V.SAMPLE_RATE)
MAX_STEP = FULL_SCALE / FADE_SAMPLES + 8

failures = []


def check(name, actual, limit):
    ok = actual <= limit
    fmt = "{:.3f}" if limit < 1 else "{:.0f}"
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {fmt.format(actual)} "
          f"(limit {fmt.format(limit)})")
    if not ok:
        failures.append(name)


def _gen(vsi_seq):
    """Feed a sequence of (vsi, n_samples) through the generator, as the
    audio timer would, and return the concatenated stream."""
    v = V.VarioAudio()
    v._volume = 1.0                  # worst case for step size
    out = []
    for vsi, n in vsi_seq:
        out.extend(v._generate(vsi, n))
    return out


# A sine wave passing through zero produces a one-sample "0", which is not
# silence. Only a run this long counts as the tone actually having stopped.
MIN_SILENCE = 256


def _last_nonzero(s):
    """Absolute value of the final non-zero sample — the step down to
    silence, which is exactly what a click is."""
    for x in reversed(s):
        if x != 0:
            return abs(x)
    return 0


def _steps_into_silence(s):
    """Sizes of the steps from a non-zero sample into a genuine silence run.

    Requiring MIN_SILENCE consecutive zeros is what separates a real tone
    edge from the sine's own zero crossings.
    """
    steps = []
    i = 0
    n = len(s)
    while i < n:
        if s[i] == 0:
            j = i
            while j < n and s[j] == 0:
                j += 1
            if j - i >= MIN_SILENCE and i > 0:
                steps.append(abs(s[i - 1]))
            i = j
        else:
            i += 1
    return steps


def test_tone_end_fades():
    """A tone must ramp to zero however it ends.

    Sink is a continuous tone, so entering the dead band is the ONLY way it
    ever stops. A climb beep can also be cut mid-beep by the dead band. The
    cut offset is swept because a single offset can land near a zero
    crossing by luck and hide a full-scale step.
    """
    for vsi, label in ((-3.0, "sink -3.0"), (3.0, "climb +3.0")):
        worst = 0
        for lead in range(4000, 4064):
            s = _gen([(vsi, lead), (0.0, 4 * FADE_SAMPLES)])
            worst = max(worst, _last_nonzero(s))
        check(f"end of {label} tone ramps to silence", worst, MAX_STEP)


def test_tone_start_fades():
    """A tone must ramp up from zero, not begin at full amplitude."""
    for vsi, label in ((-3.0, "sink -3.0"), (3.0, "climb +3.0")):
        s = _gen([(0.0, 2000), (vsi, 4 * FADE_SAMPLES)])
        # A tenth of the way into the fade, the envelope must still be low.
        head = s[:len(s) - 4 * FADE_SAMPLES + FADE_SAMPLES // 10]
        peak = max((abs(x) for x in head), default=0)
        check(f"start of {label} tone ramps up from silence",
              peak, FULL_SCALE * 0.2)


def test_beep_edges_fade():
    """Regression guard: the per-beep edges inside a steady climb were
    already clean and must stay clean."""
    s = _gen([(3.0, 3 * V.SAMPLE_RATE)])
    steps = _steps_into_silence(s)
    assert len(steps) >= 5, f"expected several beep edges, saw {len(steps)}"
    check(f"climb beep edges ramp to silence ({len(steps)} edges)",
          max(steps), MAX_STEP)


def test_beep_duty_cycle():
    """The beep rhythm carries climb-rate information, so the tone-on
    fraction must stay at DUTY. The fade has to fit *inside* the tone-on
    window; letting it run past shortens the gaps and speeds the rhythm up.
    """
    for vsi in (0.5, 3.0, 5.0):
        s = _gen([(vsi, 3 * V.SAMPLE_RATE)])
        runs = []
        i, n = 0, len(s)
        while i < n:
            if s[i] == 0:
                j = i
                while j < n and s[j] == 0:
                    j += 1
                if j - i >= MIN_SILENCE:
                    runs.append((i, j - i))
                i = j
            else:
                i += 1
        assert len(runs) >= 4, f"vsi={vsi}: too few beeps ({len(runs)})"
        # Whole cycles only: start of one silence run to the start of the next
        period = (runs[-1][0] - runs[0][0]) / (len(runs) - 1)
        off = sum(r[1] for r in runs[1:-1]) / (len(runs) - 2)
        duty = 1.0 - off / period
        print(f"      vsi={vsi:+.1f}  period={period / V.SAMPLE_RATE * 1000:6.1f} ms  "
              f"duty={duty:.3f}")
        check(f"beep duty at vsi={vsi:+.1f} matches DUTY={V.DUTY}",
              abs(duty - V.DUTY), 0.04)


def test_deadband_is_silent():
    """Regression guard: the dead band must still end up fully silent."""
    s = _gen([(3.0, 4000), (0.0, 8 * FADE_SAMPLES)])
    tail = s[-FADE_SAMPLES:]
    check("dead band settles to true silence",
          max((abs(x) for x in tail), default=0), 0)


class _StubSink:
    """Stand-in for QAudioSink so the shutdown logic can be tested without an
    audio device."""

    def __init__(self):
        self.stopped = False

    def stop(self):
        self.stopped = True


def test_closing_fades_out_while_still_climbing():
    """Switching the vario off must ramp the tone down even though the
    aircraft is still climbing, so the VSI cannot hold the gate open.

    QAudioSink.stop() discards whatever is queued, so without this the
    waveform is cut mid-cycle — the same full-scale step as a dead-band cut.
    """
    v = V.VarioAudio()
    v._volume = 1.0
    body = v._generate(3.0, 4000)
    assert max(abs(x) for x in body) > 20000, "tone should be sounding first"

    v._closing = True
    tail = []
    for _ in range(4):
        tail.extend(v._generate(3.0, FADE_SAMPLES))   # VSI still says climb

    check("switching off ramps the tone to silence",
          _last_nonzero(body + tail), MAX_STEP)
    check("switching off reaches true silence",
          max((abs(x) for x in tail[-FADE_SAMPLES:]), default=0), 0)


def test_stale_release_is_ignored():
    """Switching back on mid-fade must not let the already-scheduled release
    tear down the device that is now in use again."""
    v = V.VarioAudio()
    v._sink = _StubSink()
    v._closing = True
    stale = v._stop_token

    v._stop_token += 1            # what _start() does when it cancels
    v._closing = False
    v._hard_stop(stale)           # the release scheduled before that
    check("stale release leaves the live sink alone",
          0 if (v._sink is not None and not v._sink.stopped) else 1, 0)

    # ...while a current release does stop it
    v._hard_stop(v._stop_token)
    check("current release does stop the sink", 0 if v._sink is None else 1, 0)


def test_buffer_is_shallow():
    """The queue depth is the latency the pilot hears, and also the underrun
    margin — it must stay small but several feed intervals deep."""
    v = V.VarioAudio()
    ms = v._buffer_samples / V.SAMPLE_RATE * 1000
    print(f"      buffer = {v._buffer_samples} samples = {ms:.0f} ms "
          f"= {ms / V.FEED_INTERVAL_MS:.1f} feed intervals")
    # The queue depth is also how long sound continues after switch-off.
    check("audio latency / switch-off tail stays under 100 ms", ms, 100)
    check("buffer keeps >=3 feed intervals of underrun margin",
          -(ms / V.FEED_INTERVAL_MS), -3.0)


if __name__ == "__main__":
    print(f"fade = {V.FADE_MS} ms = {FADE_SAMPLES} samples, "
          f"max tolerated step = {MAX_STEP:.0f} of {FULL_SCALE}\n")
    test_tone_end_fades()
    test_tone_start_fades()
    test_beep_edges_fade()
    test_beep_duty_cycle()
    test_deadband_is_silent()
    test_closing_fades_out_while_still_climbing()
    test_stale_release_is_ignored()
    test_buffer_is_shallow()
    print()
    if failures:
        print("FAILED: " + ", ".join(failures))
        sys.exit(1)
    print("all vario audio envelope tests passed")

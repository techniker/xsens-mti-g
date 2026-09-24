#!/usr/bin/env python3
"""
Variometer audio synthesiser — glider-style beeping vario.

Tone parameters follow XCSoar / XCVario conventions:
  - Climb: beeping tone, frequency and beep rate increase with climb rate
  - Sink:  continuous low tone, frequency decreases with sink rate
  - Dead band: configurable silence zone around zero

Reference: XCSoar VarioSynthesiser, OpenVario variod, BlueFlyVario.
"""

import math
import struct
import time
from PyQt6.QtCore import QIODevice, QByteArray, QTimer
from PyQt6.QtMultimedia import QAudio, QAudioFormat, QAudioSink


# ─── Default parameters (XCSoar-style) ───
SAMPLE_RATE = 44100
FREQ_MIN = 200       # Hz at max sink (-5 m/s)
FREQ_ZERO = 500      # Hz at 0 m/s
FREQ_MAX = 1500      # Hz at max climb (+5 m/s)
CLIMB_MAX = 5.0      # m/s — frequency/beep clamp
SINK_MAX = 5.0       # m/s (positive value)

PERIOD_MIN_MS = 150   # ms at max climb (fast beep)
PERIOD_MAX_MS = 600   # ms at zero climb (slow beep)
DUTY = 0.67           # tone-on fraction

DEADBAND_LOW = -0.3   # m/s — no sound above this sink
DEADBAND_HIGH = 0.1   # m/s — no sound below this climb

FADE_MS = 15          # ms rise/fall envelope to avoid clicks

# Audio buffering. A vario is a control instrument, so the queue is kept
# shallow rather than filled as deep as the device allows. BUFFER_MS sets
# three things at once:
#   - the latency between a change in lift and the tone that reports it,
#   - how long sound continues after the vario is switched off, because
#     already-queued audio has to play out (discarding it is what clicks),
#   - the underrun margin, which must stay several feed intervals deep so a
#     late timer cannot starve the device.
# The first two want it small, the third wants it large; a shallow buffer fed
# often satisfies all three. Generation costs ~1% of a feed interval, so
# feeding at 10 ms is cheap.
#
# The feed shares the GUI thread with the 30 Hz PFD repaint, so the buffer
# has to outlast the worst repaint stall. Measured worst-case event-loop gap
# with the map and synthetic vision running is ~25 ms, so 100 ms leaves 4x
# margin for slower panel hardware. Going shallower shortens the switch-off
# tail but starts to risk dropouts, which crackle in their own right.
FEED_INTERVAL_MS = 10
BUFFER_MS = 100       # 10 feed intervals of headroom


class VarioAudio:
    """Generates variometer audio from vertical speed data."""

    def __init__(self):
        self._enabled = False
        self._vsi = 0.0  # current vertical speed in m/s
        self._volume = 0.7

        # Audio output
        fmt = QAudioFormat()
        fmt.setSampleRate(SAMPLE_RATE)
        fmt.setChannelCount(1)
        fmt.setSampleFormat(QAudioFormat.SampleFormat.Int16)
        self._format = fmt
        self._sink = None
        self._device = None
        self._buffer_samples = int(SAMPLE_RATE * BUFFER_MS / 1000)
        self._fade_samples = max(1, int(FADE_MS / 1000.0 * SAMPLE_RATE))

        # Closing down: the generator drives the gate to zero and the device
        # is released only once the fade has actually been played out.
        # _stop_token invalidates a pending release if the vario is switched
        # back on mid-fade.
        self._closing = False
        self._stop_token = 0

        # Tone state
        self._phase = 0.0          # oscillator phase [0, 2*pi)
        self._beep_phase = 0.0     # position within beep cycle (seconds)

        # Amplitude gate, 0..1, ramped over FADE_MS toward whether the tone
        # should be sounding. Persisting it across buffers is what lets a
        # tone that stops between two calls — dead band entry, which is the
        # only way a continuous sink tone ever ends — fade out instead of
        # being truncated mid-waveform.
        self._gate = 0.0
        # Held so a tone keeps its pitch and beep rate while fading out,
        # after the VSI that produced it has already gone.
        self._freq = FREQ_ZERO
        self._period_s = 0.0       # 0 = continuous (sink), >0 = beep period
        self._tone_on = 0.0

        # Buffer generation timer — feed audio at regular intervals
        self._timer = QTimer()
        self._timer.setInterval(FEED_INTERVAL_MS)
        self._timer.timeout.connect(self._feed_audio)

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, val):
        if val == self._enabled:
            return
        self._enabled = val
        if val:
            self._start()
        else:
            self._stop()

    @property
    def volume(self):
        return self._volume

    @volume.setter
    def volume(self, val):
        self._volume = max(0.0, min(1.0, val))

    def update_vsi(self, vsi_ms: float):
        """Update the vertical speed value (m/s, positive = climb)."""
        self._vsi = vsi_ms

    def _start(self):
        self._stop_token += 1      # cancel any pending release
        if self._sink is not None:
            # Switched back on while fading out — keep the device and let the
            # gate ramp back up instead of tearing the sink down and
            # rebuilding it.
            self._closing = False
            if not self._timer.isActive():
                self._timer.start()
            return

        self._sink = QAudioSink(self._format)
        self._sink.setBufferSize(self._buffer_samples * 2)   # 16-bit mono
        self._device = self._sink.start()
        self._phase = 0.0
        self._beep_phase = 0.0
        self._gate = 0.0           # fade in from silence
        self._closing = False
        self._timer.start()
        self._feed_audio()         # prime immediately rather than idling a tick

    def _stop(self):
        """Fade out, then release the device.

        QAudioSink.stop() discards whatever is still queued, cutting the
        waveform mid-cycle — the same full-scale step that makes a click. So
        the gate is driven to zero, the fade is fed to the device, and the
        sink is released only once it has drained. This is only affordable
        because BUFFER_MS is shallow; behind a full second of queued audio
        the fade would never be reached.
        """
        if self._sink is None:
            self._timer.stop()
            return

        self._closing = True
        # The device normally reports itself drained first; this is only a
        # backstop for a backend that never does.
        token = self._stop_token
        QTimer.singleShot(BUFFER_MS + FADE_MS + 4 * FEED_INTERVAL_MS,
                          lambda: self._hard_stop(token))

    def _hard_stop(self, token=None):
        """Release the audio device. Ignored if the vario was switched back
        on since this release was scheduled."""
        if token is not None and token != self._stop_token:
            return
        self._timer.stop()
        self._closing = False
        if self._sink:
            self._sink.stop()
            self._sink = None
            self._device = None

    def _feed_audio(self):
        if not self._device or not self._sink:
            return

        if self._closing and self._gate <= 0.0:
            # Fade is written. Stop feeding so the device can drain, then
            # release it — writing more silence would keep it from ever
            # reporting itself idle.
            if self._sink.state() == QAudio.State.IdleState:
                self._hard_stop(self._stop_token)
            return

        # Generate enough samples to fill available buffer space
        free = self._sink.bytesFree()
        if free < 64:
            return
        n_samples = min(free // 2, self._buffer_samples)

        if self._closing:
            # Only the fade itself has to reach the device. Topping the queue
            # back up with silence behind it would just lengthen the drain,
            # and the drain is how long sound continues after switch-off.
            n_samples = min(n_samples,
                            int(self._gate * self._fade_samples) + 1)

        vsi = self._vsi
        samples = self._generate(vsi, n_samples)
        data = QByteArray(struct.pack(f'<{len(samples)}h', *samples))
        self._device.write(data)

    def _generate(self, vsi: float, n_samples: int) -> list:
        """Synthesise n_samples of vario audio for the given VSI."""
        dt = 1.0 / SAMPLE_RATE
        fade_samples = self._fade_samples
        gate_step = 1.0 / fade_samples

        fade_s = fade_samples * dt
        # Closing overrides the VSI: the tone must go quiet even if the
        # aircraft is still climbing.
        silent = self._closing or DEADBAND_LOW <= vsi <= DEADBAND_HIGH

        # Fully faded out with nothing to sound — skip the per-sample loop
        # rather than synthesise silence 30 times a second.
        if silent and self._gate <= 0.0:
            self._beep_phase = 0.0
            return [0] * n_samples

        if not silent:
            # Frequency and beep rate track the current VSI. While the gate
            # is closing they keep their previous values, so a tone does not
            # change pitch on its way out.
            if vsi > 0:
                climb = min(vsi, CLIMB_MAX)
                self._freq = FREQ_ZERO + (climb / CLIMB_MAX) * (FREQ_MAX - FREQ_ZERO)
                # Faster beep at higher climb.
                self._period_s = (PERIOD_MIN_MS + (CLIMB_MAX - climb) / CLIMB_MAX
                                  * (PERIOD_MAX_MS - PERIOD_MIN_MS)) / 1000.0
                self._tone_on = self._period_s * DUTY
            else:
                sink = min(-vsi, SINK_MAX)
                self._freq = FREQ_ZERO - (sink / SINK_MAX) * (FREQ_ZERO - FREQ_MIN)
                self._period_s = 0.0    # continuous — no beep cycle
                self._tone_on = 0.0

        freq = self._freq
        period_s = self._period_s
        tone_on = self._tone_on
        # Where the tone-off ramp begins. Capped at half the tone-on window so
        # an over-long FADE_MS yields a quieter beep rather than no beep.
        off_at = tone_on - min(fade_s, tone_on * 0.5)
        vol = self._volume
        phase = self._phase
        beep_phase = self._beep_phase
        gate = self._gate

        out = []
        for _ in range(n_samples):
            # Should the tone be sounding on this sample?
            if silent:
                target = 0.0
            elif period_s > 0.0:
                # Beeping: on during the first DUTY fraction of the period.
                # The gate is released early enough for the ramp to finish
                # exactly at tone_on — a fade that ran past it would eat into
                # the gap and speed up the beep rhythm, which is how a pilot
                # reads climb rate by ear.
                target = 1.0 if beep_phase < off_at else 0.0
                beep_phase += dt
                if beep_phase >= period_s:
                    beep_phase -= period_s
            else:
                target = 1.0            # sink: continuous tone

            # One envelope for every edge. Ramping a persistent gate, rather
            # than shaping by position within the beep cycle, is what makes a
            # tone cut short by the dead band fade out as well: a
            # position-based envelope cannot see the tone stop between two
            # calls, and truncating a full-scale sine is the crackle.
            if gate < target:
                gate = min(target, gate + gate_step)
            elif gate > target:
                gate = max(target, gate - gate_step)

            out.append(int(math.sin(phase) * vol * gate * 32000))

            phase += 2.0 * math.pi * freq * dt
            if phase >= 2.0 * math.pi:
                phase -= 2.0 * math.pi

        self._phase = phase
        self._beep_phase = beep_phase
        self._gate = gate
        return out

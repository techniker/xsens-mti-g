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
from PyQt6.QtMultimedia import QAudioFormat, QAudioSink


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

        # Tone state
        self._phase = 0.0          # oscillator phase [0, 2*pi)
        self._beep_phase = 0.0     # position within beep cycle (seconds)

        # Buffer generation timer — feed audio at regular intervals
        self._timer = QTimer()
        self._timer.setInterval(30)  # ~30ms chunks
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
        self._sink = QAudioSink(self._format)
        self._sink.setBufferSize(SAMPLE_RATE * 2)  # 1 second buffer (16-bit mono)
        self._device = self._sink.start()
        self._phase = 0.0
        self._beep_phase = 0.0
        self._timer.start()

    def _stop(self):
        self._timer.stop()
        if self._sink:
            self._sink.stop()
            self._sink = None
            self._device = None

    def _feed_audio(self):
        if not self._device or not self._sink:
            return

        # Generate enough samples to fill available buffer space
        free = self._sink.bytesFree()
        if free < 64:
            return
        n_samples = min(free // 2, SAMPLE_RATE // 10)  # at most 100ms per call

        vsi = self._vsi
        samples = self._generate(vsi, n_samples)
        data = QByteArray(struct.pack(f'<{len(samples)}h', *samples))
        self._device.write(data)

    def _generate(self, vsi: float, n_samples: int) -> list:
        """Synthesise n_samples of vario audio for the given VSI."""
        out = []
        dt = 1.0 / SAMPLE_RATE
        fade_samples = int(FADE_MS / 1000.0 * SAMPLE_RATE)

        # Dead band — silence
        if DEADBAND_LOW <= vsi <= DEADBAND_HIGH:
            self._beep_phase = 0.0
            return [0] * n_samples

        # Compute frequency
        if vsi > 0:
            climb = min(vsi, CLIMB_MAX)
            freq = FREQ_ZERO + (climb / CLIMB_MAX) * (FREQ_MAX - FREQ_ZERO)
        else:
            sink = min(-vsi, SINK_MAX)
            freq = FREQ_ZERO - (sink / SINK_MAX) * (FREQ_ZERO - FREQ_MIN)

        # Beep timing (climb only; sink is continuous)
        if vsi > 0:
            climb = min(vsi, CLIMB_MAX)
            period_s = (PERIOD_MAX_MS + (CLIMB_MAX - climb) / CLIMB_MAX
                        * (PERIOD_MIN_MS - PERIOD_MAX_MS)) / 1000.0
            # Correct formula: faster beep at higher climb
            period_s = (PERIOD_MIN_MS + (CLIMB_MAX - climb) / CLIMB_MAX
                        * (PERIOD_MAX_MS - PERIOD_MIN_MS)) / 1000.0
            tone_on = period_s * DUTY
        else:
            period_s = 0.0  # continuous
            tone_on = 0.0

        vol = self._volume
        phase = self._phase
        beep_phase = self._beep_phase

        for i in range(n_samples):
            # Determine if tone should be on
            if vsi > 0:
                # Beeping: on during first DUTY fraction of period
                in_tone = beep_phase < tone_on
                beep_phase += dt
                if beep_phase >= period_s:
                    beep_phase -= period_s
            else:
                # Sink: continuous tone
                in_tone = True

            if in_tone:
                sample = math.sin(phase) * vol
                # Simple fade envelope at beep edges (climb only)
                if vsi > 0:
                    # Fade in at start of beep
                    beep_sample_pos = beep_phase / dt
                    if beep_sample_pos < fade_samples:
                        sample *= beep_sample_pos / fade_samples
                    # Fade out at end of tone-on
                    samples_to_off = (tone_on - beep_phase) / dt
                    if samples_to_off < fade_samples:
                        sample *= max(0.0, samples_to_off / fade_samples)
            else:
                sample = 0.0

            phase += 2.0 * math.pi * freq * dt
            if phase >= 2.0 * math.pi:
                phase -= 2.0 * math.pi

            out.append(int(sample * 32000))

        self._phase = phase
        self._beep_phase = beep_phase
        return out

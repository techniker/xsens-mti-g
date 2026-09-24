#!/usr/bin/env python3
"""PFD rolling-drum digit tests — no rendering, no display required.

The drum draws three glyphs per position and slides them; whichever glyph
sits on the baseline is the digit the pilot reads. These tests reconstruct
that reading and assert it equals the value the drum is showing, which is
the property that actually matters on an instrument.

Run: python test_pfd_widget.py
"""
import math
import sys
sys.path.insert(0, '.')

from pfd_widget import PFDWidget

failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        failures.append(name)


def drum_reads(value, num_digits):
    """The number the drum face reads.

    _draw_drum_pointer offsets the glyphs by -frac * digit_h and draws
    `digit`, `digit+1` and `digit-1`, so `digit` sits on the baseline at
    frac 0 and `digit+1` has taken its place by frac 1. Past the halfway
    point the incremented glyph is the one being read.
    """
    total = 0
    for position, (digit, frac, visible) in enumerate(
            PFDWidget._drum_digits(value, num_digits)):
        if not visible:
            continue
        shown = digit if frac < 0.5 else (digit + 1) % 10
        total += shown * 10 ** position
    return total


def expected(value, num_digits):
    """Nearest integer, halves rounding up — matching the frac >= 0.5
    handover above. Clamped to what the digits can physically show."""
    limit = 10 ** num_digits - 1
    return min(math.floor(abs(value) + 0.5), limit)


def test_reading_matches_value():
    """Swept invariant: the face must read the value, at every fraction."""
    for num_digits in (3, 5):
        bad = []
        v = 0.0
        while v < 10 ** num_digits:
            if drum_reads(v, num_digits) != expected(v, num_digits):
                bad.append((round(v, 2), drum_reads(v, num_digits),
                            expected(v, num_digits)))
            v += 0.13            # lands on many different fractions
        check(f"{num_digits}-digit drum reads its value "
              f"({len(bad)} mismatches)", not bad,
              f"first few: {bad[:4]}" if bad else "")


def test_crossing_a_power_of_ten():
    """The reported bug: just below a power of ten, the leading digit was
    suppressed while the digits below had already rolled over, so the drum
    read 00 instead of 100."""
    for value, num_digits, want in ((9.98, 3, 10),
                                    (99.98, 3, 100),
                                    (99.98, 5, 100),
                                    (999.98, 5, 1000),
                                    (9999.98, 5, 10000)):
        got = drum_reads(value, num_digits)
        check(f"{value} on a {num_digits}-digit drum reads {want}",
              got == want, f"got {got}")


def test_leading_zeros_stay_suppressed():
    """Suppression must still hide leading zeros the rest of the time."""
    for value, num_digits, want_visible in ((0.0, 3, 1), (7.0, 3, 1),
                                            (42.0, 3, 2), (137.0, 3, 3),
                                            (1234.0, 5, 4)):
        vis = sum(1 for d in PFDWidget._drum_digits(value, num_digits) if d[2])
        check(f"{value} shows {want_visible} digit(s)", vis == want_visible,
              f"got {vis}")


def test_overflow_is_clamped():
    """A value too large for the drum must peg at all nines rather than wrap
    and read a smaller number than the aircraft is actually doing."""
    for value, num_digits, want in ((1000.0, 3, 999), (5000.0, 3, 999),
                                    (123456.0, 5, 99999)):
        got = drum_reads(value, num_digits)
        check(f"{value} on a {num_digits}-digit drum pegs at {want}",
              got == want, f"got {got}")


def test_negative_uses_magnitude():
    """Altitude below sea level draws its sign separately, so the digits
    carry the magnitude."""
    for value, want in ((-250.0, 250), (-99.98, 100)):
        got = drum_reads(value, 5)
        check(f"{value} reads {want} (sign drawn separately)",
              got == want, f"got {got}")


def test_cascade_keeps_upper_digits_still():
    """Odometer feel: a higher digit must not scroll unless the digit below
    it is at 9 and rolling."""
    digits = PFDWidget._drum_digits(123.5, 3)
    check("tens/hundreds hold still at 123.5",
          digits[1][1] == 0.0 and digits[2][1] == 0.0,
          f"fracs {[round(d[1], 3) for d in digits]}")
    digits = PFDWidget._drum_digits(199.9, 3)
    check("all positions scroll during a 199->200 cascade",
          digits[0][1] > 0 and digits[1][1] > 0 and digits[2][1] > 0,
          f"fracs {[round(d[1], 3) for d in digits]}")


if __name__ == "__main__":
    test_reading_matches_value()
    test_crossing_a_power_of_ten()
    test_leading_zeros_stay_suppressed()
    test_overflow_is_clamped()
    test_negative_uses_magnitude()
    test_cascade_keeps_upper_digits_still()
    print()
    if failures:
        print("FAILED: " + ", ".join(failures))
        sys.exit(1)
    print("all PFD drum tests passed")

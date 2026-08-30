"""Fitting a delayed response to the signal that caused it.

Telemetry on a 250K link arrives a few hundred milliseconds after the command
that produced it, so comparing the two by wall-clock position mixes each phase
with the previous one. These recover the delay from the data instead.

The fit is chosen on correlation and reported as a regression slope, both
bounded quantities. Selecting on the size of the raw response instead lets a
large offset win by grouping the samples lopsidedly -- which once produced a
confident "188% of a possible" answer at the edge of the search range.
"""
from __future__ import annotations

import statistics
from bisect import bisect_right

MIN_PAIRS = 8            # fewer paired samples than this cannot support a fit
MAX_LAG = 0.6            # a real link delay is a few hundred ms
LAG_STEP = 0.02


def paired_at_lag(commands, samples, lag: float
                  ) -> tuple[list[float], list[float]]:
    """Pair each sample with the lean commanded `lag` earlier."""
    times = [stamp for stamp, _ in commands]
    commanded: list[float] = []
    measured: list[float] = []
    for stamp, value in samples:
        target = stamp - lag
        if times and times[0] <= target <= times[-1]:
            commanded.append(commands[bisect_right(times, target) - 1][1])
            measured.append(value)
    return commanded, measured


def fit_at_lag(commands, samples, lag: float) -> tuple[float, float] | None:
    """Return (correlation, gain) of measured pitch against commanded lean.

    Correlation is bounded to [-1, 1], so unlike a raw difference of means it
    cannot be inflated by an offset that happens to split the data unevenly --
    which is how maximising the response picked a nonsense lag at the edge of
    the search and reported 188% of a possible answer.

    Gain is degrees of estimate per degree commanded: about -1 when the drone
    tracks, since cflib transmits -pitch.
    """
    commanded, measured = paired_at_lag(commands, samples, lag)
    if len(commanded) < MIN_PAIRS:
        return None
    try:
        correlation = statistics.correlation(commanded, measured)
    except statistics.StatisticsError:
        return None          # one side is constant; nothing to correlate
    return correlation, statistics.linear_regression(commanded, measured).slope


def best_fit(commands, samples) -> tuple[float, float, float]:
    """Find the telemetry lag that best explains the data.

    Returns (lag, gain, correlation). Selects on correlation rather than on
    the size of the response, so a lag cannot win merely by grouping the
    samples lopsidedly.
    """
    best = (0.0, 0.0, 0.0)
    lag = 0.0
    while lag <= MAX_LAG:
        fit = fit_at_lag(commands, samples, lag)
        if fit is not None:
            correlation, gain = fit
            if abs(correlation) > abs(best[2]):
                best = (lag, gain, correlation)
        lag += LAG_STEP
    return best

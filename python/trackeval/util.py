"""Small numeric helpers shared by the analyzers."""

from __future__ import annotations

import numpy as np


def rms(x: np.ndarray, axis=None) -> np.ndarray:
    return np.sqrt(np.mean(np.square(x), axis=axis))


def smooth(x: np.ndarray, window: int = 11, order: int = 3) -> np.ndarray:
    from scipy.signal import savgol_filter

    if len(x) <= window:
        return x
    return savgol_filter(x, window, order, axis=0)


def shift_series(t: np.ndarray, y: np.ndarray, delay: float) -> np.ndarray:
    """y(t - delay) by interpolation; samples outside the record hold the end values."""
    return np.interp(t - delay, t, y)


def parabolic_peak(c: np.ndarray, k: int) -> float:
    if k <= 0 or k >= len(c) - 1:
        return float(k)
    a, b, cc = c[k - 1], c[k], c[k + 1]
    denom = a - 2 * b + cc
    return float(k) if abs(denom) < 1e-12 else float(k) + 0.5 * (a - cc) / denom


def pearson(x: np.ndarray, y: np.ndarray) -> float:
    if np.std(x) < 1e-12 or np.std(y) < 1e-12:
        return 0.0
    return float(np.corrcoef(x, y)[0, 1])


def lstsq_r2(X: np.ndarray, y: np.ndarray):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    res = y - X @ beta
    ss_tot = np.sum((y - y.mean()) ** 2)
    return beta, res, 1 - np.sum(res**2) / max(ss_tot, 1e-18)


def quat_angle_deg(q_a: np.ndarray, q_b: np.ndarray) -> np.ndarray:
    """Rotation angle between two quaternion series (N,4) x y z w, in degrees."""
    dots = np.abs(np.sum(q_a * q_b, axis=1))
    return np.degrees(2 * np.arccos(np.clip(dots, -1.0, 1.0)))


def wrap_pi(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi

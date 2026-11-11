"""Truth-model evaluation helpers shared by the four studies."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc

SIG_V = 0.015      # exit speed shot-to-shot sigma (relative); 1% and 2% are also run
SIG_TH = 0.5       # launch angle sigma (deg)


class Truth:
    def __init__(self, name):
        self.name = name
        self.F = sc.load_field(name)
        self.S = sc.score_indicator(self.F)
        self._blur = {}

    def P_field(self, sig_v=SIG_V, sig_th=SIG_TH):
        key = (sig_v, sig_th)
        if key not in self._blur:
            suffix = "" if sc.DESCENT_MIN_DEG <= 0 else f"_desc{sc.DESCENT_MIN_DEG:g}"
            path = os.path.join(sc.CACHE, f"blur_{self.name}_{sig_v:.3f}_{sig_th:.2f}{suffix}.npy")
            if os.path.exists(path):
                self._blur[key] = np.load(path)
            else:
                self._blur[key] = sc.blur_prob(self.S, sig_v, sig_th)
                np.save(path, self._blur[key])
        return self._blur[key]

    def vt(self, rpm, hood, gain=sc.K_GAIN):
        hood = np.asarray(hood, float)
        return sc.rpm_to_v(rpm, hood, gain), sc.hood_to_theta(hood)

    def at(self, D, rpm, hood, gain=sc.K_GAIN, keys=("w", "vz", "tof")):
        D, rpm, hood = np.broadcast_arrays(np.asarray(D, float), np.asarray(rpm, float), np.asarray(hood, float))
        v, th = self.vt(rpm, hood, gain)
        oob = (hood < 0) | (hood > 1)
        out = {}
        for k in keys:
            out[k] = np.where(oob, np.nan, sc.trilinear(self.F[k], D, v, th))
        return out

    def scored(self, D, rpm, hood, gain=sc.K_GAIN, margin=sc.MARGIN):
        r = self.at(D, rpm, hood, gain, keys=("w", "vz", "vx"))
        lo, hi = sc.w_band(margin)
        with np.errstate(invalid="ignore"):
            return sc.descending(r["vz"], r["vx"]) & (r["w"] >= lo) & (r["w"] <= hi)

    def P(self, D, rpm, hood, gain=sc.K_GAIN, sig_v=SIG_V, sig_th=SIG_TH):
        D, rpm, hood = np.broadcast_arrays(np.asarray(D, float), np.asarray(rpm, float), np.asarray(hood, float))
        v, th = self.vt(rpm, hood, gain)
        p = sc.trilinear(self.P_field(sig_v, sig_th), D, v, th)
        p = np.where((hood < 0) | (hood > 1), 0.0, p)
        return np.nan_to_num(p, nan=0.0)

    def miss(self, D, rpm, hood, gain=sc.K_GAIN):
        """Signed miss along the opening face from the centre of the allowed band (in); +inf-ish if no crossing.
        Rising crossings are reported with their w but flagged separately."""
        r = self.at(D, rpm, hood, gain, keys=("w", "vz"))
        return r["w"] - sc.W_MID, r["vz"]


def run_extent(S, axis):
    """For a boolean array, cells from each True cell to the last True cell of its run, below and above along axis."""
    S = np.moveaxis(S, axis, 0)
    n = S.shape[0]
    idx = np.arange(n).reshape((n,) + (1,) * (S.ndim - 1))
    idx = np.broadcast_to(idx, S.shape)
    last_false = np.maximum.accumulate(np.where(~S, idx, -1), axis=0)
    down = idx - last_false - 1                       # true cells strictly below in the run
    rev = S[::-1]
    idx_r = idx[::-1]
    last_false_r = np.minimum.accumulate(np.where(~rev, idx_r, n), axis=0)[::-1]
    up = last_false_r - idx - 1
    down = np.where(S, down, -1)
    up = np.where(S, up, -1)
    return np.moveaxis(down, 0, axis), np.moveaxis(up, 0, axis)


def fmt_table(rows, cols, floatfmt="{:.3g}"):
    w = {c: max(len(c), *(len(_f(r.get(c), floatfmt)) for r in rows)) for c in cols}
    line = " | ".join(c.ljust(w[c]) for c in cols)
    sep = "-|-".join("-" * w[c] for c in cols)
    body = "\n".join(" | ".join(_f(r.get(c), floatfmt).ljust(w[c]) for c in cols) for r in rows)
    return line + "\n" + sep + "\n" + body


def _f(x, ff):
    if x is None:
        return "-"
    if isinstance(x, (float, np.floating)):
        if math.isnan(x):
            return "nan"
        return ff.format(x)
    return str(x)

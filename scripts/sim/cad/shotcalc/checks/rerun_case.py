"""Reruns two study cases into rerun_out/ and compares them with ../out bit for bit (run ../study.py for both first)."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import study
ORIG = study.OUT
study.OUT = os.path.join(HERE, "rerun_out")
for case in (("pollen", "geo", "nom"), ("nectar", "cal", "high")):
    p = study.run(*case)
    a, b = np.load(p), np.load(os.path.join(ORIG, os.path.basename(p)))
    worst = {}
    for kname in a.files:
        x, y = np.asarray(a[kname], float), np.asarray(b[kname], float)
        both = np.isfinite(x) & np.isfinite(y)
        worst[kname] = (float(np.max(np.abs(x[both] - y[both]))) if both.any() else 0.0, int(np.sum(np.isfinite(x) != np.isfinite(y))))
    print(case, "max |diff| and NaN-pattern mismatches:", {k: v for k, v in worst.items() if v != (0.0, 0)} or "identical")

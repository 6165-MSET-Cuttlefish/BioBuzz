"""Probe the Cl +0.15 'best speed' picks: steep backspin shots. Scan z at x = D against v, and the plane crossing."""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import indep as I
dia, mass = I.BALL['pollen']; k = I.kq(dia, mass, 0.5); kl = k * 0.15 / 0.5
for m, D in ((26, 48), (21.5, 132)):
    th = 57.7 + m; x0, z0 = I.exitp('pollen', m)
    print(f"m {m} D {D} th {th:.1f}")
    for v in np.arange(300, 700, 20):
        o = I.fly(v, th, x0, z0, D, k, kl)
        print(f"  v {v:5.0f}: z@x=D {o.get('za', float('nan')):7.1f} t {o.get('ta', float('nan')):.2f} vz@D {o.get('vza', float('nan')):7.1f} | plane w {o.get('w', float('nan')):6.2f} vz {o.get('vz', float('nan')):7.1f} margin {I.margin_fn(o, dia):+.2f}")

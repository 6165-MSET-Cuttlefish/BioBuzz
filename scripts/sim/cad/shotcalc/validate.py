"""Reproduces the drag-free min-speed line and its speed window from ../../ballistics/nodrag.py and windows_nodrag.py
for a 14 in exit at the turret axis, and the 40 g, Cd 0.45 case of ../../ballistics/run_drag.py, before switching
to the CAD geometry."""
import json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cadcore import vmin_search, kq_per_in, v_aim, Z_AIM, AIM_DX
from windows import analyze

EXIT14 = lambda th: (np.zeros_like(np.asarray(th, float)), np.full_like(np.asarray(th, float), 14.0))
DS = [36, 60, 84, 108, 132]
REF = dict(v=[195.26, 212.11, 230.07, 247.76, 264.78], th=[69.97, 62.76, 58.50, 55.81, 53.99],
           desc=[20.03, 27.24, 31.50, 34.19, 36.01], tof=[538, 618, 699, 776, 848],
           s_lo=[-2.10, -2.29, -2.03, -1.77, -1.55], s_hi=[6.00, 4.62, 3.59, 2.90, 2.41],
           v40=[198.9, 217.1, 236.9, 256.7, 276.0])
print(f"aim z {Z_AIM:.4f} (ref 56.8151), aim behind opening bottom edge {AIM_DX:.3f} in (ref 5.378)")
rows = []
worst = {}
for i, D in enumerate(DS):
    v, th = vmin_search(EXIT14, D)
    a = analyze(2.8, EXIT14, np.array([th]), D, v=np.array([v]))
    v40, th40 = vmin_search(EXIT14, D, kq_per_in(40, 0.45, 2.8))
    row = dict(D=D, v=v, th=th, desc=float(a["descent_aim"][0]), tof=float(a["ta"][0] * 1000),
               s_lo=float(a["s_lo"][0] * 100), s_hi=float(a["s_hi"][0] * 100), h_lo=float(a["h_lo"][0]),
               h_hi=float(a["h_hi"][0]), v40=v40)
    rows.append(row)
    for k in ("v", "th", "desc", "tof", "s_lo", "s_hi", "v40"):
        worst[k] = max(worst.get(k, 0), abs(row[k] - REF[k][i]))
    print(f"D={D:3d}: v {v:7.2f} (ref {REF['v'][i]})  th {th:6.2f} ({REF['th'][i]})  desc {row['desc']:6.2f} ({REF['desc'][i]})  "
          f"tof {row['tof']:6.1f} ({REF['tof'][i]})  speed win {row['s_lo']:+.2f}/{row['s_hi']:+.2f} "
          f"({REF['s_lo'][i]:+.2f}/{REF['s_hi'][i]:+.2f})  hood {row['h_lo']:+.2f}/{row['h_hi']:+.2f}  v40g {v40:6.1f} ({REF['v40'][i]})")
print("max abs deviation:", {k: round(v, 3) for k, v in worst.items()})
print("ref hood window at 36 in on the min-speed line: -0.74/+5.31 deg (windows_nodrag.py, vmin family, phi 0)")
os.makedirs(os.path.join(os.path.dirname(os.path.abspath(__file__)), "out"), exist_ok=True)
json.dump(dict(rows=rows, max_dev=worst), open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "validate.json"), "w"), indent=1)

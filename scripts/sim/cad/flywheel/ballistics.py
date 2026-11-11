"""Drag-free shot geometry for the CAD exit points and hood ranges.

Opening, aim point and entry rule re-implemented from BioBuzzField.java / ShotSolver.checkEntry / ShotTuning
and from ../../ballistics/geom.py and cross.py, parametrised by exit height, exit offset and ball diameter.
validate() first reproduces the 14 in exit numbers of ../../ballistics/nodrag.py and windows_nodrag.py.
"""
import math
import numpy as np

G = 386.09
W_OPEN, SHOULDER, H_OPEN, BOTTOM_Z = 20.0, 7.61, 14.0, 53.5
TILT = math.radians(30)
AIM_DEPTH, RIM = 3.0, 1.0
U = np.array([1.0, 0.0, 0.0])
WV = np.array([0.0, math.sin(TILT), math.cos(TILT)])
NV = np.array([0.0, -math.cos(TILT), math.sin(TILT)])
O = np.array([0.0, 0.0, BOTTOM_Z])


def centroid_w():
    rect = W_OPEN * SHOULDER
    tri = W_OPEN * (H_OPEN - SHOULDER) / 2
    return (rect * SHOULDER / 2 + tri * (SHOULDER + (H_OPEN - SHOULDER) / 3)) / (rect + tri)


AIM = O + centroid_w() * WV - AIM_DEPTH * NV
DS = [36, 60, 84, 108, 132]

SHOOTERS = {
    # launch angle = a0 + m (deg); m in [0, mmax]; exit point linear in m between the two CAD rows [inf]
    'pollen': dict(a0=57.7, a0_cal=55.9, mmax=30.0, x0=1.56, z0=12.46, x1=1.99, z1=13.83, dia=2.8),
    'nectar': dict(a0=60.8, a0_cal=57.4, mmax=20.0, x0=1.23, z0=12.53, x1=1.60, z1=13.57, dia=3.6),
}


def edge_slack(u, w, margin):
    half = W_OPEN / 2
    rise = H_OPEN - SHOULDER
    slant = math.hypot(half, rise)
    below = ((H_OPEN - w) * half - abs(u) * rise) / slant
    return min(w - margin, half - abs(u) - margin, below - margin)


def crossing(p0, v0):
    a = -0.5 * G * NV[2]; b = NV @ v0; c = NV @ (p0 - O)
    disc = b * b - 4 * a * c
    if c <= 0:
        return None, 'BEHIND'
    if disc < 0:
        return None, 'NO_ENTRY'
    for t in sorted([(-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)]):
        if t > 0 and b + 2 * a * t < 0:
            p = p0 + v0 * t + np.array([0, 0, -0.5 * G * t * t])
            v = v0 + np.array([0, 0, -G * t])
            return (t, p, v), 'OK'
    return None, 'NO_ENTRY'


def entry_ok(dexit, zexit, v, th, dia, rim=RIM):
    """Ball launched from horizontal distance dexit (to the aim point) at height zexit, azimuth 0."""
    p0 = np.array([AIM[0], AIM[1] - dexit, zexit])
    v0 = np.array([0.0, v * math.cos(th), v * math.sin(th)])
    res, st = crossing(p0, v0)
    if res is None:
        return False, st
    t, p, vv = res
    dp = p - O
    u, w = dp @ U, dp @ WV
    inc = -(vv @ NV) / np.linalg.norm(vv)
    margin = dia / 2 / max(inc, 0.33) + rim
    if vv[2] >= 0:
        return False, 'RISING'
    return edge_slack(u, w, margin) >= 0, 'OK'


def v_for_angle(dexit, h, th):
    den = 2 * math.cos(th) ** 2 * (dexit * math.tan(th) - h)
    return math.sqrt(G * dexit * dexit / den) if den > 0 else float('nan')


def vmin_angle(dexit, h):
    r = math.hypot(dexit, h)
    return math.sqrt(G * (h + r)), math.atan((h + r) / dexit)


def speed_window(dexit, zexit, v, th, dia, rim=RIM):
    def ok(x):
        return entry_ok(dexit, zexit, v * (1 + x), th, dia, rim)[0]
    if not ok(0.0):
        return float('nan'), float('nan')
    out = []
    for sgn in (-1, 1):
        lo, hi = 0.0, 0.25
        xs = np.linspace(0, 0.25, 501)
        for x in xs[1:]:
            if not ok(sgn * x):
                hi = x
                break
            lo = x
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if ok(sgn * mid):
                lo = mid
            else:
                hi = mid
        out.append(sgn * lo)
    return out[0], out[1]


def exit_point(sh, m):
    s = SHOOTERS[sh]
    a = m / s['mmax']
    return s['x0'] + a * (s['x1'] - s['x0']), s['z0'] + a * (s['z1'] - s['z0'])


def shot(sh, d, calibrated=False):
    """Nominal shot: min-speed angle if the hood reaches it, else the hood's lowest angle. Fixed point on m."""
    s = SHOOTERS[sh]
    a0 = s['a0_cal'] if calibrated else s['a0']
    m = 0.0
    for _ in range(60):
        x, z = exit_point(sh, m)
        dexit, h = d - x, AIM[2] - z
        vm, thm = vmin_angle(dexit, h)
        mnew = min(max(math.degrees(thm) - a0, 0.0), s['mmax'])
        if abs(mnew - m) < 1e-9:
            break
        m = mnew
    x, z = exit_point(sh, m)
    dexit, h = d - x, AIM[2] - z
    vm, thm = vmin_angle(dexit, h)
    th = math.radians(a0 + m)
    pinned = m <= 0.0 and math.degrees(thm) < a0
    v = vm if not pinned else v_for_angle(dexit, h, th)
    lo, hi = speed_window(dexit, z, v, th, s['dia'])
    tof = dexit / (v * math.cos(th))
    vz = v * math.sin(th) - G * tof
    return dict(shooter=sh, d=d, m=m, launch_deg=a0 + m, vmin_angle_deg=math.degrees(thm), pinned=pinned,
                exit_x=x, exit_z=z, dexit=dexit, rise=h, v_in_s=v, vmin_in_s=vm, extra_vs_vmin_pct=100 * (v / vm - 1),
                tof_s=tof, descent_deg=math.degrees(math.atan2(-vz, v * math.cos(th))),
                win_lo_pct=100 * lo, win_hi_pct=100 * hi)


REF_WIN = {36: (-2.10, 6.00), 60: (-2.29, 4.62), 84: (-2.03, 3.59), 108: (-1.77, 2.90), 132: (-1.55, 2.41)}
REF_V = {36: 195.26, 60: 212.11, 84: 230.07, 108: 247.76, 132: 264.78}


def validate():
    rows = []
    for d in DS:
        h = AIM[2] - 14.0
        v, th = vmin_angle(d, h)
        lo, hi = speed_window(d, 14.0, v, th, 2.8)
        rows.append(dict(d=d, v=v, th=math.degrees(th), lo=100 * lo, hi=100 * hi,
                         ref_v=REF_V[d], ref_lo=REF_WIN[d][0], ref_hi=REF_WIN[d][1]))
        assert abs(v - REF_V[d]) < 0.01, (d, v)
        assert abs(100 * lo - REF_WIN[d][0]) < 0.011 and abs(100 * hi - REF_WIN[d][1]) < 0.011, (d, lo, hi)
    return dict(aim_z=float(AIM[2]), rows=rows)


if __name__ == '__main__':
    import json
    v = validate()
    print(f"aim z {v['aim_z']:.4f} in (ref 56.8151)")
    for r in v['rows']:
        print(f" d={r['d']}: vmin {r['v']:.2f} (ref {r['ref_v']}) th {r['th']:.2f}  window {r['lo']:+.2f}/{r['hi']:+.2f} "
              f"(ref {r['ref_lo']:+.2f}/{r['ref_hi']:+.2f})")
    for sh in SHOOTERS:
        for cal in (False, True):
            for d in DS:
                s = shot(sh, d, cal)
                print(f"{sh:6s} cal={int(cal)} d={d:3d} m={s['m']:5.2f} launch={s['launch_deg']:5.2f} vminang={s['vmin_angle_deg']:5.2f} "
                      f"pinned={int(s['pinned'])} z={s['exit_z']:.2f} v={s['v_in_s']:6.2f} (+{s['extra_vs_vmin_pct']:.2f}% vs vmin) "
                      f"tof={1000*s['tof_s']:.0f}ms descent={s['descent_deg']:.1f} win {s['win_lo_pct']:+.2f}/{s['win_hi_pct']:+.2f}")

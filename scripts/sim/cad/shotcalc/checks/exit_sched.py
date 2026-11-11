"""Drag-free speed change from the CAD exit vs (0, 14 in) on schedule (i) shots, and the solver-style entry check
(exit at the axis, 14 in) applied to the CAD rising limit at 36 in."""
import math
G, ZAIM = 386.09, 56.81508
def v_nd(th, x0, z0, D):
    t = math.radians(th); d = D - x0; h = ZAIM - z0
    return math.sqrt(G * d * d / (2 * math.cos(t) ** 2 * (d * math.tan(t) - h)))
for sh, base, mmax, ex, sched in (("pollen", 57.7, 30, ((1.56, 12.46), (1.99, 13.83)), {36: 15, 48: 8, 60: 2, 84: 0, 132: 0}),
                                 ("nectar", 60.8, 20, ((1.23, 12.53), (1.60, 13.57)), {36: 11.5, 48: 4.5, 60: 0, 84: 0, 132: 0})):
    cells = []
    for D, m in sched.items():
        f = m / mmax
        x0 = ex[0][0] + f * (ex[1][0] - ex[0][0]); z0 = ex[0][1] + f * (ex[1][1] - ex[0][1])
        cells.append(f"{D}:{100 * (v_nd(base + m, x0, z0, D) / v_nd(base + m, 0, 14, D) - 1):+.2f}%")
    print(sh, "schedule (i), same launch angle:", " ".join(cells))

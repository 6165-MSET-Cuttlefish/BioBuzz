"""Order-of-magnitude flywheel sag per ball and sustained ball rate [inferred, not measured].

Model: the ball leaves at v = eta * wheel surface speed, rolling on the stationary hood (spin v / r_ball).
Impulse balance on the ball (wheel side J1, hood side J2): J1 + J2 = m v, (J1 - J2) r_b = k m r_b v
-> J1 = (1 + k)/2 m v.  The flywheel loses J1 * r_w of angular momentum per ball (feed-roller and side-guide
friction ignored), so the fractional sag is dw/w = (1 + k)/2 * m * eta * r_w^2 / I, independent of rpm.
Recovery: net motor torque tau_s (1 - w / w_free) at 12 V; per-ball recovery time = dL / tau; sustained rate = tau / dL.
"""
import json, math, os

HERE = os.path.dirname(os.path.abspath(__file__))
os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
R_W = 0.036                     # m, 72 mm wheel [CAD], unloaded
TAU_S = 0.1442                  # N m, 1.47 kg cm at 12 V: goBILDA 5203 1:1 page
                                # (https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/)
I_CAD = 0.78e-4                 # kg m^2, estimate from the CAD model as of 2026-11-10, excluding rotor and feed roller [inf]
BALLS = {"pollen": (24.9, 23.5, 27.2), "nectar": (41.3, 39.0, 45.0)}
K_SHELL, K_SOLID = 2 / 3, 2 / 5

rows = []
for name, (m_nom, m_lo, m_hi) in BALLS.items():
    for eta in (0.35, 0.45, 0.50):
        for k, kname in ((K_SOLID, "solid"), (K_SHELL, "shell")):
            c = (1 + k) / 2 * m_nom / 1000 * eta * R_W ** 2          # dL / w  (kg m^2)
            for I in (I_CAD, 1.0e-4, 1.5e-4, 3.0e-4):
                rows.append(dict(ball=name, eta=eta, spin=kname, I=I, sag_pct=100 * c / I))
out = {"sag": rows, "rate": []}
for name, (m_nom, _, _) in BALLS.items():
    for eta in (0.35, 0.45, 0.50):
        c = (1 + K_SHELL) / 2 * m_nom / 1000 * eta * R_W ** 2
        for free in (5400, 5800):
            for f in (0.6, 0.7, 0.8, 0.9):
                w = f * free * 2 * math.pi / 60
                tau = TAU_S * (1 - f)
                dL = c * w
                out["rate"].append(dict(ball=name, eta=eta, free=free, frac=f, rate_per_s=tau / dL,
                                        recover_ms=1000 * dL / tau))
json.dump(out, open(os.path.join(HERE, "out", "sag_energy.json"), "w"), indent=1)
print("per-ball sag %, shell spin (solid in brackets), by I (kg m^2):")
for name in BALLS:
    for eta in (0.35, 0.45, 0.50):
        sh = [r for r in rows if r["ball"] == name and r["eta"] == eta and r["spin"] == "shell"]
        so = [r for r in rows if r["ball"] == name and r["eta"] == eta and r["spin"] == "solid"]
        print(f" {name:6s} eta {eta:.2f}: " + "  ".join(f"I={a['I']:.2e}: {a['sag_pct']:5.1f} ({b['sag_pct']:4.1f})" for a, b in zip(sh, so)))
print("sustained balls/s and per-ball recovery (ms) at 12 V, shell spin, free 5400 [5800]:")
for name in BALLS:
    for eta in (0.35, 0.45, 0.50):
        a = [r for r in out["rate"] if r["ball"] == name and r["eta"] == eta and r["free"] == 5400]
        b = [r for r in out["rate"] if r["ball"] == name and r["eta"] == eta and r["free"] == 5800]
        print(f" {name:6s} eta {eta:.2f}: " + "  ".join(f"{x['frac']:.0%} free: {x['rate_per_s']:4.1f}/s {x['recover_ms']:4.0f} ms [{y['rate_per_s']:4.1f}]" for x, y in zip(a, b)))

# ---- burst at the stated rapid-fire rate: per-ball drop c/I, full-power recovery between balls (ideal motor, 12 V)
RATE = 10.9                      # balls/s, transfer-only estimate [inf]
print(f"burst at {RATE} balls/s, eta .45, shell spin, free 5400: ball n speed below ball 1 (%), n = 2..5, and the 5th ball's per-ball recovery shortfall")
out["burst"] = []
for name, (m_nom, _, _) in BALLS.items():
    c = (1 + K_SHELL) / 2 * m_nom / 1000 * 0.45 * R_W ** 2
    for f in (0.6, 0.7, 0.8):
        wf = 5400 * 2 * math.pi / 60
        w0 = f * wf
        line = []
        for I in (I_CAD, 1.5e-4, 3.0e-4, 6.0e-4):
            w = w0
            speeds = []
            for n in range(5):
                speeds.append(w)
                w *= 1 - c / I
                tau_c = I * wf / TAU_S
                w = wf - (wf - w) * math.exp(-(1 / RATE) / tau_c)
                w = min(w, w0)
            rel = [100 * (1 - x / speeds[0]) for x in speeds[1:]]
            out["burst"].append(dict(ball=name, frac=f, I=I, drop_pct=rel))
            line.append(f"I={I:.2e}: " + "/".join(f"{x:4.1f}" for x in rel))
        print(f" {name:6s} {f:.0%} free: " + "  ".join(line))
json.dump(out, open(os.path.join(HERE, "out", "sag_energy.json"), "w"), indent=1)

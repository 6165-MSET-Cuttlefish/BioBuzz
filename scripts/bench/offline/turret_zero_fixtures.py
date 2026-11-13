"""Synthetic Bench: Turret Zero results, built from a model of one Through Bore, for the offline checks and dry run."""
import math

CPT = 8192


def wrap(deg):
    return (deg + 180) % 360 - 180


def summary(p50, count, p95=None):
    return {"count": count, "mean": p50, "min": p50 - 0.5, "p50": p50, "p90": p50 + 0.4,
            "p95": p50 + 0.8 if p95 is None else p95, "p99": p50 + 1.2, "max": p50 + 2.0, "largest": [p50 + 2.0]}


def info():
    return {"phase": "x", "chipId": 0x51, "firmware": "3.1.0", "uptimeMs": 12345, "bankAsFound": "ALL_QUADRATURE",
            "pulseWidthParams": [0, 65535], "tracksWrapAsFound": False}


def bank(first_ms=2.0):
    return {"set": "BANK1_QUADRATURE_BANK2_PULSE_WIDTH", "readBack": "BANK1_QUADRATURE_BANK2_PULSE_WIDTH",
            "firstReads": [[0.4, 0, 1], [first_ms, 512, 1]], "firstPlausibleMs": first_ms}


def run(result, crashed=False, error="", pid=(100, 100)):
    return {"crashed": crashed, "error": error, "timedOut": False, "appRestarted": pid[0] != pid[1],
            "pidBefore": pid[0], "pidAfter": pid[1], "result": result}


def track(rests=None, rows=None, **extra):
    t = {"rests": rests or [], "rows": rows or [], "hubEvents": [], "fakeReads": 0, "crcFailures": 0}
    t.update(extra)
    return {"info": info(), "bank": bank(), "track": t, "restore": {"bank": "ALL_QUADRATURE"}}


def to_mark(key, start, turns, end_off=0, per_turn=CPT):
    """Rests of turns to a mark and back: start, one on the way, the far mark, the start mark again."""
    far = start + round(per_turn * turns)
    return [{key: start}, {key: start + per_turn // 3}, {key: far}, {key: (start + far) // 2}, {key: start + end_off}]


def span_rows(turns=3.0, sign=1, per_turn=CPT, n=6000, lo=1.0, hi=1024.0, positions=1024, zero_deg=40.0,
              start_count=1234):
    turn_us = (hi - lo) * positions / (positions - 1)
    rows = []
    for i in range(n + 1):
        t = turns * i / n
        ang = (zero_deg + 360 * t) % 360
        pulse = math.floor(lo + ang / 360 * turn_us) if positions == 1024 else round(lo + ang / 360 * turn_us)
        rows.append([i * 3.5, start_count + round(sign * per_turn * t), pulse, 1, 0])
    return rows


def agreement_rests(sign=1, zero_deg=211.0, zero_count=4000, steps=13, step_turns=0.19, noise=0.2, bad_at=None,
                    bad_deg=0.0, hub_sign=None, still=False):
    """Rests as the OpMode writes them: errors from the hub count with hub_sign, the shaft turning with sign."""
    hub_sign = sign if hub_sign is None else hub_sign
    rests = []
    for k in range(steps):
        count = zero_count + (0 if still else round(k * step_turns * CPT))
        true = (zero_deg + sign * (count - zero_count) * 360 / CPT) % 360
        a = (true + (noise if k % 2 else -noise) + (bad_deg if k == bad_at else 0)) % 360
        counted = zero_deg + hub_sign * (count - zero_count) * 360 / CPT
        flipped = zero_deg - hub_sign * (count - zero_count) * 360 / CPT
        rests.append({"count": count, "absoluteDeg": a, "errorDeg": wrap(a - counted),
                      "errorFlippedSignDeg": wrap(a - flipped), "error180Deg": wrap(a - counted - 180)})
    return rests


def init_result(absolute, count, offset, reads=1, p95=6.0, crc=0, spread=0.35, valid=True):
    init = {"count": count, "pulseUs": 1 + round(absolute / 360 * 1024), "tries": 1, "valid": valid,
            "absoluteDeg": absolute, "offsetDeg": offset, "readMs": summary(4.5, reads, p95), "crcFailures": crc,
            "spreadDeg": spread, "countAfter": count}
    return {"info": info(), "bank": bank(), "init": init, "restore": {"bank": "ALL_QUADRATURE"}}


def good(brownout="no lab supply", v2=False):
    zero_offset, rest_count = 77.0, 4000
    runs = {"quad": run(track(to_mark("median", 1000, 5))),
            "split_plugged": run(track(to_mark("count", 500, 10))),
            "split_unplugged": run(track(to_mark("count", 600, 10, end_off=3))),
            "span": run(track(rows=span_rows()))}
    absolute = (zero_offset + rest_count * 360 / CPT) % 360
    for i in range(1, 11):
        runs[f"init_{i:02d}"] = run(init_result(absolute, rest_count, zero_offset + (0.1 if i % 2 else -0.1),
                                                reads=50 if i == 1 else 1))
    runs["init_moved"] = run(init_result((absolute + 90) % 360, rest_count + 2048, zero_offset + 0.2))
    runs["agreement"] = run(track(agreement_rests(), zero={"valid": True, "count": 4000, "absoluteDeg": 211.0}))
    runs["sign"] = run({"info": info(), "bank": bank(), "sign": {
        "orientation": "CCW", "FORWARD": {"raw": 2048, "getCurrentPosition": -2048, "bulk": 2048, "bulkFake": False},
        "REVERSE": {"raw": 2048, "getCurrentPosition": 2048, "bulk": 2048, "bulkFake": False}}})
    runs["reset"] = run({"info": info(), "bank": bank(), "reset": {
        "countAtInit": 3000, "afterRunWithoutEncoder": [3000, 3000], "afterRunUsingEncoder": [3000, 3001],
        "afterBackToRunWithoutEncoder": [3000, 3000], "absolute": {"absoluteDeg": 100.0, "valid": True}}})
    runs["reset_after"] = run({"info": info(), "bank": bank(), "resetAfter": {
        "countAtInit": 3000, "absoluteBefore": {"absoluteDeg": 100.0, "valid": True}, "afterStopAndReset": [0, 0],
        "afterRunWithoutEncoder": [0, 0], "absoluteAfter": {"absoluteDeg": 100.2, "valid": True}}})
    tz = {"answers": {"cable1": "ok", "split": "OK", "supplyV": "3.29", "highV": "3.27 V",
                      "labSupply": "yes" if brownout == "ran" else "no"},
          "runs": runs, "v2": "asked" if v2 else "not asked for", "brownout": brownout}
    if brownout == "ran":
        runs["brownout_before"] = run(init_result(100.0, 2048, 10.0))
        runs["brownout"] = run(track(firstCount=2048, lastCount=2048))
        runs["brownout_after"] = run(init_result(100.3, 2048, 10.3))
        tz["brownoutRunner"] = {"warnings": [], "before": ["", ""]}
    if v2:
        tz["answers"].update(v2SupplyV="3.29", v2HighV="3.28", v2sHighV="3.27")
        runs["v2_a"] = run(track(to_mark("count", 100, 1), rows=span_rows(turns=1.2, lo=3.884, hi=998.06,
                                                                           positions=4096)))
        still = [[i * 3.5, 0, 4 + round(i / 2000 * 990), 1, 0] for i in range(2001)]
        runs["v2_s"] = run(track(rows=still))
    return tz

"""Bench: Rangefinders: the two Brushland laser rangefinders on a still card, swept by a fast target to find their
analog update rate, a 20 ms control on sensor A (written over I2C, then its as-found settings written back), and cable
flexes and swaps that name a sensor or a cable behind a swinging reading (scripts/bench/README.md)."""
import json
from xml.etree import ElementTree

POLL = "Bench: Analog Poll"
READ = "Bench: Rangefinder Read"
CONFIGURE = "Configure Laser Rangefinder"
ANALOG_SOURCE = "laser_rangefinder"
CONFIG_ANALOG = "bench_rangefinders"
CONFIG_I2C = "bench_rf_i2c"
I2C_LINE = '<RevColorSensorV3 name="Laser" bus="3" port="0" />'
A, B = "analog0", "analog1"
PORT = {A: 1, B: 3}
STILL_S = 60.0
FLEX_S = 20.0
SWAP_STILL_S = 30.0
RATE_S = 30.0
READ_WAIT_S = 120.0
WARNING_WAIT_S = 10.0
CONFIGURE_HOLD_S = 3.0
LOW_MV = 50
STEP_MV = 200
SPREAD_MV = 50
SD_FACTOR = 3
GATE_MS = 10.0
MAX_GAP_MS = 60.0
MIN_STEPS = 200
MIN_FAST = 0.8
BUDGET_MS = 10
CONTROL_MS = 20
DWELL_RANGE = {BUDGET_MS: (9.0, 12.0), CONTROL_MS: (18.0, 22.0)}
CARD_MM = 150
DISTANCE_MODES = {1: "SHORT", 2: "MEDIUM", 3: "LONG"}
ANALOG_PIN = 1
MIN_ROI = 4
# LaserRangefinder.setTiming refuses a budget outside 5-1000 ms and a nonzero period under the budget + 3 ms.
BUDGET_LIMITS_MS = (5, 1000)
MIN_PERIOD_MARGIN_MS = 3
# LaserRangefinder.setROI refuses a corner outside 0-15.
ROI_MAX = 15
READ_FIELDS = ("pin0", "pin1", "distanceModeRaw", "timing", "roi", "opticalCenter")
ARRANGEMENTS = (
    ("base", "each sensor on its own cable, A in Control Hub analog port 1, B in port 3",
     {A: ("sensor A", "cable A"), B: ("sensor B", "cable B")}),
    ("sensors swapped", "sensor A on cable B, sensor B on cable A, the hub ends unchanged",
     {A: ("sensor B", "cable A"), B: ("sensor A", "cable B")}),
    ("cables swapped", "each sensor on its own cable, cable A in analog port 3, cable B in port 1",
     {A: ("sensor B", "cable B"), B: ("sensor A", "cable A")}),
)
LEFT_WRITTEN = ("!! Rangefinder A may still have the 20 ms control written to it. Put it back with Configure Laser "
                "Rangefinder on bench_rf_i2c: raw/rangefinder_asfound.json has its settings from before the bench")

STILL_MEANS = (f"on a still card the reading spread {SPREAD_MV} mV or more (p1 to p99) or fell under {LOW_MV} mV: the "
               "sensor, its cable or the hub port swings, which is what the swaps try to pin down")
RATE_MEANS = ("the analog output steps less often than every measurement at the stored budget, so a 10 ms poll reads "
              "repeated values; or not enough motion (see the detail), which is no verdict: swing the target faster")
CONTROL_MEANS = ("setup: at a 20 ms budget the bench didn't measure a 20 ms dwell (or the target moved too slowly), so "
                 "it can't tell a 10 ms sensor from a 20 ms one; its 10 ms results mean nothing")
READBACK_MEANS = ("rangefinder A doesn't read back the settings it had before the bench: it is left on other settings "
                  "(raw/rangefinder_asfound.json has the old ones; write them with Configure Laser Rangefinder)")
SWAPS_MEANS = ("the swaps didn't name one part: either no reading fell under 50 mV or stepped over 200 mV in any "
               "arrangement (the swing wasn't reproduced), or the fault didn't follow one sensor, cable or port")


def analog_lines():
    root = ElementTree.parse(rb.REPO_ROOT / rb.SHIPPED_CONFIG_DIR / f"{ANALOG_SOURCE}.xml").getroot()
    lines = [f'<AnalogInput name="{e.get("name")}" port="{e.get("port")}" />' for e in root.iter("AnalogInput")]
    names = {e.get("name"): int(e.get("port")) for e in root.iter("AnalogInput")}
    if names != PORT:
        raise rb.BenchError(f"{ANALOG_SOURCE}.xml names its rangefinders {names}, not {PORT}: update the bench")
    return lines


def ask(r, text):
    try:
        r.ask(f">>> {text}")
    except EOFError:
        raise rb.BenchError(f"the terminal closed before you confirmed: {text}") from None


def do(r, text):
    ask(r, f"{text} Then press Enter.")


def ready(r, text):
    ask(r, f"Press Enter when you are ready to {text}. Start when the runner says GO.")


def go(r, text):
    return lambda: r.log(f">>> GO: {text}")


def put_back(record, key, text=None):
    """What the person has to put back if the bench stops here; summary.md lists it."""
    if text is None:
        record["putBack"].pop(key, None)
    else:
        record["putBack"][key] = text


def strip(stats):
    return {k: v for k, v in (stats or {}).items() if k not in ("tMs", "mv")}


def steady(r, label, seconds, during=None):
    o = r.run_opmode(POLL, f"rangefinders_{label.replace(' ', '_')}",
                     {"mode": "steady", "label": label.replace(" ", "_"), "seconds": seconds, "lowMv": LOW_MV,
                      "stepMv": STEP_MV, "sensorA": A, "sensorB": B},
                     timeout=seconds + 60, during=during, result_files=[f"analog_poll_{label.replace(' ', '_')}"])
    if o.get("crashed"):
        raise rb.BenchError(f"{POLL} crashed in its {label} phase: {o.get('errorMessage')!r}")
    res = r.pull(f"analog_poll_{label.replace(' ', '_')}")
    phase = (res.get("phases") or {}).get(f"steady {label.replace(' ', '_')}") or {}
    return {ch: strip(phase.get(ch)) for ch in (A, B)}


def threshold_mv(sd):
    return max(1.0, SD_FACTOR * (sd or 0))


def speed_advice(still, threshold):
    mean = still.get("meanMv") or 0
    if mean <= 0:
        return f"Swing it fast: each 10 ms update has to move the reading over {threshold:.0f} mV."
    step_mm = threshold * CARD_MM / mean
    return (f"Swing it fast: each 10 ms update has to move the reading over {threshold:.0f} mV, about {step_mm:.0f} mm "
            f"with the card at {CARD_MM} mm, so {step_mm / GATE_MS:.1f} m/s or faster through the beam.")


def rate(r, label, channel, sensor, still, budget):
    threshold = threshold_mv(still.get("sdMv"))
    advice = speed_advice(still, threshold)
    r.log(f">>> {advice}")
    ready(r, f"swing the fast target through rangefinder {sensor}'s beam 20 times")
    name = f"rate_{label}"
    o = r.run_opmode(POLL, f"rangefinders_{name}",
                     {"mode": "rate", "label": name, "seconds": RATE_S, "channel": channel, "thresholdMv": threshold,
                      "expectedMs": GATE_MS, "maxGapMs": MAX_GAP_MS, "sensorA": A, "sensorB": B},
                     timeout=RATE_S + 60,
                     during=go(r, f"swing the target through rangefinder {sensor}'s beam 20 times now ({RATE_S:.0f} s)"),
                     result_files=[f"analog_poll_{name}"])
    r.log("Stop swinging.")
    if o.get("crashed"):
        raise rb.BenchError(f"{POLL} crashed in its {name} phase: {o.get('errorMessage')!r}")
    res = r.pull(f"analog_poll_{name}")
    phase = (res.get("phases") or {}).get(f"rate {name}") or {}
    return {**{k: v for k, v in phase.items() if k not in ("changeTMs", "changeMv", "what")},
            "channel": channel, "sensor": sensor, "budgetMs": budget, "thresholdMv": threshold, "advice": advice}


def config_diff(values):
    """SAVE_CONFIG for Configure Laser Rangefinder's statics: ints, and its distanceMode enum by name."""
    leaves = {k: ({"__type": "enum", "__value": v} if k == "distanceMode" else {"__type": "int", "__value": v})
              for k, v in values.items()}
    return {"type": "SAVE_CONFIG", "configDiff": {"__type": "custom", "__value": {
        CONFIGURE: {"__type": "custom", "__value": leaves}}}}


def save_statics(r, values):
    """Configure Laser Rangefinder's statics are back at their defaults (a 20 ms budget) after every app restart, so
    they are saved before each START and read back, and it isn't STARTed on anything else."""
    r.dash.send(config_diff(values))
    seq = r.dash.config_seq
    r.dash.send({"type": "GET_CONFIG"})
    r.dash.wait_until(lambda: r.dash.config_seq > seq, 10, "the dashboard's config")
    got = {k: (rb.config_value(r.dash.config_root, (CONFIGURE, k)) or {}).get("__value") for k in values}
    if got != values:
        raise rb.BenchError(f"the dashboard didn't take {CONFIGURE}'s settings: wanted {values}, it reads {got}; "
                            "nothing was written to the sensor")


def settings_for(found, budget=None):
    """Configure Laser Rangefinder's statics that write `found` back exactly, with the timing budget changed if given
    (a measurement period too short for that budget is then written as 0, and noted). The sensor saves whatever it is
    given, so settings it can't write back exactly are refused before anything is written."""
    s = found.get("settings") or {}
    refused = []
    mode = DISTANCE_MODES.get(s.get("distanceModeRaw"))
    if mode is None:
        refused.append(f"its distance mode reads {s.get('distanceModeRaw')}")
    tlx, tly, brx, bry = s["roi"]
    size = brx - tlx + 1
    cx, cy = s["opticalCenter"]
    if size < MIN_ROI or tly - bry + 1 != size or (tlx, tly) != (cx - size // 2, cy + size // 2 - 1):
        refused.append(f"its ROI {s['roi']} isn't a square of 4x4 or more centred on its optical centre {[cx, cy]}")
    elif not all(0 <= v <= ROI_MAX for v in s["roi"]):
        refused.append(f"its ROI {s['roi']} doesn't fit the sensor's 0-{ROI_MAX} grid")
    pin0, pin1 = s["pin0"], s["pin1"]
    if pin0[0] != ANALOG_PIN:
        refused.append(f"pin 0 isn't in analog mode ({pin0})")
    if pin1 != pin0:
        refused.append(f"pin 1 {pin1} differs from pin 0 {pin0}, and {CONFIGURE} writes both the same")
    found_budget, period = s["timing"]
    low_budget, high_budget = BUDGET_LIMITS_MS
    if not low_budget <= found_budget <= high_budget or (period != 0 and period < found_budget + MIN_PERIOD_MARGIN_MS):
        refused.append(f"its timing {s['timing']} (budget, period in ms) isn't one {CONFIGURE} can write")
    if refused:
        raise rb.BenchError(f"rangefinder A's settings can't be written back as found, so nothing was written to it: "
                            f"{'; '.join(refused)}")
    notes = []
    budget = found_budget if budget is None else budget
    if period != 0 and period < budget + MIN_PERIOD_MARGIN_MS:
        notes.append(f"its measurement period {period} ms is too short for a {budget} ms budget, so 0 is written")
        period = 0
    return {"values": {"analogLowMm": pin0[1], "analogHighMm": pin0[2], "distanceMode": mode, "timingBudgetMs": budget,
                       "measurementPeriodMs": period, "roiSize": size},
            "notes": notes}


def read_settings(r, c, label, plug):
    """Bench: Rangefinder Read, INIT only: it reads until rangefinder A answers on I2C and never writes. With plug,
    the person plugs A into I2C port 3 while it reads, so A sees I2C traffic in its first second and leaves analog
    mode."""
    rec = c.setdefault("reads", {})[label] = {}

    def plug_now():
        seen = r.wait_or_crash(lambda: "I2C" in str((r.dash.status or {}).get("warningMessage") or ""), WARNING_WAIT_S)
        rec["i2cWarningSeenS"] = None if seen is None else round(seen, 2)
        rec["warning"] = (r.dash.status or {}).get("warningMessage")
        do(r, "Plug rangefinder A's cable into Control Hub I2C port 3 now.")

    name = f"rangefinder_read_{label}"
    o = r.run_opmode(READ, f"rangefinders_read_{label}", {"label": label, "waitSeconds": READ_WAIT_S}, start=False,
                     timeout=READ_WAIT_S + 30, timeout_ok=True, during_init=plug_now if plug else None,
                     result_files=[name])
    res = r.pull(name, required=False) or {}
    rec.update(answering=res.get("answering") is True, firstAnswerMs=res.get("firstAnswerMs"),
               crashed=bool(o.get("crashed")), error=(o.get("errorMessage") or "") if o.get("crashed") else "")
    if rec["answering"]:
        rec["settings"] = res.get("settings")
        return rec
    return None


def wake_and_read(r, c, label):
    found = read_settings(r, c, label, plug=True)
    if found is None:
        do(r, "Rangefinder A didn't answer on I2C. Unplug its cable from Control Hub I2C port 3 and keep it in your "
              "hand.")
        found = read_settings(r, c, f"{label}_again", plug=True)
    return found


def configure(r, c, key, settings):
    save_statics(r, settings["values"])
    c["touched"] = True
    o = r.run_opmode(CONFIGURE, f"rangefinders_configure_{key}", start=True, timeout=CONFIGURE_HOLD_S, timeout_ok=True)
    shown = {k: v for k, v in dict(r.dash.data).items() if not k.startswith("Bench ")}
    c[key] = {"values": settings["values"], "notes": settings["notes"], "shown": shown,
              "crashed": bool(o.get("crashed")), "error": (o.get("errorMessage") or "") if o.get("crashed") else ""}
    if o.get("crashed"):
        raise rb.BenchError(f"{CONFIGURE} crashed writing rangefinder A: {c[key]['error']!r}")


def unplug_a(r, c):
    where = "analog port 1" if c["aOn"] == "analog" else "I2C port 3"
    do(r, f"Unplug rangefinder A's cable from Control Hub {where} and keep it in your hand.")
    c["aOn"] = "hand"


def restore(r, record, c, found):
    """Writes A's as-found settings back and reads them back, waking A on I2C port 3 before each read: once Configure
    Laser Rangefinder has written its pins, A stops answering on I2C until it is replugged."""
    unplug_a(r, c)
    put_back(record, "A", "rangefinder A's cable into Control Hub analog port 1")
    if r.active_config() != CONFIG_I2C:
        r.activate_config(CONFIG_I2C, rb.hub_config([I2C_LINE]))
    before = wake_and_read(r, c, "before_restore")
    c["aOn"] = "i2c"
    if before is None:
        raise rb.BenchError("rangefinder A didn't answer on I2C for the write back, so it may keep the 20 ms control")
    configure(r, c, "restore", settings_for(found))
    unplug_a(r, c)
    c["readBack"] = wake_and_read(r, c, "read_back")
    c["aOn"] = "i2c"
    do(r, "Move rangefinder A's cable from Control Hub I2C port 3 to analog port 1.")
    c["aOn"] = "analog"
    put_back(record, "A")
    c["restored"] = True


def control(r, record, still_a):
    """The 20 ms control on A: read its settings over I2C, write a 20 ms budget, time it on analog port 1, then write
    the settings it had back, whatever happened in between (Ctrl-C included; only a lost hub, or a second Ctrl-C during
    the write back, leaves it to be done by hand, and the log says how)."""
    c = record["control"] = {"aOn": "analog"}
    if CONFIGURE not in {name for name, _ in r.dash.opmodes or []}:
        c["notRun"] = f"the hub has no OpMode {CONFIGURE!r}"
        return
    do(r, "Unplug the Prism from Control Hub I2C port 3.")
    put_back(record, "Prism", "the Prism into Control Hub I2C port 3")
    unplug_a(r, c)
    put_back(record, "A", "rangefinder A's cable into Control Hub analog port 1")
    r.activate_config(CONFIG_I2C, rb.hub_config([I2C_LINE]))
    found = wake_and_read(r, c, "as_found")
    if found is None:
        skip_control(r, record, c, "rangefinder A didn't answer on I2C twice, so nothing was written to it")
        return
    c["aOn"] = "i2c"
    c["asFound"] = found["settings"]
    (r.out / "raw" / "rangefinder_asfound.json").write_text(json.dumps({"A": found["settings"]}, indent=1))
    try:
        settings_for(found)
        write_20 = settings_for(found, budget=CONTROL_MS)
    except rb.BenchError as e:
        skip_control(r, record, c, str(e))
        return
    try:
        configure(r, c, "write_20", write_20)
        do(r, "Move rangefinder A's cable from Control Hub I2C port 3 to analog port 1.")
        c["aOn"] = "analog"
        put_back(record, "A")
        r.activate_config(CONFIG_ANALOG, rb.hub_config(analog_lines()))
        record["rate"]["control_A"] = rate(r, "control_A", A, "A", still_a, CONTROL_MS)
    except (rb.ProtocolError, rb.OtherHub):
        if c.get("touched"):
            r.log(LEFT_WRITTEN)
        raise
    except BaseException as e:
        if not c.get("touched"):
            raise
        c["failedBeforeRestore"] = f"{type(e).__name__}: {e}"
        r.log(f"!! the control stopped ({type(e).__name__}: {e}); writing rangefinder A's as-found settings back first "
              "(Ctrl-C again to skip that)")
        try:
            # A Ctrl-C just after an INIT is sent can leave that OpMode in INIT, unseen by run_opmode's STOP.
            r.settle()
            restore(r, record, c, found)
        except BaseException:
            r.log(LEFT_WRITTEN)
            raise
        raise
    try:
        restore(r, record, c, found)
    except BaseException:
        r.log(LEFT_WRITTEN)
        raise
    prism_back(r, record)


def skip_control(r, record, c, why):
    c["notRun"] = why
    do(r, "Move rangefinder A's cable from Control Hub I2C port 3 to analog port 1.")
    c["aOn"] = "analog"
    put_back(record, "A")
    prism_back(r, record)


def prism_back(r, record):
    do(r, "Plug the Prism back into Control Hub I2C port 3.")
    put_back(record, "Prism")


def arrangement(r, record, key, still_s):
    entry = record["arrangements"][key]
    do(r, "Aim both rangefinders at the matte card about 150 mm away again, and keep everything still.")
    entry["still"] = steady(r, f"{key} still", still_s)
    ready(r, f"flex both cables along their whole length, one in each hand, for {FLEX_S:.0f} s")
    entry["flex"] = steady(r, f"{key} flex", FLEX_S, during=go(r, f"flex both cables now ({FLEX_S:.0f} s)"))
    r.log("Stop flexing and let go.")


def run(r, record):
    record.update(arrangements={key: {"what": what} for key, what, _ in ARRANGEMENTS}, rate={}, putBack={})
    try:
        steps(r, record)
    except BaseException:
        if record["putBack"]:
            r.log(f"!! Put back by hand: {'; '.join(record['putBack'].values())}")
        raise


def steps(r, record):
    analog = analog_lines()
    do(r, "Aim rangefinder A (Control Hub analog port 1) and rangefinder B (analog port 3) at a matte card about 150 "
          "mm away, and keep everything still.")
    r.activate_config(CONFIG_ANALOG, rb.hub_config(analog))
    base = record["arrangements"]["base"]
    base["still"] = steady(r, "base still", STILL_S)
    dead = [f"rangefinder {s} on Control Hub analog port {PORT[ch]}" for ch, s in ((A, "A"), (B, "B"))
            if (base["still"][ch].get("maxMv") or 0) < LOW_MV]
    if dead:
        raise rb.BenchError(f"{' and '.join(dead)} read under {LOW_MV} mV the whole time: plug it in, aimed at the card")
    record["stillSdMv"] = {ch: base["still"][ch].get("sdMv") for ch in (A, B)}
    for ch, sensor in ((A, "A"), (B, "B")):
        record["rate"][f"stored_{sensor}"] = rate(r, f"stored_{sensor}", ch, sensor, base["still"][ch], BUDGET_MS)
    control(r, record, base["still"][A])
    if r.active_config() != CONFIG_ANALOG:
        r.activate_config(CONFIG_ANALOG, rb.hub_config(analog))
    for ch, sensor in ((A, "A"), (B, "B")):
        ready(r, f"flex rangefinder {sensor}'s cable along its whole length for {FLEX_S:.0f} s")
        base[f"flex {sensor}"] = steady(r, f"base flex {sensor}", FLEX_S,
                                        during=go(r, f"flex rangefinder {sensor}'s cable end to end now"))
        r.log("Stop flexing and let go.")
    do(r, "Swap the two sensors at the sensor end: sensor A onto cable B, sensor B onto cable A. Leave both cables in "
          "the hub.")
    put_back(record, "sensors", "each sensor back on its own cable")
    arrangement(r, record, "sensors swapped", SWAP_STILL_S)
    do(r, "Put each sensor back on its own cable: sensor A on cable A, sensor B on cable B.")
    put_back(record, "sensors")
    do(r, "Swap the two cables at the hub: cable A into Control Hub analog port 3, cable B into analog port 1.")
    put_back(record, "cables", "cable A into Control Hub analog port 1 and cable B into analog port 3")
    arrangement(r, record, "cables swapped", SWAP_STILL_S)
    do(r, "Swap the cables back at the hub: cable A into Control Hub analog port 1, cable B into analog port 3.")
    put_back(record, "cables")


def channel_fault(stats):
    return bool(stats) and ((stats.get("belowLowMv") or 0) > 0 or (stats.get("stepsOverStepMv") or 0) > 0)


def still_check(record, ch):
    stats = ((record.get("arrangements") or {}).get("base") or {}).get("still", {}).get(ch) or {}
    if not stats.get("count"):
        return False, "no still samples"
    spread = stats["p99Mv"] - stats["p1Mv"]
    return (spread < SPREAD_MV and stats.get("belowLowMv") == 0,
            f"p1-p99 {stats['p1Mv']}-{stats['p99Mv']} mV (spread {spread}), {stats.get('belowLowMv')} readings under "
            f"{LOW_MV} mV, min {stats.get('minMv')}, max {stats.get('maxMv')}, SD {stats.get('sdMv', 0):.1f} mV over "
            f"{stats['count']} samples")


def motion(rec):
    steps, fast = rec.get("timedSteps") or 0, rec.get("fastFraction") or 0
    return steps >= MIN_STEPS and fast >= MIN_FAST, (f"{steps} timed level steps, {100 * fast:.0f}% fast enough "
                                                     f"(need {MIN_STEPS} and {100 * MIN_FAST:.0f}%)")


def rate_check(rec, budget):
    if not rec:
        return False, "not run"
    enough, how = motion(rec)
    if not enough:
        return False, f"not enough motion, no verdict: {how}"
    low, high = DWELL_RANGE[budget]
    p50 = rec.get("dwellP50Ms")
    return (p50 is not None and low <= p50 <= high,
            f"dwell p50 {p50} ms (p10 {rec.get('dwellP10Ms')}, p90 {rec.get('dwellP90Ms')}), want {low:g}-{high:g} "
            f"at {budget} ms; {how}; {rec.get('stepsPerSecond', 0):.0f} level steps/s")


def readback_check(record):
    c = record.get("control") or {}
    if c.get("notRun"):
        return False, f"not run: {c['notRun']}"
    found, back = c.get("asFound"), (c.get("readBack") or {}).get("settings")
    if found is None or back is None:
        return False, ("no read-back: " + (c.get("failedBeforeRestore") or "the write back didn't finish"))
    differ = {f: (found.get(f), back.get(f)) for f in READ_FIELDS if found.get(f) != back.get(f)}
    return not differ, (f"read back {[back.get(f) for f in READ_FIELDS]}" if not differ
                        else f"differs from raw/rangefinder_asfound.json (as found, read back): {differ}")


def faults_by_arrangement(record):
    """{arrangement: {hub channel: whether any still or flex phase there read under LOW_MV or stepped over STEP_MV}}."""
    out = {}
    for key, _, _ in ARRANGEMENTS:
        entry = (record.get("arrangements") or {}).get(key) or {}
        phases = [v for k, v in entry.items() if k != "what" and isinstance(v, dict)]
        if phases:
            out[key] = {ch: any(channel_fault(p.get(ch)) for p in phases) for ch in (A, B)}
    return out


def bad_part(record):
    """(part or None, detail): the one sensor, cable or hub port present in every channel that faulted and in none
    that stayed clean."""
    faults = faults_by_arrangement(record)
    if len(faults) < len(ARRANGEMENTS):
        return None, f"not every arrangement ran ({sorted(faults)})"
    parts = {key: {ch: set(sc) | {f"Control Hub analog port {PORT[ch]}"} for ch, sc in wiring.items()}
             for key, _, wiring in ARRANGEMENTS}
    bad = [parts[k][ch] for k, chans in faults.items() for ch, f in chans.items() if f]
    good = [parts[k][ch] for k, chans in faults.items() for ch, f in chans.items() if not f]
    seen = {k: [f"port {PORT[ch]}" for ch, f in chans.items() if f] for k, chans in faults.items()}
    if not bad:
        return None, (f"swing not reproduced: no reading under {LOW_MV} mV and no step over {STEP_MV} mV in any "
                      "arrangement")
    left = set.intersection(*bad) - set().union(*good)
    if len(left) == 1:
        return next(iter(left)), f"the fault follows {next(iter(left))} (faulted: {seen})"
    return None, f"the fault didn't follow one part (faulted: {seen}; candidates {sorted(left) or 'none'})"


def checks(record):
    rates = record.get("rate") or {}
    budget = (((record.get("control") or {}).get("asFound") or {}).get("timing") or [None])[0]
    stored = f" (A's as-found budget {budget} ms)" if budget is not None else ""
    part, detail = bad_part(record)
    rate_a = rate_check(rates.get("stored_A"), BUDGET_MS)
    return [
        ("rangefinders: still: rangefinder A spreads under 50 mV and never reads under 50 mV", *still_check(record, A),
         STILL_MEANS),
        ("rangefinders: still: rangefinder B spreads under 50 mV and never reads under 50 mV", *still_check(record, B),
         STILL_MEANS),
        ("rangefinders: rate: rangefinder A's analog output steps about every 10 ms",
         rate_a[0], rate_a[1] + stored, RATE_MEANS),
        ("rangefinders: rate: rangefinder B's analog output steps about every 10 ms",
         *rate_check(rates.get("stored_B"), BUDGET_MS), RATE_MEANS),
        ("rangefinders: control: at a 20 ms budget rangefinder A steps about every 20 ms",
         *(rate_check(rates.get("control_A"), CONTROL_MS) if rates.get("control_A")
           else (False, f"not run: {(record.get('control') or {}).get('notRun') or 'the control stopped'}")),
         CONTROL_MEANS),
        ("rangefinders: control: rangefinder A reads back its as-found settings", *readback_check(record),
         READBACK_MEANS),
        ("rangefinders: swaps: a fault in the swaps names one sensor, cable or port", part is not None, detail,
         SWAPS_MEANS),
    ]


def fmt(v, digits=0):
    return "" if v is None else f"{v:.{digits}f}"


def summary(record):
    s = ["## Bench: Rangefinders (laser rangefinders still, swept, flexed and swapped)\n"]
    if record.get("putBack"):
        s.append(f"**Put back by hand: {'; '.join(record['putBack'].values())}**\n")
    s += ["| arrangement | phase | hub port | sensor, cable | mean mV | SD | p1-p99 | min-max | under 50 mV | steps over "
          "200 mV |", "|---|---|---|---|---|---|---|---|---|---|"]
    for key, _, wiring in ARRANGEMENTS:
        entry = (record.get("arrangements") or {}).get(key) or {}
        for label, phase in entry.items():
            if label == "what" or not isinstance(phase, dict):
                continue
            for ch in (A, B):
                st = phase.get(ch) or {}
                s.append(f"| {key} | {label} | {PORT[ch]} | {', '.join(wiring[ch])} | {fmt(st.get('meanMv'))} | "
                         f"{fmt(st.get('sdMv'), 1)} | {st.get('p1Mv', '')}-{st.get('p99Mv', '')} | "
                         f"{st.get('minMv', '')}-{st.get('maxMv', '')} | {st.get('belowLowMv', '')} | "
                         f"{st.get('stepsOverStepMv', '')} |")
    s += ["", "| rate run | budget | level steps | fast enough | dwell p10/p50/p90 ms | steps/s | threshold mV |",
          "|---|---|---|---|---|---|---|"]
    for key, rec in (record.get("rate") or {}).items():
        s.append(f"| {key} ({rec.get('channel')}) | {rec.get('budgetMs')} ms | {rec.get('timedSteps')} | "
                 f"{fmt(100 * (rec.get('fastFraction') or 0))}% | {fmt(rec.get('dwellP10Ms'), 1)}/"
                 f"{fmt(rec.get('dwellP50Ms'), 1)}/{fmt(rec.get('dwellP90Ms'), 1)} | {fmt(rec.get('stepsPerSecond'))} | "
                 f"{fmt(rec.get('thresholdMv'), 1)} |")
    s.append("")
    c = record.get("control") or {}
    if c.get("notRun"):
        s.append(f"- Control: not run: {c['notRun']}")
    if c.get("asFound") is not None:
        s.append(f"- Rangefinder A as found: {[c['asFound'].get(f) for f in READ_FIELDS]} ({', '.join(READ_FIELDS)})")
    if c.get("write_20"):
        s.append(f"- Written for the control: {c['write_20']['values']}" + (f"; {'; '.join(c['write_20']['notes'])}"
                                                                           if c["write_20"]["notes"] else ""))
    back = (c.get("readBack") or {}).get("settings")
    if back is not None:
        s.append(f"- Read back after writing the as-found values: {[back.get(f) for f in READ_FIELDS]}")
    for label, rec in (c.get("reads") or {}).items():
        s.append(f"- Read {label}: answering {rec.get('answering')}, first answer {rec.get('firstAnswerMs')} ms into "
                 "INIT" + (f"; I2C warning seen after {rec.get('i2cWarningSeenS')} s ({rec.get('warning')!r})"
                           if "warning" in rec else ""))
    s.append(f"- Swaps: {bad_part(record)[1]}")
    s.append("")
    return s


def needs(r):
    if not r.args.interactive:
        return ("Bench: Rangefinders asks you to swing a target, move and flex cables and swap the sensors, so it needs "
                "--interactive and a terminal")
    return None


BENCH = rb.Bench(
    name="rangefinders",
    title="Bench: Rangefinders",
    flag="--rangefinders",
    help="Bench: Rangefinders: both laser rangefinders still, swept by a fast target, a 20 ms control on sensor A over "
         "I2C, cable flexes and swaps (about 25 min; rangefinder A on Control Hub analog port 1, B on port 3; needs "
         "--interactive and a terminal)",
    run=run,
    checks=checks,
    summary=summary,
    needs=needs,
    person=rb.Person(
        "Bench: Rangefinders: you swing a target through the beams, flex and swap the cables, and move sensor A to "
        "I2C and back twice",
        "rangefinder A on Control Hub analog port 1 and B on analog port 3, aimed at a matte card about 150 mm away; "
        "a fast target (a card on a swinging arm through 50-300 mm, or a slotted disc spun by hand); the Prism in "
        "Control Hub I2C port 3 to unplug; nothing on any motor or servo port; about 25 min",
        flags=["--rangefinders", "--interactive"]),
    configs=(CONFIG_ANALOG, CONFIG_I2C),
    opmodes=(POLL, READ),
)

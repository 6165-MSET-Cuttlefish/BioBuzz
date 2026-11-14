"""Bench: Fault Render: a person reads the Driver Station and answers y or n. Bench: Limelight Fault's FAULT line, then
Bench: Fault Render's phases (a short fault, a 150-character one, two faults over ten normal lines, a 1 s Driver Station
interval, Driver Station telemetry off), then its first frame after INIT (scripts/bench/README.md)."""
import shutil
from pathlib import Path

OPMODE = "Bench: Fault Render"
CUE_PATH = (OPMODE, "cue")
PHASES = ("short", "long", "two", "slow", "off")
PHASE_S = 15.0
RUN1_WAIT_S = 5.0
FIRST_FRAME_WAIT_S = 10.0
DS_REMOTE = "/sdcard/bench_ds_screencap.png"
YES, NO = "y", "n"
EXPECTED = {"llfault": YES, "short": YES, "long": YES, "two": YES, "slow": YES, "off": NO, "modules": YES,
            "firstFrame": YES}
ANSWER_HOW = "Type y or n, then Enter (add the file path of a photo after it if you took one)."
QUESTIONS = {
    "llfault": "Look at the Driver Station: is FAULT Limelight the first telemetry line, in red?",
    "short": "Is the first line on the Driver Station FAULT Bench: a short test message, in red and larger than the "
             "lines under it?",
    "long": "Can you read the whole FAULT Bench message on the Driver Station, down to the word END at its end?",
    "two": "Are FAULT Bench and then FAULT Second the first two lines, both red, above the rows Line 1 to Line 10?",
    "slow": "With the Driver Station now updating once a second, is FAULT Bench still the first line, in red, its "
            "seconds counting up?",
    "off": "Driver Station telemetry is now switched off. Is there a line FAULT Hidden anywhere on the Driver Station?",
    "modules": "Is there a MODULES header with the line RenderStub: READY under it?",
    "firstFrame": "Bench: Fault Render has just been INITed and the Driver Station keeps its first frame. Is that "
                  "frame drawn with colours and bold text, with no raw tags like <b>, <font> or <big> showing?",
}
CHECK_NAMES = {
    "llfault": "Bench: Limelight Fault's FAULT Limelight is the first Driver Station line, in red",
    "short": "a short fault is the first Driver Station line, red and larger",
    "long": "a 150-character fault message is readable to its end on the Driver Station",
    "two": "two faults lead the Driver Station, above ten normal lines",
    "slow": "the fault still leads at a 1000 ms Driver Station interval",
    "off": "with Driver Station telemetry off, no new fault line reaches the Driver Station",
    "modules": "the MODULES header is followed by the stub module's line",
    "firstFrame": "the first Driver Station frame after INIT shows rendered markup, not raw tags",
}
FAIL_MEANS = {
    "llfault": "a driver at an event, where the Driver Station may be the only screen, wouldn't see that the "
               "Limelight is gone: the FAULT line isn't first or isn't red there",
    "short": "the FAULT line doesn't stand out on the Driver Station (not first, not red or not larger)",
    "long": "a long fault message is cut off on the Driver Station, so its end, often the useful part, is lost",
    "two": "with several faults and other lines, a fault can drop below other telemetry or lose its colour",
    "slow": "at a slower Driver Station interval the FAULT line is lost or loses its place",
    "off": "Driver Station telemetry switched off still sends lines, so OptimizationToggles.dsTelemetry doesn't "
           "hold the Driver Station frame",
    "modules": "the MODULES group doesn't render on the Driver Station as EnhancedOpMode builds it",
    "firstFrame": "the first frame after INIT shows raw HTML tags: the Driver Station's display format isn't HTML "
                  "before the first update",
}
SETUP_MEANS = ("Bench: Fault Render didn't reach every phase on the runner's cue (it crashed, or the dashboard didn't "
               "take the cue), so the answers after that phase aren't about what the question says")


def parse_answer(typed):
    """('y' or 'n', photo path or None) from what the person typed, or None when it starts with neither."""
    words = (typed or "").strip().split(None, 1)
    if not words:
        return None
    first = words[0].lower()
    answer = YES if first in ("y", "yes") else NO if first in ("n", "no") else None
    if answer is None:
        return None
    return answer, (words[1].strip() if len(words) > 1 else None)


def yes_no(r, record, key):
    """Asks until the person types y or n; a closed terminal stops the bench, never taken as an answer."""
    while True:
        try:
            typed = r.answer(f">>> {QUESTIONS[key]} {ANSWER_HOW}")
        except EOFError:
            raise rb.BenchError(f"the terminal closed before the answer to: {QUESTIONS[key]}") from None
        parsed = parse_answer(typed)
        if parsed is not None:
            break
        r.log(f"{typed!r} isn't y or n; asking again")
    answer, photo = parsed
    entry = {"answer": answer, "typed": typed}
    if photo:
        entry["photo"] = photo
        path = Path(photo).expanduser()
        if path.is_file():
            local = r.out / "raw" / f"ds_photo_{key}{path.suffix}"
            shutil.copyfile(path, local)
            entry["photoCopy"] = f"raw/{local.name}"
        else:
            entry["photoMissing"] = True
    record.setdefault("answers", {})[key] = entry
    return answer


def cue_diff(value):
    node = {"__type": "int", "__value": value}
    for key in reversed(CUE_PATH):
        node = {"__type": "custom", "__value": {key: node}}
    return {"type": "SAVE_CONFIG", "configDiff": node}


def set_cue(r, value):
    """Saves Bench: Fault Render's cue through the dashboard and reads the config back, so a person is never asked
    about a phase the OpMode isn't in."""
    r.dash.send(cue_diff(value))
    seq = r.dash.config_seq
    r.dash.send({"type": "GET_CONFIG"})
    r.dash.wait_until(lambda: r.dash.config_seq > seq, 10, "the dashboard's config")
    node = rb.config_value(r.dash.config_root, CUE_PATH) or {}
    if node.get("__value") != value:
        raise rb.BenchError(f"the dashboard didn't take {OPMODE}'s cue {value}: its config reads {node!r}")


def driver_hub(r, record):
    """The Driver Hub's own adb, for screenshots, when --fault-render-ds-serial names one adb lists apart from the
    Control Hub; else None."""
    serial = r.args.fault_render_ds_serial
    if serial is None:
        return None
    if serial == r.adb.serial:
        raise rb.BenchError(f"--fault-render-ds-serial {serial} is the Control Hub's own adb serial: pass the Driver "
                            "Hub's, from `adb devices`")
    state = {row[0]: row[1] for row in r.adb.devices()}.get(serial)
    if state != "device":
        raise rb.BenchError(f"adb doesn't list the Driver Hub {serial} as a device ({state or 'not listed'}): plug its "
                            "USB cable into this Mac and accept its debugging prompt, or drop --fault-render-ds-serial")
    ds = rb.Adb(serial, r.log)
    if ds.run("shell", f"getprop {rb.CONTROL_HUB_PROP}").strip() in rb.PROP_TRUE:
        raise rb.BenchError(f"{serial} is a Control Hub, not a Driver Hub: pass the Driver Hub's adb serial")
    record["driverHub"] = serial
    return ds


def screencap(r, ds, record, key, name):
    if ds is None:
        return
    local = r.out / "raw" / name
    try:
        ds.run("shell", f"screencap -p {DS_REMOTE}")
        ds.run("pull", DS_REMOTE, str(local))
        ds.run("shell", f"rm -f {DS_REMOTE}", check=False)
    except (rb.ProtocolError, rb.OtherHub):
        raise
    except rb.BenchError as e:
        record.setdefault("screencapErrors", {})[key] = str(e)
        return
    record.setdefault("screencaps", {})[key] = f"raw/{name}"


def outcome(o):
    return {"crashed": bool(o.get("crashed")), "error": o.get("errorMessage") or "" if o.get("crashed") else "",
            "timedOut": bool(o.get("timedOut"))}


def run_llfault(r, record, ds):
    ip = rb.LIMELIGHT_CONFIG_IP if r.limelight is None else rb.UNREACHABLE_IP
    tag = rb.limelight_tag(ip)
    record["llfaultConfigLine"] = tag
    r.activate_config(rb.CONFIG_LLFAULT, rb.hub_config(rb.main_devices(r.imu_type), [tag]))

    def ask():
        r.wait_or_crash(lambda: False, RUN1_WAIT_S)
        if r.crashed():
            return
        screencap(r, ds, record, "llfault", "ds_fault.png")
        yes_no(r, record, "llfault")

    o = r.run_opmode("Bench: Limelight Fault", "fault_render_llfault",
                     {"mode": "absent", "runSeconds": rb.LLFAULT_RUN_S, "creep": 0}, init_hold=rb.LLFAULT_INIT_HOLD_S,
                     timeout=rb.LLFAULT_RUN_S + 60, result_files=["limelight_fault_absent"], during=ask,
                     quiet_after=5.0)
    record["llfault"] = outcome(o)
    res = r.pull("limelight_fault_absent", required=False, local_name="fault_render_llfault") or {}
    record["llfault"]["firstFaultMs"] = res.get("firstFaultMs")


def run_phases(r, record, ds):
    def script():
        for i, name in enumerate(PHASES, 1):
            set_cue(r, i)
            r.wait_or_crash(lambda: False, PHASE_S)
            if r.crashed():
                return
            screencap(r, ds, record, name, f"ds_{name}.png")
            yes_no(r, record, name)
            if name == "short":
                yes_no(r, record, "modules")

    o = r.run_opmode(OPMODE, "fault_render_phases", {"mode": "phases"}, timeout=60, during=script,
                     result_files=["fault_render_phases"], quiet_after=10.0)
    record["phasesRun"] = outcome(o)
    res = r.pull("fault_render_phases", required=False) or {}
    record["phases"] = {name: (res.get("phases") or {}).get(name) for name in ("waiting",) + PHASES}
    record["longMessage"] = (res.get("info") or {}).get("longMessage")


def run_first_frame(r, record, ds):
    def ask():
        held = r.wait_or_crash(lambda: r.dash.done_line is not None, FIRST_FRAME_WAIT_S)
        record["firstFrameHeldS"] = held
        if held is None:
            return
        screencap(r, ds, record, "firstFrame", "ds_first_frame.png")
        yes_no(r, record, "firstFrame")

    o = r.run_opmode(OPMODE, "fault_render_first_frame", {"mode": "firstFrame"}, start=False, timeout=30,
                     timeout_ok=True, during_init=ask, result_files=["fault_render_firstFrame"])
    record["firstFrameRun"] = outcome(o)
    res = r.pull("fault_render_firstFrame", required=False) or {}
    record["firstFrameDsFrames"] = res.get("dsFramesOpened")


def run(r, record):
    record["answers"] = {}
    ds = driver_hub(r, record)
    r.log(f"{OPMODE}: look at the Driver Station paired with this Control Hub; each question waits for your answer")
    run_llfault(r, record, ds)
    r.activate_config(rb.CONFIG_MAIN, r.main_config())
    run_phases(r, record, ds)
    run_first_frame(r, record, ds)


def answer_check(record, key):
    entry = (record.get("answers") or {}).get(key)
    if entry is None:
        return False, "not asked: its OpMode crashed or stopped before the question"
    expected = EXPECTED[key]
    shot = (record.get("screencaps") or {}).get(key)
    detail = (f"answered {entry['answer']!r}, expected {expected!r}"
              + (f"; photo {entry.get('photoCopy') or entry.get('photo')}" if entry.get("photo") else "")
              + (f"; screenshot {shot}" if shot else ""))
    return entry["answer"] == expected, detail


def setup_check(record):
    phases = record.get("phases") or {}
    reached = [name for name in PHASES if phases.get(name)]
    run = record.get("phasesRun") or {}
    ok = reached == list(PHASES) and not run.get("crashed")
    detail = f"phases with a result: {reached or 'none'}" + (f"; crashed: {run.get('error')!r}" if run.get("crashed")
                                                             else "")
    return ok, detail


def checks(record):
    out = [(f"fault render: {OPMODE} went through its five phases on the runner's cue", *setup_check(record),
            SETUP_MEANS)]
    llfault = record.get("llfault") or {}
    ok, detail = answer_check(record, "llfault")
    if llfault.get("crashed"):
        ok, detail = False, f"Bench: Limelight Fault crashed: {llfault.get('error')!r}"
    elif not (record.get("screencaps") or {}).get("llfault"):
        detail += ("; no screenshot: " + ((record.get("screencapErrors") or {}).get("llfault")
                                          or "pass --fault-render-ds-serial with the Driver Hub on adb for one"))
    out.append((f"fault render: {CHECK_NAMES['llfault']}", ok, detail, FAIL_MEANS["llfault"]))
    for key in PHASES + ("modules", "firstFrame"):
        out.append((f"fault render: {CHECK_NAMES[key]}", *answer_check(record, key), FAIL_MEANS[key]))
    return out


def fmt_rate(value):
    return "" if value is None else f"{value:.1f}"


def summary(record):
    s = ["## Bench: Fault Render (the Driver Station read by a person)\n"]
    if record.get("driverHub"):
        s.append(f"- Driver Hub on adb: {record['driverHub']}; screenshots in raw/")
    else:
        s.append("- No Driver Hub on adb (`--fault-render-ds-serial`): no screenshots")
    llfault = record.get("llfault") or {}
    s.append(f"- Bench: Limelight Fault on `{record.get('llfaultConfigLine')}`: first fault at "
             f"{llfault.get('firstFaultMs')} ms; crashed {llfault.get('crashed')}")
    s += ["", "| question | answer | expected | DS frames/s | fault line HTML chars | screenshot or photo |",
          "|---|---|---|---|---|---|"]
    answers = record.get("answers") or {}
    phases = record.get("phases") or {}
    for key in ("llfault",) + PHASES + ("modules", "firstFrame"):
        entry = answers.get(key) or {}
        phase = phases.get(key) or {}
        evidence = (record.get("screencaps") or {}).get(key) or entry.get("photoCopy") or entry.get("photo") or ""
        s.append(f"| {CHECK_NAMES[key]} | {entry.get('answer', 'not asked')} | {EXPECTED[key]} | "
                 f"{fmt_rate(phase.get('dsFramesPerSecond'))} | {phase.get('faultLineHtmlChars', '')} | {evidence} |")
    s.append("")
    waiting = phases.get("waiting") or {}
    s.append(f"- Driver Station frames opened per second before the first phase: "
             f"{fmt_rate(waiting.get('dsFramesPerSecond')) or 'n/a'}; frames opened in the first-frame run: "
             f"{record.get('firstFrameDsFrames')}")
    if record.get("screencapErrors"):
        s.append(f"- screenshots that failed: {record['screencapErrors']}")
    s.append("")
    return s


def needs(r):
    if not r.args.interactive:
        return "Bench: Fault Render asks you what the Driver Station shows, so it needs --interactive and a terminal"
    return None


BENCH = rb.Bench(
    name="fault_render",
    title="Bench: Fault Render",
    flag="--fault-render",
    help="Bench: Fault Render: you read the Driver Station and answer y or n eight times (about 8 min; a Driver Hub "
         "paired with the Control Hub; needs --interactive)",
    options=(("--fault-render-ds-serial", {
        "default": None, "metavar": "SERIAL",
        "help": "with --fault-render: the Driver Hub's adb serial (`adb devices`, its USB cable on this Mac), to save "
                "a screenshot of it at each question"}),),
    run=run,
    checks=checks,
    summary=summary,
    needs=needs,
    person=rb.Person(
        "Bench: Fault Render: you read the Driver Station and answer y or n eight times",
        "a Driver Hub paired with the Control Hub and showing its telemetry (on this Mac's USB if you can, for "
        "screenshots); about 8 min",
        flags=["--fault-render", "--interactive"]),
    configs=(rb.CONFIG_LLFAULT,),
    opmodes=("Bench: Limelight Fault", OPMODE),
)

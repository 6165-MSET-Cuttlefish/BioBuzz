"""Bench: Camera Power Loss: the powered USB hub that carries the webcam and the Limelight loses its power bank, before
INIT of BioBuzz Tele (after a control INIT with it powered) and mid-run under Bench: Full Load Soak
(scripts/bench/README.md)."""
import re
from xml.etree import ElementTree

TELE = "BioBuzz Tele"
SOAK = "Bench: Full Load Soak"
CONFIG_CAM = "bench_biobuzz_cam"
SHIPPED = "biobuzz"
SOAK_RESULT = "full_load_soak_powerloss"
EXPANSION_ADDRESS = 2
HOLD_S = 6.0
HANG_S = 10.0
BACK_WAIT_S = 180.0
PROBE_EVERY_S = 10.0
HEALTHY_WAIT_S = 30.0
BEFORE_PULL_S = 20.0
FAULT_SEEN_WAIT_S = 15.0
AFTER_PULL_S = 10.0
AFTER_REPLUG_S = 60.0
FAULT_LIMIT_MS = 1000.0
CAMERA_WORDS = ("webcam", "camera", "usb", "uvc", "limelight", "apriltagdetector", "ethernet", "failed to open")
TEAMCODE_FRAME = re.compile(r"\bat (org\.firstinspires\.ftc\.teamcode\.[\w.$]+)\.([\w$<>]+)\(([^)]*)\)")
UNPLUG = "Unplug the power bank's cable from the powered USB hub."
REPLUG = "Plug the power bank's cable back into the powered USB hub."

CONTROL_MEANS = ("setup: with the camera hub powered, BioBuzz Tele didn't hold INIT on bench_biobuzz_cam (biobuzz.xml "
                 "with this station's webcam and Limelight), so an INIT with the cameras unpowered can't be told apart "
                 "from it; the error says which device or step stopped it")
UNPLUGGED_MEANS = ("with the camera hub's power bank out, BioBuzz Tele's INIT went as it did with it in, or the probe "
                   "still found a camera, so this run can't say what a flat power bank does at INIT")
FAULT_MEANS = (f"the Limelight fault didn't come within {FAULT_LIMIT_MS / 1000:g} s of the Limelight's last good poll "
               "when the camera hub lost power mid-run, so the driver would see the fallback late or not at all")
CRASH_MEANS = ("the OpMode crashed when the camera hub lost power mid-run: in a match that stops the robot (the error "
               "and the first team-code frame say where)")


def shipped_xml():
    return (rb.REPO_ROOT / rb.SHIPPED_CONFIG_DIR / f"{SHIPPED}.xml").read_text()


def camera_config(xml_text, webcam_serial, limelight):
    """biobuzz.xml with the webcam CellTipCamera opens set to this station's serial and the Limelight to this
    station's address, each added when the config has none."""
    root = ElementTree.fromstring(xml_text)
    webcam = next((e for e in root.iter("Webcam") if e.get("name") == rb.CELL_TIP_WEBCAM), None)
    if webcam is None:
        webcam = ElementTree.SubElement(root, "Webcam", {"name": rb.CELL_TIP_WEBCAM})
    webcam.set("serialNumber", webcam_serial)
    ll = next((e for e in root.iter("EthernetDevice") if e.get("name") == rb.LIMELIGHT_DEVICE), None)
    if ll is None:
        ll = ElementTree.SubElement(root, "EthernetDevice", {"name": rb.LIMELIGHT_DEVICE})
    ll.set("serialNumber", limelight["serial"])
    ll.set("port", ll.get("port") or "0")
    ll.set("ipAddress", limelight["ipAddress"])
    return "<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>\n" + ElementTree.tostring(root, "unicode") + "\n"


def cameras_in(probe):
    return {"webcams": sorted(w.get("serial") for w in (probe or {}).get("webcams") or [] if w.get("attached")),
            "limelights": sorted(l.get("ipAddress") for l in (probe or {}).get("limelights") or []
                                 if not str(l.get("status")).startswith("unreachable"))}


def stack_frames(log_text):
    """The team-code frames in a step's logcat, first first: (class, method, file:line)."""
    return [m.groups() for m in TEAMCODE_FRAME.finditer(log_text or "")]


def ask(r, text):
    try:
        r.ask(f">>> {text} Then press Enter.")
    except EOFError:
        raise rb.BenchError(f"the terminal closed before you confirmed: {text}") from None


def init_tele(r, step):
    """BioBuzz Tele INITed, held HOLD_S and STOPped, never STARTed."""
    try:
        o = r.run_opmode(TELE, step, start=False, timeout=HOLD_S, timeout_ok=True)
    except (rb.ProtocolError, rb.OtherHub):
        raise
    except rb.BenchError as e:
        r.settle()
        return {"failed": str(e)}
    log = r.out / "logs" / f"{step}.log"
    frames = stack_frames(log.read_text() if log.exists() else "")
    error = (o.get("errorMessage") or "") if o.get("crashed") else ""
    constructor = next((f for f in frames if f[1] == "<init>"), None)
    return {"crashed": bool(o.get("crashed")), "error": error,
            "initSeenS": round(o["initSeenSeconds"], 2) if o.get("initSeenSeconds") is not None else None,
            "stopS": round(o["stopSecondsRunnerSide"], 2) if o.get("stopSecondsRunnerSide") is not None else None,
            "pidBefore": o.get("pidBefore"), "pidAfter": o.get("pidAfter"), "appRestarted": bool(o.get("appRestarted")),
            "stackTop": ".".join(frames[0][:2]) + f" ({frames[0][2]})" if frames and error else None,
            "constructor": f"{constructor[0]}.<init> ({constructor[2]})" if constructor and error else None,
            "namesCamera": any(w in error.lower() for w in CAMERA_WORDS)}


def probe(r, step):
    r.run_opmode("Bench: Probe", step, timeout=60, result_files=["probe"])
    return cameras_in(r.pull("probe", local_name=step))


def wait_cameras(r, record):
    """Bench: Probe every PROBE_EVERY_S until the run's webcam and Limelight are both back, or BACK_WAIT_S."""
    t0 = rb.time.monotonic()
    tries = []
    while True:
        found = probe(r, f"camera_power_back_{len(tries) + 1}")
        waited = round(rb.time.monotonic() - t0, 1)
        tries.append(dict(found, s=waited))
        back = r.webcam in found["webcams"] and r.limelight["ipAddress"] in found["limelights"]
        if back or waited > BACK_WAIT_S:
            record["back"] = {"both": back, "seconds": waited, "probes": tries}
            return back
        r.log(f"waiting for the cameras: found {found}")
        rb.time.sleep(PROBE_EVERY_S)


def midrun(r, record):
    watch = rb.FaultWatch(rb.LIMELIGHT_FAULT_KEY)
    rec = record["midrun"] = {}

    def prompt(text):
        ask(r, text)
        record["bankOut"] = text == UNPLUG
        return round(watch.seconds_now(), 2)

    def script():
        settled = r.wait_or_crash(lambda: watch.bench_packets > 20 and not watch.active, HEALTHY_WAIT_S)
        if settled is None:
            rec["skipped"] = (f"the Limelight was faulted before the pull: {watch.message!r}" if watch.active
                              else f"no healthy frames within {HEALTHY_WAIT_S:g} s, or the OpMode crashed")
            return
        r.wait_or_crash(lambda: False, BEFORE_PULL_S)
        if r.crashed():
            rec["skipped"] = "the OpMode crashed before the pull"
            return
        rec["pullEnterS"] = prompt(UNPLUG)
        try:
            seen = r.wait_or_crash(lambda: watch.active, FAULT_SEEN_WAIT_S)
            rec["faultSeenAfterPullEnterS"] = None if seen is None else round(seen, 2)
            r.wait_or_crash(lambda: False, AFTER_PULL_S)
        finally:
            rec["replugEnterS"] = prompt(REPLUG)
        if not r.crashed():
            r.wait_or_crash(lambda: False, AFTER_REPLUG_S)

    r.dash.packet_hook = watch.on_packet
    try:
        o = r.run_opmode(SOAK, "camera_power_midrun", {"mode": "powerloss"}, timeout=60, during=script,
                         result_files=[SOAK_RESULT], quiet_after=10.0)
    finally:
        r.dash.packet_hook = None
    log = r.out / "logs" / "camera_power_midrun.log"
    frames = stack_frames(log.read_text() if log.exists() else "")
    rec.update(crashed=bool(o.get("crashed")), error=(o.get("errorMessage") or "") if o.get("crashed") else "",
               crashAfterStartS=o.get("crashSeenAfterStartSeconds"),
               stackTop=".".join(frames[0][:2]) + f" ({frames[0][2]})" if frames and o.get("crashed") else None,
               dashboard={"transitions": watch.transitions, "benchPackets": watch.bench_packets})
    res = r.pull(SOAK_RESULT, required=False) or {}
    rec["result"] = {k: res.get(k) for k in ("pull", "cameras", "loop", "events", "complete")}


def run(r, record):
    record["bankOut"] = False
    try:
        steps(r, record)
    except BaseException:
        if record["bankOut"]:
            r.log(f"!! {REPLUG}")
        raise


def steps(r, record):
    record["atStart"] = {"webcam": r.webcam, "limelight": r.limelight}
    xml = camera_config(shipped_xml(), r.webcam, r.limelight)
    r.activate_config(CONFIG_CAM, xml)
    if TELE not in {name for name, _ in r.dash.opmodes or []}:
        raise rb.BenchError(f"the hub has no OpMode {TELE!r}: do the full install with the Run button")
    record["control"] = init_tele(r, "camera_power_control")
    if control_check(record)[0]:
        ask(r, UNPLUG)
        record["bankOut"] = True
        record["unplugged"] = {"probe": probe(r, "camera_power_probe_unplugged"),
                               "init": init_tele(r, "camera_power_unplugged")}
        ask(r, REPLUG)
        record["bankOut"] = False
    else:
        record["unplugged"] = {"notRun": "the control INIT didn't hold, so an INIT with the cameras unpowered couldn't "
                                         "be told apart from it"}
    if not wait_cameras(r, record):
        raise rb.BenchError(f"the webcam {r.webcam} and the Limelight at {r.limelight['ipAddress']} weren't both back "
                            f"{BACK_WAIT_S:.0f} s after the power bank was plugged in: check the bank and the powered USB "
                            "hub, then rerun")
    r.activate_config(rb.CONFIG_MAIN, r.main_config())
    midrun(r, record)


def control_check(record):
    c = record.get("control") or {}
    if c.get("failed"):
        return False, f"the runner lost it: {c['failed']}"
    if c.get("crashed"):
        return False, (f"INIT stopped: {c.get('error', '')[:240]!r}"
                       + (f" at {c['stackTop']}" if c.get("stackTop") else ""))
    if c.get("initSeenS") is None or c["initSeenS"] > HANG_S or c.get("appRestarted"):
        return False, f"INIT seen in {c.get('initSeenS')} s; app restarted {c.get('appRestarted')}"
    return True, f"held INIT {HOLD_S:g} s with no error (seen in {c.get('initSeenS')} s, pid {c.get('pidAfter')})"


def unplugged_check(record):
    u = record.get("unplugged") or {}
    if u.get("notRun"):
        return False, f"not run: {u['notRun']}"
    left = {k: v for k, v in (u.get("probe") or {}).items() if v}
    init = u.get("init") or {}
    if left:
        return False, f"the probe still found {left} with the power bank out"
    if init.get("failed"):
        return True, f"INIT never showed, unlike the control: {init['failed']}"
    if not init.get("crashed"):
        return False, f"INIT held {HOLD_S:g} s with both cameras unpowered, as in the control"
    return True, (f"INIT stopped: {init.get('error', '')[:240]!r}"
                  + (f" at {init['stackTop']}" if init.get("stackTop") else "")
                  + (f"; constructor {init['constructor']}" if init.get("constructor") else "; no constructor in the "
                                                                                             "stack")
                  + ("; names a camera or USB device" if init.get("namesCamera") else ""))


def raise_latency_ms(record):
    pull = ((record.get("midrun") or {}).get("result") or {}).get("pull") or {}
    if pull.get("raiseMs") is None or pull.get("lastPollMs") is None:
        return None
    return pull["raiseMs"] - pull["lastPollMs"]


def fault_check(record):
    m = record.get("midrun") or {}
    if m.get("skipped"):
        return False, f"not run: {m['skipped']}"
    pull = (m.get("result") or {}).get("pull") or {}
    latency = raise_latency_ms(record)
    if not pull:
        return False, ("no Limelight fault after the pull" + (f"; the OpMode crashed first: {m.get('error')!r}"
                                                               if m.get("crashed") else ""))
    if latency is None:
        return False, f"the Limelight fault came at {pull.get('raiseMs')} ms with no poll before it to time it from"
    return latency <= FAULT_LIMIT_MS, (f"raised {latency:.0f} ms after the Limelight's last good poll "
                                       f"({pull['raiseMs']:.0f} ms after START); the runner saw it on the dashboard "
                                       f"{m.get('faultSeenAfterPullEnterS')} s after your Enter")


def crash_check(record):
    m = record.get("midrun") or {}
    if m.get("crashed"):
        return False, (f"crashed {m.get('crashAfterStartS')} s after START: {m.get('error', '')[:240]!r}"
                       + (f" at {m['stackTop']}" if m.get("stackTop") else ""))
    if m.get("skipped"):
        return False, f"not run: {m['skipped']}"
    if "result" not in m:
        return False, f"not run: {SOAK} didn't run"
    return True, "no crash through the pull and the replug"


def checks(record):
    return [
        ("camera power: control: BioBuzz Tele held INIT past every lookup with the camera hub powered",
         *control_check(record), CONTROL_MEANS),
        ("camera power: with the camera hub's power bank out, BioBuzz Tele's INIT ends otherwise than the control's",
         *unplugged_check(record), UNPLUGGED_MEANS),
        (f"camera power: the Limelight fault came within {FAULT_LIMIT_MS / 1000:g} s of the mid-run pull",
         *fault_check(record), FAULT_MEANS),
        ("camera power: no crash when the camera hub lost power mid-run", *crash_check(record), CRASH_MEANS),
    ]


def fmt_ms(v):
    return "n/a" if v is None else f"{v:.0f} ms"


def summary(record):
    s = ["## Bench: Camera Power Loss (the camera hub's power bank pulled before INIT and mid-run)\n"]
    start = record.get("atStart") or {}
    s.append(f"- At the start: webcam {start.get('webcam')}, Limelight {(start.get('limelight') or {}).get('ipAddress')}")
    for label, key in (("control (bank in)", "control"), ("bank out", None)):
        init = record.get(key) if key else (record.get("unplugged") or {}).get("init")
        if init is None:
            s.append(f"- INIT, {label}: not run ({(record.get('unplugged') or {}).get('notRun')})")
            continue
        s.append(f"- INIT, {label}: " + ("lost: " + init["failed"] if init.get("failed") else
                                        f"stopped: {init.get('error', '')[:200]!r}" if init.get("crashed") else
                                        f"held {HOLD_S:g} s")
                 + (f"; first team-code frame {init['stackTop']}" if init.get("stackTop") else "")
                 + (f"; constructor {init['constructor']}" if init.get("constructor") else ""))
    u = record.get("unplugged") or {}
    if u.get("probe") is not None:
        s.append(f"- Bench: Probe with the bank out found: {u['probe']}")
    back = record.get("back") or {}
    s.append(f"- Both cameras back after the replug: {back.get('both')} after {back.get('seconds')} s "
             f"({len(back.get('probes') or [])} probes)")
    m = record.get("midrun") or {}
    res = m.get("result") or {}
    pull, cams, loop = res.get("pull") or {}, res.get("cameras") or {}, res.get("loop") or {}
    s.append(f"- Mid-run: {('skipped: ' + m['skipped']) if m.get('skipped') else 'pulled'}; crashed {m.get('crashed')}"
             + (f" ({m.get('error', '')[:160]!r} at {m.get('stackTop')})" if m.get("crashed") else ""))
    if pull:
        last_frame = pull.get("lastWebcamFrameMs")
        last_poll = pull.get("lastPollMs")
        s.append(f"- At the pull: Limelight fault at {fmt_ms(pull.get('raiseMs'))} after START, "
                 f"{fmt_ms(raise_latency_ms(record))} after its last good poll; last webcam frame "
                 + (f"{last_frame - last_poll:+.0f} ms from that poll" if last_frame is not None and last_poll is not None
                    else "n/a")
                 + f"; cell tip scorable {pull.get('scorable')}, tipped {pull.get('tipped')}, staleness "
                   f"{fmt_ms(pull.get('stalenessMs'))}")
    before, after = loop.get("beforePullMs") or {}, loop.get("afterPullMs") or {}
    s.append(f"- Loop period p95 before the pull {fmt_ms(before.get('p95'))} (n={before.get('count')}), after it "
             f"{fmt_ms(after.get('p95'))} (n={after.get('count')})")
    s.append(f"- Back without a re-INIT: webcam {'yes, at ' + fmt_ms(cams.get('webcamBackMs')) if cams.get('webcamBackMs') is not None else 'no'}; "
             f"Limelight {'yes, at ' + fmt_ms(cams.get('limelightBackMs')) if cams.get('limelightBackMs') is not None else 'no'} "
             f"(after START, the run lasting {cams.get('ranSeconds')} s)")
    s.append("")
    return s


def needs(r):
    if not r.args.interactive:
        return ("Bench: Camera Power Loss asks you to pull and replug the camera hub's power bank, so it needs "
                "--interactive and a terminal")
    if r.args.expansion_address != EXPANSION_ADDRESS:
        return f"biobuzz.xml names its Expansion Hub at address {EXPANSION_ADDRESS}: run it without --expansion-address"
    if r.expansion is None:
        return (f"no Expansion Hub answered on RS485 at address {EXPANSION_ADDRESS}: connect one to the Control Hub's "
                "RS485 port, powered from the Control Hub's second XT30 port")
    if r.webcam is None:
        return ("no webcam answered: plug the webcam into the powered USB hub, with its power bank in, on the Control "
                "Hub's USB 3.0 port")
    if r.limelight is None:
        return "no Limelight answered: plug the Limelight 3A into the powered USB hub and wait for it to boot"
    return None


BENCH = rb.Bench(
    name="camera_power",
    title="Bench: Camera Power Loss",
    flag="--camera-power",
    help="Bench: Camera Power Loss: the camera hub's power bank pulled before an INIT of BioBuzz Tele and mid-run "
         "(about 10 min; the webcam and the Limelight on a powered USB hub, an Expansion Hub on RS485 at address 2; "
         "needs --interactive, --s3-ports and a terminal)",
    run=run,
    checks=checks,
    summary=summary,
    needs=needs,
    person=rb.Person(
        "Bench: Camera Power Loss: you pull and replug the camera hub's power bank three times",
        "the webcam and the Limelight 3A on the powered USB hub, its power bank plugged in, on the Control Hub's USB "
        "3.0 port; an Expansion Hub on RS485 at address 2, powered from the Control Hub's second XT30 port; nothing on "
        "either hub's motor ports 0-3 or servo ports 0-5; at the start you type that the ports are empty; about 10 "
        "min",
        flags=["--camera-power", "--interactive", "--s3-ports"]),
    drives=rb.S3_PORTS,
    powers="whatever BioBuzz Tele writes to biobuzz.xml's motors during two INITs (each held 6 s, never STARTed); "
           "Bench: Full Load Soak sets no power (its follower's drive motors on Control Hub motor ports 0-3 stay at 0)",
    configs=(CONFIG_CAM,),
    opmodes=(SOAK,),
)

"""Bench: Config Check: each robot OpMode INITed on its own shipped config, held 4 s and STOPped, never STARTed, so
every hardware lookup it makes at INIT runs against the config it ships for (scripts/bench/README.md)."""
import re
from xml.etree import ElementTree

PAIRS = (("BioBuzz Tele", "biobuzz"), ("Turret Aim Test", "biobuzz"), ("SOTM Test", "biobuzz"),
         ("BioBuzz Shooter Test", "biobuzz"), ("Intake Transfer Shooter Test", "intake_transfer_shooter"),
         ("Cell Tip Test", "camera"), ("Decode Tele", "cuttle_decode"))
CONFIGS = tuple(dict.fromkeys(config for _, config in PAIRS))
EXPANSION_ADDRESS = 2
HOLD_S = 4.0
HANG_S = 10.0
PROBE_RUN = "config_check"
PROBE_RESULT = f"photon_stock_{PROBE_RUN}"
PROBE_HUBS = 2
PROBE_MOTORS_PER_HUB = 4
MISSING_DEVICE = re.compile(r'Unable to find a hardware device with (?:the )?name "([^"]*)"(?: and type (\w+))?')
NULL_POINTER = "NullPointerException"
WEBCAM_OPEN_FAILED = re.compile(r"(\S+) failed to open")
# The Cuttle bot's config: at this station its I2C devices other than the IMU are on other ports or not plugged in.
ELSEWHERE_HERE = {"Decode Tele": "cuttle_decode"}
JAVA_ROOT = "TeamCode/src/main/java"
TEAMCODE = "org/firstinspires/ftc/teamcode"
LOOKUP_LITERALS = (re.compile(r'\.get\(\s*[\w.]+\.class\s*,\s*"([^"]+)"\s*\)'),
                   re.compile(r'new\s+Enhanced(?:Motor|Servo|CRServo)\(\s*\w+\s*,\s*"([^"]+)"'))

LOOKUP_MEANS = ("this OpMode can't INIT on the config it ships for: its code asks for a device the config doesn't "
                "name (or names with another type), or INIT stopped on an error this station doesn't explain before "
                "every lookup ran; fix the config (res/xml) or the code's device name. 'Not judged': INIT stopped at a "
                "device that is elsewhere at this station, and the code throws there instead of reporting a fault and "
                "going on, so the lookups after it are unchecked (step 1's source list covers some)")
CLEAN_MEANS = ("INIT on its own config threw a NullPointerException, took over 10 s to show or to stop, or restarted "
               "the Robot Controller app: a driver would see the app hang or restart at INIT")
PROBE_MEANS = ("a motor port on the Control Hub or the Expansion Hub drew current, or the probe didn't cover both hubs' "
               "motor ports 0-3, so no robot OpMode was INITed: unplug what is there and run it again")


def slug(opmode):
    return re.sub(r"[^a-z0-9]+", "_", opmode.lower()).strip("_")


def config_xml(name):
    return (rb.REPO_ROOT / rb.SHIPPED_CONFIG_DIR / f"{name}.xml").read_text()


def config_names(xml_text):
    return {e.get("name") for e in ElementTree.fromstring(xml_text).iter() if e.get("name")}


def config_webcams(xml_text):
    return {e.get("name"): e.get("serialNumber") for e in ElementTree.fromstring(xml_text).iter("Webcam")}


def elsewhere_tokens(xml_text):
    """Words an error about one of this config's I2C devices (the IMU aside) would carry: its name and its type."""
    tokens = set()
    for e in ElementTree.fromstring(xml_text).iter():
        if e.get("bus") is None or not e.get("name") or "imu" in e.tag.lower():
            continue
        tokens.add(e.get("name").lower())
        kind = re.sub(r"^(gobilda|rev)|v\d+$", "", e.tag.lower())
        if len(kind) >= 4:
            tokens.add(kind)
    return sorted(tokens)


def missing_device(error):
    m = MISSING_DEVICE.search(error or "")
    return None if m is None else {"name": m.group(1), "type": m.group(2)}


def allowed_fault(pair, webcams_attached, config_cameras, elsewhere):
    """Why this INIT error is about a device that is elsewhere or absent at this station, or None."""
    error = (pair.get("error") or "") if pair.get("crashed") else ""
    if not error or missing_device(error) or NULL_POINTER in error:
        return None
    m = WEBCAM_OPEN_FAILED.search(error)
    if m and m.group(1) in config_cameras and config_cameras[m.group(1)] not in webcams_attached:
        return (f"{m.group(1)} failed to open: the config names webcam {config_cameras[m.group(1)]}, and the attached "
                f"webcams are {sorted(webcams_attached) or 'none'}")
    squashed = re.sub(r"\s+", "", error.lower())
    hit = next((t for t in elsewhere if re.sub(r"\s+", "", t) in squashed), None)
    if hit is not None:
        return f"an error about {hit}, an I2C device of {pair['config']} that isn't on that port at this station"
    return None


def pair_checks(pair, webcams_attached, config_cameras, elsewhere):
    """(lookups ok, detail) and (clean ok, detail) for one OpMode INITed on its own config."""
    if pair.get("notRun"):
        return (False, f"not run: {pair['notRun']}"), (False, f"not run: {pair['notRun']}")
    if pair.get("failed"):
        why = f"the runner lost it: {pair['failed']}"
        return (False, why), (False, why)
    error = (pair.get("error") or "") if pair.get("crashed") else ""
    missing = missing_device(error)
    allowed = allowed_fault(pair, webcams_attached, config_cameras, elsewhere)
    if missing is not None:
        lookups = (False, f"INIT stopped at a missing device: {missing['name']!r}"
                          + (f" as {missing['type']}" if missing["type"] else "") + f" ({error[:240]!r})")
    elif pair.get("crashed") and allowed is not None:
        lookups = (False, f"not judged: INIT stopped at {allowed}, so the lookups after it didn't run and this station "
                          f"can't check them ({error[:200]!r})")
    elif pair.get("crashed"):
        lookups = (False, f"INIT threw {error[:240]!r}, which isn't a fault this station explains, so the lookups "
                          "after it didn't run")
    else:
        lookups = (True, f"held INIT {HOLD_S:g} s with no error: every lookup found its device")
    problems = []
    if NULL_POINTER in error:
        problems.append("a NullPointerException")
    seen, stop = pair.get("initSeenS"), pair.get("stopS")
    if seen is None or seen > HANG_S:
        problems.append(f"INIT or its error took {seen} s to show")
    if stop is not None and stop > HANG_S:
        problems.append(f"STOP took {stop} s")
    if pair.get("appRestarted") or pair.get("pidBefore") != pair.get("pidAfter"):
        problems.append(f"the app restarted (pid {pair.get('pidBefore')} -> {pair.get('pidAfter')})")
    clean = (not problems, "; ".join(problems) if problems else
             f"INIT {'error' if pair.get('crashed') else 'seen'} in {seen} s"
             + (f", stopped in {stop} s" if stop is not None else "") + f", pid {pair.get('pidAfter')} throughout")
    return lookups, clean


def probe_check(probe):
    rows = (probe or {}).get("ports") or []
    hubs = {row.get("hub") for row in rows}
    busy = [(row.get("hub"), row.get("channel"), row.get("drivenAmps")) for row in rows if not row.get("empty")]
    ok = ((probe or {}).get("empty") is True and not busy and len(hubs) == PROBE_HUBS
          and len(rows) == PROBE_HUBS * PROBE_MOTORS_PER_HUB)
    detail = (f"{len(rows)} motor ports on {len(hubs)} hubs at {(probe or {}).get('power')} power, limit "
              f"{(probe or {}).get('limitAmps')} A; drawing current: {busy or 'none'}"
              + (f"; Bench: Photon Stock: {probe['error']!r}" if (probe or {}).get("error") else ""))
    return ok, detail


def checks(record):
    out = [("config check: Bench: Photon Stock's port probe found both hubs' motor ports 0-3 empty",
            *probe_check(record.get("portProbe")), PROBE_MEANS)]
    attached = set(record.get("webcams") or [])
    for pair in record.get("pairs") or []:
        lookups, clean = pair_checks(pair, attached, pair.get("cameras") or {}, pair.get("elsewhere") or [])
        out.append((f"config check: {pair['opmode']} on {pair['config']}: every hardware lookup found its device",
                    *lookups, LOOKUP_MEANS))
        out.append((f"config check: {pair['opmode']} on {pair['config']}: no NullPointerException, no hang over "
                    f"{HANG_S:g} s, same pid", *clean, CLEAN_MEANS))
    return out


def summary(record):
    s = ["## Bench: Config Check (each robot OpMode INITed on its own shipped config, never STARTed)\n"]
    if record.get("stopped"):
        s.append(f"**No robot OpMode was INITed: {record['stopped']}**\n")
    probe = record.get("portProbe") or {}
    s.append(f"- Port probe (Bench: Photon Stock on bench_photon, Expansion Hub on RS485 at address "
             f"{EXPANSION_ADDRESS}): {probe_check(probe)[1]}")
    s.append(f"- Webcams attached: {record.get('webcams') or 'none'}")
    s += ["", "| OpMode | config | INIT | error | missing device | INIT to error | pid before/after |",
          "|---|---|---|---|---|---|---|"]
    for p in record.get("pairs") or []:
        error = (p.get("error") or "") if p.get("crashed") else ""
        missing = missing_device(error)
        state = (f"not run: {p['notRun']}" if p.get("notRun") else f"lost: {p['failed']}" if p.get("failed")
                 else "threw" if p.get("crashed") else f"held {HOLD_S:g} s")
        s.append(f"| {p['opmode']} | {p['config']} | {state} | {error[:160].replace('|', '/')!r} | "
                 f"{missing['name'] if missing else ''} | "
                 f"{'' if p.get('initToErrorMs') is None else round(p['initToErrorMs'])} | "
                 f"{p.get('pidBefore', '')}/{p.get('pidAfter', '')} |")
    s.append("")
    desktop = record.get("desktop") or {}
    s.append("Device names each OpMode's code passes as string literals to `hardwareMap.get` or an `Enhanced*` "
             "constructor, against its config (from the source on this Mac, record only: names held in constants or "
             "arrays aren't seen):\n")
    for p in record.get("pairs") or []:
        d = desktop.get(p["opmode"]) or {}
        s.append(f"- {p['opmode']} ({d.get('file', 'source not found')}): {len(d.get('literals') or [])} names; not "
                 f"in {p['config']}: {d.get('notInConfig') or 'none'}")
    s.append("")
    return s


def java_index(root):
    files = {}
    for path in (root / JAVA_ROOT / TEAMCODE).rglob("*.java"):
        files[path.relative_to(root / JAVA_ROOT).with_suffix("").as_posix().replace("/", ".")] = path
    return files


def references(path, text, files):
    """The teamcode classes a Java file names: its explicit and wildcard imports, and its own package's classes."""
    package = re.search(r"^package\s+([\w.]+);", text, re.M)
    package = package.group(1) if package else ""
    imports = re.findall(r"^import\s+([\w.]+)\s*;", text, re.M)
    wildcards = re.findall(r"^import\s+([\w.]+)\.\*\s*;", text, re.M)
    words = set(re.findall(r"\b[A-Z]\w*\b", text))
    found = {files[i] for i in imports if i in files and i.rsplit(".", 1)[-1] in words}
    for scope in [package] + wildcards:
        found |= {files[f"{scope}.{w}"] for w in words if f"{scope}.{w}" in files}
    return found - {path}


def code_lookups(root, opmode):
    """The device-name literals reachable from an OpMode's source, or None when no file declares it."""
    files = java_index(root)
    start = next((p for p in files.values() if f'name = "{opmode}"' in p.read_text()), None)
    if start is None:
        return None
    seen, todo, names = set(), [start], set()
    while todo:
        path = todo.pop()
        if path in seen or "/opmodes/test/bench/" in path.as_posix():
            continue
        seen.add(path)
        text = path.read_text()
        for pattern in LOOKUP_LITERALS:
            names.update(pattern.findall(text))
        todo += references(path, text, files)
    return {"file": start.relative_to(root).as_posix(), "files": len(seen), "literals": sorted(names)}


def desktop_lookups(root):
    out = {}
    for opmode, config in PAIRS:
        found = code_lookups(root, opmode)
        if found is not None:
            found["notInConfig"] = sorted(set(found["literals"]) - config_names(config_xml(config)))
        out[opmode] = found
    return out


def init_pair(r, pair):
    try:
        o = r.run_opmode(pair["opmode"], f"config_check_{slug(pair['opmode'])}", start=False, timeout=HOLD_S,
                         timeout_ok=True)
    except (rb.ProtocolError, rb.OtherHub):
        raise
    except rb.BenchError as e:
        pair["failed"] = str(e)
        r.settle()
        return
    crash_s = o.get("crashSeenSeconds")
    pair.update(initSeenS=round(o["initSeenSeconds"], 2) if o.get("initSeenSeconds") is not None else None,
                crashed=bool(o.get("crashed")), error=o.get("errorMessage") or "" if o.get("crashed") else "",
                errorSource=o.get("errorSource"), statusError=o.get("statusErrorMessage") or "",
                initToErrorMs=None if not o.get("crashed") or crash_s is None else round(crash_s * 1000),
                stopS=round(o["stopSecondsRunnerSide"], 2) if o.get("stopSecondsRunnerSide") is not None else None,
                pidBefore=o.get("pidBefore"), pidAfter=o.get("pidAfter"), appRestarted=bool(o.get("appRestarted")),
                packets=(o.get("dashboard") or {}).get("packets"))


def port_probe(r, record):
    r.activate_config(rb.CONFIG_PHOTON, rb.hub_config(rb.main_devices(r.imu_type),
                                                       expansion=(EXPANSION_ADDRESS, rb.PHOTON_EXPANSION_DEVICES)))
    o = r.run_opmode("Bench: Photon Stock", "config_check_port_probe",
                     {"runId": PROBE_RUN, "expansion": 2, "i2c": 0, "portProbe": 1, "probeOnly": 1}, timeout=300,
                     result_files=[PROBE_RESULT])
    res = r.pull(PROBE_RESULT, required=False) or {}
    record["portProbe"] = dict(res.get("portProbe") or {}, error=o.get("errorMessage") or "")
    return probe_check(record["portProbe"])[0]


def run(r, record):
    record["pairs"] = [{"opmode": o, "config": c, "cameras": config_webcams(config_xml(c)),
                        "elsewhere": elsewhere_tokens(config_xml(c)) if ELSEWHERE_HERE.get(o) == c else []}
                       for o, c in PAIRS]
    record["webcams"] = sorted(w["serial"] for w in (r.probe or {}).get("webcams", []) if w.get("attached"))
    record["desktop"] = desktop_lookups(rb.REPO_ROOT)
    if not r.switch_with_adb:
        raise rb.BenchError("Bench: Config Check activates each shipped config by name in the app's settings, which "
                            "needs run-as: do the full install with the Run button (a debuggable build)")
    if not port_probe(r, record):
        record["stopped"] = "Bench: Photon Stock's port probe didn't find both hubs' motor ports 0-3 empty"
        for pair in record["pairs"]:
            pair["notRun"] = "the port probe didn't pass"
        return
    shipped = (r.probe or {}).get("shippedConfigs") or {}
    on_hub = {name for name, _ in r.dash.opmodes or []}
    for config in CONFIGS:
        pairs = [p for p in record["pairs"] if p["config"] == config]
        if config not in shipped:
            for p in pairs:
                p["notRun"] = f"this install doesn't ship {config}"
            continue
        try:
            r.activate_config(config, config={"name": config, "resourceId": shipped[config], "location": "RESOURCE",
                                              "isDirty": False})
        except (rb.ProtocolError, rb.OtherHub):
            raise
        except rb.BenchError as e:
            for p in pairs:
                p["notRun"] = f"the robot didn't start on {config}: {e}"
            continue
        for p in pairs:
            if p["opmode"] not in on_hub:
                p["notRun"] = f"the hub has no OpMode {p['opmode']!r}"
                continue
            init_pair(r, p)


def needs(r):
    if r.args.expansion_address != EXPANSION_ADDRESS:
        return (f"every shipped config names its Expansion Hub at address {EXPANSION_ADDRESS}: run it without "
                "--expansion-address")
    if r.expansion is None:
        return (f"no Expansion Hub answered on RS485 at address {EXPANSION_ADDRESS}: connect one to the Control Hub's "
                "RS485 port, powered from the Control Hub's second XT30 port")
    return None


BENCH = rb.Bench(
    name="config_check",
    title="Bench: Config Check",
    flag="--config-check",
    help="Bench: Config Check: each robot OpMode INITed on its own shipped config for 4 s, never STARTed (about 8 "
         "min; an Expansion Hub on RS485 at address 2; needs --s3-ports and a terminal)",
    run=run,
    checks=checks,
    summary=summary,
    needs=needs,
    person=rb.Person(
        "Bench: Config Check: each robot OpMode INITed on its own shipped config, never STARTed",
        "an Expansion Hub on RS485 at address 2, powered from the Control Hub's second XT30 port; nothing on either "
        "hub's motor ports 0-3 or servo ports 0-5; the Pinpoint in Control Hub I2C port 1, the OctoQuad in port 2, the "
        "Prism in port 3, the webcam and the Limelight on the Control Hub's USB; at the start you type that the ports "
        "are empty; about 8 min",
        address=EXPANSION_ADDRESS),
    drives=rb.S3_PORTS,
    powers="Bench: Photon Stock's port probe at 0.1 power for 0.2 s on each motor port 0-3 of both hubs, one at a "
           "time; then whatever each robot OpMode writes to its config's motors and servos during INIT (held 4 s, "
           "never STARTed)",
    configs=(rb.CONFIG_PHOTON,) + CONFIGS,
    opmodes=("Bench: Photon Stock",),
)

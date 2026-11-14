"""Bench: Limelight Reload: LimelightSync's one POST sender across real deploySloth reloads, a STOP in the middle of
the INIT sync, a SnapScript stamp carried by deploySloth, and a fake Limelight holding a POST across a reload
(scripts/bench/README.md)."""
import json
import re
import threading

OPMODE = "Bench: Limelight Reload"
RESULT = "limelight_reload_"
AFTER_STOP = "_afterstop"
GO_FILE = "limelight_reload_go"
SENDER = "LimelightSync"
RELOAD_STOPS_S = (0.3, 0.8, 1.4)
RELOAD_ROUNDS = 3
MID_SYNC_STOPS_S = (0.3, 0.8, 1.3)
SLOTH_STOP_S = 0.8
CONTENTION_ORDERS = ("after_stop", "staged")
HOLD_S = 5.0
STAMP_HOLD_S = 1.0
HUNG_MS = 300
FIRST_FRAME_MS = 5000.0
NEXT_SYNC_MS = 5000.0
LOOP_MAX_MS = 50.0
AFTER_STOP_MS = 3000.0
SYNC_WAIT_S = 20.0
REAL_TIMEOUT_S = 30.0
ANSWER_TIMEOUT_S = 45.0
HANG_TIMEOUT_S = 25.0
STATUS_AFTER_STOP_S = 2.0
ADB_AFTER_STOP_S = 3.0
AFTER_STOP_PULL_S = 4.0
BUILD_TIMEOUT_S = 900
GENERATED = "TeamCode/build/generated"
STAMP_LINE = re.compile(r"static final double BALL_SCRIPT_STAMP = (\d+(?:\.\d+)?);")

STOPPED_MEANS = ("the STOP didn't land inside the INIT sync (it came before the sync started or after it ended), so "
                 "these trials didn't test a STOP in the middle of one")
RELOADED_MEANS = ("the deploySloth didn't land before the next INIT (no 'Processed Sloth Load' in logcat, or the next "
                  "run had the same class loader), so it ran the old code and tested no reload")
SYNC_MEANS = ("after a STOP in the middle of the INIT sync and a deploySloth, the reloaded code's sync failed or its "
              f"first ball frame took over {FIRST_FRAME_MS / 1000:.0f} s: ball vision isn't ready after a hot reload")
RESTART_MEANS = ("the Robot Controller app restarted during a STOP mid-sync or the deploySloth after it, losing "
                 "dashboard edits and the alliance")
LEFTOVER_MEANS = ("a LimelightSync thread was still alive 5 s after a sync ended: every reload adds one, and an old "
                  "one can switch the Limelight's pipeline later")
LOOP_MEANS = (f"the INIT loop stalled {LOOP_MAX_MS:.0f} ms or more while LimelightSync synced on reloaded code: the "
              "sender blocks the OpMode thread")
NEXT_SYNC_MEANS = (f"after a STOP in the middle of the INIT sync, the next INIT's sync failed, took over "
                   f"{NEXT_SYNC_MS / 1000:.0f} s, or left the Limelight on another pipeline or another script")
SWITCH_MEANS = ("the Limelight left the ball pipeline after the next sync ended: a worker from the stopped run switched "
                "it ('on pipeline 0, want 4')")
AFTER_STOP_MEANS = ("a LimelightSync thread from the stopped run was alive 3 s after STOP and not inside a POST: an "
                    "orphan worker that can still switch the Limelight's pipeline")
STAMP_MEANS = ("deploySloth didn't carry the SnapScript change to the Limelight: after a hot reload the Limelight runs "
               "an older script, or reports a stamp that isn't the new build's")
RESTORE_MEANS = ("the bench's last deploySloth didn't put the build's own stamp back: the hub runs a salted script until "
                 "the next deploySloth or full install")
CONTENTION_MEANS = ("at the fake Limelight, the reloaded code's first POST went out while the old code's POST was still "
                    "open: two senders got past the blackboard semaphore, so an old OpMode's POST can land after the new "
                    "one's and leave the Limelight on the wrong pipeline or script")
EXERCISED_MEANS = ("semaphore not exercised: in no trial was a reloaded LimelightSync worker seen waiting on the "
                   "blackboard semaphore while the old code's POST was open (the deploySloth and its load took longer "
                   "than the 15 s POST timeout), so the single-sender check above proves nothing")


class Stuck(Exception):
    """deploySloth hung or failed: every later deploySloth would too, so the bench stops deploying."""


def outcome(o):
    keys = ("initSeenSeconds", "crashed", "errorMessage", "pidBefore", "pidAfter", "appRestarted",
            "restartedDuringStop", "stopSecondsRunnerSide", "timedOut", "doneLine")
    out = {k: o.get(k) for k in keys}
    out["logMarkers"] = (o.get("logMarkers") or [])[:5]
    return out


def adb_senders(r):
    """LimelightSync threads in the app as adb lists them (comm names), or senders None when no listing works."""
    pid = r.adb.pid()
    if pid is None:
        return {"pid": None, "method": None, "senders": None, "tried": ["no Robot Controller pid"]}
    method, names, tried = r.adb.thread_names(pid, r.thread_method)
    return {"pid": pid, "method": method, "threads": len(names),
            "senders": sum(1 for n in names if n == SENDER) if method else None, "tried": tried}


def limelight_status(r):
    reply = r.link.http("GET", rb.LIMELIGHT_PORT, "/status")
    try:
        index = json.loads(reply.get("body") or "").get("pipelineIndex")
    except (ValueError, AttributeError):
        index = None
    return {"code": reply.get("code"), "error": reply.get("error"), "ms": reply.get("ms"), "pipelineIndex": index}


def built_stamp(root):
    """BALL_SCRIPT_STAMP in the LimelightFiles.java the last Gradle build generated, or None."""
    folder = root / GENERATED
    files = sorted(folder.rglob("LimelightFiles.java"), key=lambda p: p.stat().st_mtime) if folder.is_dir() else []
    m = STAMP_LINE.search(files[-1].read_text()) if files else None
    return float(m.group(1)) if m else None


def push(r, run_id, gradle_args=()):
    hub_t0 = r.adb.hub_time()
    try:
        d = r._deploy_sloth(run_id, None, gradle_args=tuple(gradle_args))
    except (rb.ProtocolError, rb.OtherHub):
        raise
    except rb.BenchError as e:
        raise Stuck(str(e)) from None
    if d["hung"]:
        raise Stuck(f"deploySloth hung waiting for the hub's Sloth lock ({run_id}): restart the robot before the next "
                    f"deploySloth (logs/sloth_deploy_{run_id}.log)")
    return {"runId": run_id, "hubT0": hub_t0, "deploy": d, "gradleArgs": list(gradle_args)}


def landed(r, pushed):
    d = pushed["deploy"]
    load = r.sloth_load(pushed["hubT0"], pushed["runId"], d)
    return {"seconds": round(d["seconds"], 1), "returncode": d["returncode"], "gradleArgs": pushed["gradleArgs"],
            "load": load["result"], "stagedS": load.get("stagedS"), "endS": load.get("endS")}


def build_jar(r):
    """assembleSloth up front, so the deploySloths inside the 15 s POST timeout only push."""
    t0 = rb.time.monotonic()
    proc = rb.subprocess.Popen(r.gradle_command(":TeamCode:assembleSloth"), cwd=rb.REPO_ROOT,
                               stdout=rb.subprocess.PIPE, stderr=rb.subprocess.STDOUT, text=True)
    watchdog = threading.Timer(BUILD_TIMEOUT_S, proc.kill)
    watchdog.start()
    try:
        lines = list(proc.stdout)
        proc.wait()
    finally:
        watchdog.cancel()
        if proc.poll() is None:
            proc.kill()
            proc.wait()
    (r.out / "logs" / "limelight_reload_build.log").write_text("".join(lines))
    if proc.returncode != 0:
        raise Stuck(f"assembleSloth failed (logs/limelight_reload_build.log): {''.join(lines).strip()[-400:]}")
    return {"seconds": round(rb.time.monotonic() - t0, 1)}


def phase_is(r, *phases):
    return r.dash.data.get(rb.KEY_PHASE) in phases


def stop_into_sync(r, runner, delay_s):
    """during_init: STOP delay_s after the bench says its sync started (run_opmode STOPs when this returns)."""
    def during():
        seen = r.wait_or_crash(lambda: phase_is(r, "sync", "synced"), SYNC_WAIT_S)
        runner["syncSeenS"] = None if seen is None else round(seen, 2)
        if seen is not None and phase_is(r, "sync"):
            r.wait_or_crash(lambda: False, delay_s)
        runner["stopAt"] = rb.time.monotonic()
    return during


def first_run(r, run_id, during, params=None, runner=None):
    """A run the runner STOPs as soon as during() returns; its result and runner-side record."""
    result = RESULT + run_id
    o = r.run_opmode(OPMODE, f"limelight_reload_{run_id}", dict({"runId": run_id, "mode": "real"}, **(params or {})),
                     start=False, timeout=0, timeout_ok=True, during_init=during,
                     result_files=[result, result + AFTER_STOP, GO_FILE])
    return {"o": outcome(o), "res": r.pull(result, required=False) or {}, "runner": runner or {}}


def second_run(r, run_id, mode="real", hold_s=HOLD_S, timeout_s=REAL_TIMEOUT_S, stop_at=None, limelight=True):
    """A run held in INIT until its sync ended hold_s ago, then looked at from adb (and the Limelight's status)."""
    runner = {}
    result = RESULT + run_id

    def during():
        if stop_at is not None and limelight:
            r.wait_or_crash(lambda: rb.time.monotonic() >= stop_at + STATUS_AFTER_STOP_S, STATUS_AFTER_STOP_S + 30)
            runner["statusAfterStop"] = dict(limelight_status(r), afterStopS=round(rb.time.monotonic() - stop_at, 2))
        seen = r.wait_or_crash(lambda: r.dash.done_line is not None, timeout_s + 15)
        runner["doneSeenS"] = None if seen is None else round(seen, 2)
        if seen is not None:
            runner["adbAfterHold"] = adb_senders(r)
            if limelight:
                runner["statusAfterSync"] = limelight_status(r)

    o = r.run_opmode(OPMODE, f"limelight_reload_{run_id}",
                     {"runId": run_id, "mode": mode, "holdAfterSyncS": hold_s, "timeoutS": timeout_s},
                     start=False, timeout=5, during_init=during, result_files=[result, result + AFTER_STOP])
    return {"o": outcome(o), "res": r.pull(result, required=False) or {}, "runner": runner}


def after_stop(r, run_id, stop_at):
    """The stopped run's watcher file, written about 3 s after its STOP."""
    rb.time.sleep(max(0.0, stop_at + AFTER_STOP_PULL_S - rb.time.monotonic()))
    found = r.pull(RESULT + run_id + AFTER_STOP, required=False)
    if found is None or not found.get("done"):
        rb.time.sleep(2.0)
        found = r.pull(RESULT + run_id + AFTER_STOP, required=False)
    return found


def guarded(r, item, step, key="failed"):
    try:
        step()
    except (rb.ProtocolError, rb.OtherHub):
        raise
    except rb.BenchError as e:
        item[key] = str(e)
        r.settle()


def reload_trial(r, trial):
    a, b = f"reload_{trial['id']}_a", f"reload_{trial['id']}_b"
    runner = {}
    trial["first"] = first_run(r, a, stop_into_sync(r, runner, trial["stopS"]), runner=runner)
    stop_at = runner.get("stopAt", rb.time.monotonic())
    rb.time.sleep(max(0.0, stop_at + ADB_AFTER_STOP_S - rb.time.monotonic()))
    trial["adbAfterStop"] = dict(adb_senders(r), afterStopS=round(rb.time.monotonic() - stop_at, 2))
    trial["first"]["afterStop"] = after_stop(r, a, stop_at)
    trial["deploy"] = landed(r, push(r, f"limelight_reload_{trial['id']}"))
    trial["second"] = second_run(r, b)


def reload_phase(r, record):
    for n in range(1, RELOAD_ROUNDS + 1):
        for stop_s in RELOAD_STOPS_S:
            trial = {"id": f"{round(stop_s * 1000)}ms_{n}", "stopS": stop_s}
            record["reload"].append(trial)
            guarded(r, trial, lambda: reload_trial(r, trial))


def at_once_case(r, case):
    a, b = f"midsync_{case['id']}_a", f"midsync_{case['id']}_b"
    runner = {}
    case["first"] = first_run(r, a, stop_into_sync(r, runner, case["stopS"]), runner=runner)
    stop_at = runner.get("stopAt", rb.time.monotonic())
    case["next"] = second_run(r, b, stop_at=stop_at)
    case["first"]["afterStop"] = after_stop(r, a, stop_at)


def sloth_case(r, case):
    """INIT waiting for the go file, deploySloth (staged while the OpMode is in INIT), go, STOP 0.8 s into the sync;
    the load lands after the STOP, then the next INIT."""
    a, b = f"midsync_{case['id']}_a", f"midsync_{case['id']}_b"
    runner, pushed = {}, {}

    def during():
        runner["waitingSeenS"] = r.wait_or_crash(lambda: phase_is(r, "waiting"), SYNC_WAIT_S)
        pushed.update(push(r, f"limelight_reload_{case['id']}"))
        r.adb.push_json({"go": a}, f"{GO_FILE}.json", r.out / "logs" / f"{GO_FILE}.json")
        stop_into_sync(r, runner, case["stopS"])()

    case["first"] = first_run(r, a, during, params={"waitForGo": 1}, runner=runner)
    stop_at = runner.get("stopAt", rb.time.monotonic())
    case["deploy"] = landed(r, pushed) if pushed else None
    r.wait_or_crash(lambda: rb.time.monotonic() >= stop_at + STATUS_AFTER_STOP_S, STATUS_AFTER_STOP_S + 30)
    case["statusAfterStop"] = dict(limelight_status(r), afterStopS=round(rb.time.monotonic() - stop_at, 2))
    case["next"] = second_run(r, b)
    case["first"]["afterStop"] = after_stop(r, a, stop_at)


def mid_sync_phase(r, record):
    for stop_s in MID_SYNC_STOPS_S:
        case = {"id": f"at_once_{round(stop_s * 1000)}ms", "stopS": stop_s, "sloth": False}
        record["midSync"].append(case)
        guarded(r, case, lambda: at_once_case(r, case))
    case = {"id": f"sloth_{round(SLOTH_STOP_S * 1000)}ms", "stopS": SLOTH_STOP_S, "sloth": True}
    record["midSync"].append(case)
    guarded(r, case, lambda: sloth_case(r, case))


def stamp_run(r, run_id):
    return second_run(r, run_id, hold_s=STAMP_HOLD_S)


def stamp_phase(r, record):
    rec = record["stamp"]
    rec["fileBefore"] = built_stamp(rb.REPO_ROOT)
    guarded(r, rec, lambda: rec.update(before=stamp_run(r, "stamp_before")), "beforeFailed")
    rec["salt"] = f"limelight-reload-{r.out.name}"
    rec["saltedDeploy"] = landed(r, push(r, "limelight_reload_salted", (f"-Pbench.scriptSalt={rec['salt']}",)))
    rec["fileSalted"] = built_stamp(rb.REPO_ROOT)
    guarded(r, rec, lambda: rec.update(salted=stamp_run(r, "stamp_salted")), "saltedFailed")
    rec["restoreDeploy"] = landed(r, push(r, "limelight_reload_restore"))
    rec["fileRestored"] = built_stamp(rb.REPO_ROOT)
    guarded(r, rec, lambda: rec.update(restored=stamp_run(r, "stamp_restored")), "restoredFailed")


def contention_trial(r, trial):
    hang, answer = f"contention_{trial['id']}_hang", f"contention_{trial['id']}_answer"
    runner, pushed = {}, {}

    def during():
        seen = r.wait_or_crash(lambda: r.dash.done_line is not None, HANG_TIMEOUT_S + 10)
        runner["hungSeenS"] = None if seen is None else round(seen, 2)
        if trial["id"] == "staged" and seen is not None:
            pushed.update(push(r, f"limelight_reload_contention_{trial['id']}"))
        runner["stopAt"] = rb.time.monotonic()

    trial["hang"] = first_run(r, hang, during, params={"mode": "hang", "hungMs": HUNG_MS, "timeoutS": HANG_TIMEOUT_S},
                              runner=runner)
    if not pushed:
        pushed.update(push(r, f"limelight_reload_contention_{trial['id']}"))
    trial["deploy"] = landed(r, pushed)
    trial["answer"] = second_run(r, answer, mode="answer", timeout_s=ANSWER_TIMEOUT_S, limelight=False)


def contention_phase(r, record):
    r.activate_config(rb.CONFIG_LLFAULT, rb.hub_config(rb.main_devices(r.imu_type),
                                                       [rb.limelight_tag(rb.UNREACHABLE_IP)]))
    record["build"] = build_jar(r)
    for order in CONTENTION_ORDERS:
        trial = {"id": order}
        record["contention"].append(trial)
        guarded(r, trial, lambda: contention_trial(r, trial))


PHASES = (("reload", reload_phase), ("midSync", mid_sync_phase), ("stamp", stamp_phase),
          ("contention", contention_phase))


def run(r, record):
    record.update(reload=[], midSync=[], stamp={}, contention=[], notRun={})
    for i, (name, phase) in enumerate(PHASES):
        try:
            phase(r, record)
        except Stuck as e:
            record["stopped"] = f"{name}: {e}"
            for later, _ in PHASES[i + 1:]:
                record["notRun"][later] = f"the bench stopped deploying in {name}: {e}"
            return


def needs(r):
    if r.limelight is None:
        return ("no Limelight answers on an Ethernet-over-USB interface: plug the Limelight 3A into a Control Hub USB "
                "port and wait for it to boot")
    if r.args.no_sloth:
        return "it runs deploySloth about 14 times, which --no-sloth turns off: drop --no-sloth"
    if not (rb.REPO_ROOT / "gradlew").exists():
        return f"{rb.REPO_ROOT / 'gradlew'} is missing: run it from a clone of the repo"
    return None


def res(run_rec, *path, default=None):
    return rb.dig(run_rec or {}, "res", *path, default=default)


def stopped_in_sync(first):
    stop = res(first, "stop") or {}
    return stop.get("syncing") is True and stop.get("msIntoSync") is not None


def loader_changed(before, after):
    a, b = res(before, "code", "classLoaderId"), res(after, "code", "classLoaderId")
    return a is not None and b is not None and a != b


def pid_kept(first, second):
    o1, o2 = (first or {}).get("o") or {}, (second or {}).get("o") or {}
    flags = [o.get(k) for o in (o1, o2) for k in ("appRestarted", "restartedDuringStop")]
    pids = [o1.get("pidBefore"), o1.get("pidAfter"), o2.get("pidBefore"), o2.get("pidAfter")]
    return None not in pids and len(set(pids)) == 1 and not any(flags)


def lost(item, *parts):
    if item.get("failed"):
        return f"lost: {item['failed']}"
    missing = [p for p in parts if not (item.get(p) or {}).get("res")]
    return f"no result from {', '.join(missing)}" if missing else None


def aggregate(name, items, verdict, means, not_run):
    if not items:
        return name, False, f"not run: {not_run or 'no trial recorded'}", means
    out = []
    bad = 0
    for item in items:
        ok, detail = verdict(item)
        bad += not ok
        out.append(f"{item['id']} {'ok' if ok else 'FAIL'}: {detail}")
    return name, bad == 0, f"{len(items) - bad} of {len(items)}: " + "; ".join(out), means


def stop_verdict(item):
    why = lost(item, "first")
    if why:
        return False, why
    stop = res(item["first"], "stop") or {}
    into = stop.get("msIntoSync")
    if into is None:
        return False, f"not during the sync: sync started {stop.get('syncStarted')}, syncing {stop.get('syncing')}"
    return stopped_in_sync(item["first"]), f"STOP {into:.0f} ms into the sync (target {item['stopS'] * 1000:.0f})"


def reloaded_verdict(key):
    def verdict(item):
        why = lost(item, "first", key)
        if why:
            return False, why
        load = (item.get("deploy") or {}).get("load")
        changed = loader_changed(item["first"], item[key])
        return load == "processed" and changed, f"load {load}, class loader changed {changed}"
    return verdict


def sync_verdict(item):
    why = lost(item, "second")
    if why:
        return False, why
    sync, first = res(item["second"], "sync") or {}, res(item["second"], "firstFrame") or {}
    ok = "doneMs" in sync and sync.get("problem") is None and first.get("ms") is not None and first["ms"] <= FIRST_FRAME_MS
    return ok, f"sync {sync.get('ms', 'not done')} ms, problem {sync.get('problem')!r}, first frame {first.get('ms')} ms"


def restart_verdict(item):
    why = lost(item, "first", "second")
    if why:
        return False, why
    o1, o2 = item["first"]["o"], item["second"]["o"]
    return pid_kept(item["first"], item["second"]), f"pid {o1.get('pidBefore')} -> {o2.get('pidAfter')}"


def leftover_verdict(item):
    why = lost(item, "second")
    if why:
        return False, why
    hold = res(item["second"], "hold") or {}
    adb = rb.dig(item["second"], "runner", "adbAfterHold", default={}) or {}
    inside = hold.get("senderThreads")
    seen = adb.get("senders")
    ok = inside == [] and seen in (0, None) and hold.get("afterSyncMs") is not None
    return ok, (f"{'no hold record' if inside is None else len(inside)} in the app {hold.get('afterSyncMs')} ms after "
                f"the sync, {seen if seen is not None else 'no adb listing'} from adb")


def loop_verdict(item):
    why = lost(item, "second")
    if why:
        return False, why
    worst = res(item["second"], "initLoopMs", "max")
    return worst is not None and worst < LOOP_MAX_MS, f"INIT loop max {worst} ms"


def next_sync_verdict(item):
    why = lost(item, "next")
    if why:
        return False, why
    sync, first = res(item["next"], "sync") or {}, res(item["next"], "firstFrame") or {}
    stamp = res(item["next"], "code", "stamp")
    ok = ("doneMs" in sync and sync["doneMs"] <= NEXT_SYNC_MS and sync.get("problem") is None
          and first.get("pipeline") == rb.BALL_PIPELINE and stamp is not None and first.get("stamp") == stamp)
    return ok, (f"sync done {sync.get('doneMs')} ms after INIT, problem {sync.get('problem')!r}, first frame on "
                f"pipeline {first.get('pipeline')} with stamp {first.get('stamp')} (build's {stamp})")


def switch_verdict(item):
    why = lost(item, "next")
    if why:
        return False, why
    changes = res(item["next"], "pipelineAfterSync") or []
    problems = [p.get("problem") for p in res(item["next"], "problemsAfterSync") or []]
    status = rb.dig(item["next"], "runner", "statusAfterSync", "pipelineIndex")
    ok = (bool(changes) and all(c[1] == rb.BALL_PIPELINE for c in changes)
          and not any(str(p).startswith("on pipeline") for p in problems if p) and status == rb.BALL_PIPELINE)
    return ok, f"pipelines after the sync {[c[1] for c in changes]}, problems {problems}, /status after it {status}"


def after_stop_verdict(item):
    why = lost(item, "first")
    if why:
        return False, why
    watch = (item["first"] or {}).get("afterStop") or {}
    late = [s for s in watch.get("samples") or [] if (s.get("afterStopMs") or 0) >= AFTER_STOP_MS]
    if not watch.get("done") or not late:
        return False, f"no record 3 s after STOP (watcher file {'incomplete' if watch else 'missing'})"
    orphans = [t for t in late[0].get("fromStop") or [] if not t.get("inPost")]
    in_post = [t for t in late[0].get("fromStop") or [] if t.get("inPost")]
    return not orphans, (f"{len(watch.get('atStop') or [])} at STOP; {late[0]['afterStopMs']:.0f} ms after: "
                         f"{len(orphans)} alive outside a POST, {len(in_post)} inside one")


def stamp_checks(record):
    rec = record.get("stamp") or {}
    not_run = (record.get("notRun") or {}).get("stamp")
    before, salted, restored = (res(rec.get(k), "firstFrame", "stamp") for k in ("before", "salted", "restored"))
    file_salted, file_restored = rec.get("fileSalted"), rec.get("fileRestored")
    if not_run or not rec:
        why = f"not run: {not_run or 'no record'}"
        return [("limelight reload stamp: a deploySloth with a salted SCRIPT_ID put the salted build's stamp on the "
                 "Limelight", False, why, STAMP_MEANS),
                ("limelight reload stamp: the unsalted deploySloth after it put the build's own stamp back", False, why,
                 RESTORE_MEANS)]
    salted_ok = salted is not None and file_salted is not None and salted == file_salted and salted != before
    restored_ok = restored is not None and restored == file_restored and restored == before

    def load(key):
        return (rec.get(key) or {}).get("load")

    def failures(*keys):
        return "".join(f"; {k}: {rec[k]}" for k in keys if rec.get(k))

    return [("limelight reload stamp: a deploySloth with a salted SCRIPT_ID put the salted build's stamp on the "
             "Limelight", salted_ok,
             f"the Limelight reported {before} before, {salted} after the salted deploySloth (load "
             f"{load('saltedDeploy')}); the salted build's LimelightFiles stamp {file_salted}, the running code's "
             f"{res(rec.get('salted'), 'code', 'stamp')}" + failures("beforeFailed", "saltedFailed"), STAMP_MEANS),
            ("limelight reload stamp: the unsalted deploySloth after it put the build's own stamp back", restored_ok,
             f"the Limelight reported {restored} after it (load {load('restoreDeploy')}); that build's stamp "
             f"{file_restored}; {before} before the salted one" + failures("restoredFailed"), RESTORE_MEANS)]


def contention_posts(trial):
    posts = res(trial.get("answer"), "posts") or []
    old = [p for p in posts if p.get("receivedMs") is not None and p["receivedMs"] < 0]
    new = [p for p in posts if p.get("receivedMs") is not None and p["receivedMs"] >= 0]
    return posts, old, new


def max_overlap_ms(posts, end_ms):
    """How long before an earlier POST ended a later one arrived, at worst: 0 when one never came while another was
    open, however short the later one."""
    worst = 0.0
    spans = sorted((p["receivedMs"], end_ms if p.get("endedMs") is None else p["endedMs"]) for p in posts
                   if p.get("receivedMs") is not None)
    for i, (_, a1) in enumerate(spans):
        for b0, _ in spans[i + 1:]:
            worst = max(worst, a1 - b0)
    return worst


def waiting_rows(run_rec):
    cols, rows = res(run_rec, "rowColumns") or [], res(run_rec, "rows") or []
    need = ("syncing", "senderPermits", "sendersWaiting")
    if not all(c in cols for c in need):
        return []
    i = {c: cols.index(c) for c in need}
    return [row for row in rows if row[i["syncing"]] == 1 and row[i["senderPermits"]] == 0
            and row[i["sendersWaiting"]] >= 1]


def contention_verdict(trial):
    why = lost(trial, "hang", "answer")
    if why:
        return False, why
    posts, old, new = contention_posts(trial)
    held = [p for p in old if p.get("held")]
    setup = []
    if not held:
        setup.append("no POST from the hang run was held on the fake")
    if (trial.get("deploy") or {}).get("load") != "processed" or not loader_changed(trial["hang"], trial["answer"]):
        setup.append("the answer run didn't run reloaded code")
    if not new:
        setup.append("the reloaded code sent no POST")
    if setup:
        return False, "not tested: " + "; ".join(setup)
    end = max([res(trial["answer"], "stop", "ms") or 0] + [p.get("endedMs") or 0 for p in posts])
    overlap = max_overlap_ms(posts, end)
    released = [p.get("endedMs") for p in held]
    first_new = min(p["receivedMs"] for p in new)
    ok = overlap <= rb.LLHUNG_OVERLAP_MS
    return ok, (f"old POST sent {held[0]['receivedMs']:.0f} ms and released "
                f"{'never' if None in released else format(max(released), '.0f') + ' ms'} from the new INIT, the new "
                f"code's first POST at {first_new:.0f} ms, a POST open {overlap:.0f} ms past another's arrival at "
                f"worst, {len(posts)} POSTs")


def exercised_check(record):
    trials = record.get("contention") or []
    name = ("limelight reload contention: a reloaded LimelightSync worker was seen waiting on the blackboard semaphore "
            "while the old code's POST was open")
    if not trials:
        return name, False, f"not run: {(record.get('notRun') or {}).get('contention') or 'no trial recorded'}", \
            EXERCISED_MEANS
    parts, any_ok = [], False
    for t in trials:
        rows = waiting_rows(t.get("answer")) if not t.get("failed") else []
        ok = bool(rows) and loader_changed(t.get("hang"), t.get("answer"))
        any_ok |= ok
        parts.append(f"{t['id']}: {len(rows)} rows with a worker waiting, permits 0, syncing"
                     + ("" if loader_changed(t.get("hang"), t.get("answer")) else " (not reloaded code)"))
    return name, any_ok, ("" if any_ok else "semaphore not exercised; ") + "; ".join(parts), EXERCISED_MEANS


def checks(record):
    not_run = record.get("notRun") or {}
    reload, mid = record.get("reload") or [], record.get("midSync") or []
    sloth_cases = [c for c in mid if c.get("sloth")]
    return [
        aggregate("limelight reload: each first INIT was STOPped during its LimelightSync sync", reload, stop_verdict,
                  STOPPED_MEANS, not_run.get("reload")),
        aggregate("limelight reload: each second INIT ran the reloaded code", reload, reloaded_verdict("second"),
                  RELOADED_MEANS, not_run.get("reload")),
        aggregate(f"limelight reload: every reloaded INIT's sync succeeded, first ball frame within "
                  f"{FIRST_FRAME_MS / 1000:.0f} s of INIT", reload, sync_verdict, SYNC_MEANS, not_run.get("reload")),
        aggregate("limelight reload: the app never restarted across a STOP mid-sync and a deploySloth", reload,
                  restart_verdict, RESTART_MEANS, not_run.get("reload")),
        aggregate(f"limelight reload: no LimelightSync thread {HOLD_S:.0f} s after each reloaded sync ended (in the "
                  "app and from adb)", reload, leftover_verdict, LEFTOVER_MEANS, not_run.get("reload")),
        aggregate(f"limelight reload: the INIT loop never stalled {LOOP_MAX_MS:.0f} ms during a reloaded INIT", reload,
                  loop_verdict, LOOP_MEANS, not_run.get("reload")),
        aggregate("limelight reload mid-sync STOP: each STOP landed during the sync", mid, stop_verdict, STOPPED_MEANS,
                  not_run.get("midSync")),
        aggregate("limelight reload mid-sync STOP with deploySloth: the next INIT ran the reloaded code", sloth_cases,
                  reloaded_verdict("next"), RELOADED_MEANS, not_run.get("midSync")),
        aggregate(f"limelight reload mid-sync STOP: the next INIT's sync finished within {NEXT_SYNC_MS / 1000:.0f} s "
                  f"on pipeline {rb.BALL_PIPELINE} with the build's stamp", mid, next_sync_verdict, NEXT_SYNC_MEANS,
                  not_run.get("midSync")),
        aggregate("limelight reload mid-sync STOP: no pipeline switch after the next sync ended", mid, switch_verdict,
                  SWITCH_MEANS, not_run.get("midSync")),
        aggregate(f"limelight reload mid-sync STOP: no LimelightSync thread from the stopped run "
                  f"{AFTER_STOP_MS / 1000:.0f} s after STOP unless inside a POST", mid, after_stop_verdict,
                  AFTER_STOP_MEANS, not_run.get("midSync")),
        *stamp_checks(record),
        aggregate("limelight reload contention: at the fake Limelight, the reloaded code's first POST came only after "
                  "the hung POST closed, never two open at once", record.get("contention") or [], contention_verdict,
                  CONTENTION_MEANS, not_run.get("contention")),
        exercised_check(record),
    ]


def cell(value, digits=0):
    if value is None:
        return "-"
    return f"{value:.{digits}f}" if isinstance(value, float) else str(value)


def thread_counts(first):
    watch = (first or {}).get("afterStop") or {}
    counts = [str(len(watch.get("atStop") or []))] if watch else ["-"]
    for s in (watch.get("samples") or [])[:2]:
        threads = s.get("fromStop") or []
        counts.append(f"{len(threads)} ({sum(1 for t in threads if t.get('inPost'))} in a POST)")
    return " / ".join(counts)


def lost_row(item, columns):
    return f"| {item['id']} | lost: {str(item['failed'])[:120]} |" + " |" * (columns - 2)


def summary(record):
    s = ["## Bench: Limelight Reload (LimelightSync's one sender across real deploySloth reloads)\n"]
    if record.get("stopped"):
        s.append(f"**Stopped deploying: {record['stopped']}**\n")
    for phase, why in (record.get("notRun") or {}).items():
        s.append(f"- {phase} not run: {why}")
    s += ["", "STOP in the middle of the INIT sync, deploySloth, wait for `Processed Sloth Load`, INIT again on the "
          "reloaded code (the real Limelight):\n",
          "| trial | STOP into sync (ms) | stopped run's LimelightSync threads: at STOP / 1 s / 3 s after | adb 3 s "
          "after | deploySloth (s), load | loader changed | sync (ms), problem | first frame (ms) | threads 5 s after "
          "the sync: app / adb | INIT loop max (ms) | pid |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for t in record.get("reload") or []:
        first, second, deploy = t.get("first"), t.get("second"), t.get("deploy") or {}
        if t.get("failed"):
            s.append(lost_row(t, 11))
            continue
        sync = res(second, "sync") or {}
        hold = res(second, "hold") or {}
        o1, o2 = (first or {}).get("o") or {}, (second or {}).get("o") or {}
        s.append(f"| {t['id']} | {cell(res(first, 'stop', 'msIntoSync'))} | {thread_counts(first)} | "
                 f"{cell((t.get('adbAfterStop') or {}).get('senders'))} | {cell(deploy.get('seconds'), 1)}, "
                 f"{deploy.get('load')} | {loader_changed(first, second)} | {cell(sync.get('ms'))}, "
                 f"{sync.get('problem')!r} | {cell(res(second, 'firstFrame', 'ms'))} | "
                 f"{cell(len(hold['senderThreads']) if hold.get('senderThreads') is not None else None)} / "
                 f"{cell(rb.dig(second, 'runner', 'adbAfterHold', 'senders'))} | "
                 f"{cell(res(second, 'initLoopMs', 'max'), 1)} | {o1.get('pidBefore')} -> {o2.get('pidAfter')} |")
    s += ["", "STOP in the middle of the INIT sync, then the next INIT at once (`at_once`), or with a deploySloth "
          "staged during the first INIT that lands after the STOP (`sloth`):\n",
          "| case | STOP into sync (ms) | stopped run's threads: at STOP / 1 s / 3 s after | pipeline on /status "
          "~2 s after STOP | next sync done (ms after INIT), problem | first frame: pipeline, stamp | pipelines after "
          "the sync | /status after the sync |", "|---|---|---|---|---|---|---|---|"]
    for c in record.get("midSync") or []:
        if c.get("failed"):
            s.append(lost_row(c, 8))
            continue
        nxt = c.get("next")
        sync = res(nxt, "sync") or {}
        first = res(nxt, "firstFrame") or {}
        status = c.get("statusAfterStop") or rb.dig(nxt, "runner", "statusAfterStop", default={}) or {}
        s.append(f"| {c['id']} | {cell(res(c.get('first'), 'stop', 'msIntoSync'))} | {thread_counts(c.get('first'))} | "
                 f"{status.get('pipelineIndex')} ({cell(status.get('afterStopS'), 1)} s) | {cell(sync.get('doneMs'))}, "
                 f"{sync.get('problem')!r} | {first.get('pipeline')}, {cell(first.get('stamp'))} | "
                 f"{[x[1] for x in res(nxt, 'pipelineAfterSync') or []]} | "
                 f"{rb.dig(nxt, 'runner', 'statusAfterSync', 'pipelineIndex')} |")
    sloth = next((c for c in record.get("midSync") or [] if c.get("sloth") and c.get("deploy")), None)
    if sloth:
        d = sloth["deploy"]
        s.append(f"\n- `sloth`: deploySloth {cell(d.get('seconds'), 1)} s while the first run waited in INIT; load "
                 f"{d.get('load')}, push to Staged / to the load's end {d.get('stagedS')} / {d.get('endS')} s")
    st = record.get("stamp") or {}
    if st:
        s += ["", "SnapScript stamp (`llpython[0]` at the first frame) through deploySloth:\n",
              f"- before: the build's LimelightFiles stamp {cell(st.get('fileBefore'))}, the Limelight reported "
              f"{cell(res(st.get('before'), 'firstFrame', 'stamp'))}",
              f"- after a deploySloth built with `-Pbench.scriptSalt={st.get('salt')}`: the build's stamp "
              f"{cell(st.get('fileSalted'))}, the running code's {cell(res(st.get('salted'), 'code', 'stamp'))}, the "
              f"Limelight reported {cell(res(st.get('salted'), 'firstFrame', 'stamp'))}",
              f"- after the unsalted deploySloth: the build's stamp {cell(st.get('fileRestored'))}, the Limelight "
              f"reported {cell(res(st.get('restored'), 'firstFrame', 'stamp'))}"]
    s += ["", f"A fake Limelight on 127.0.0.1:15807 (config `{rb.CONFIG_LLFAULT}`) holds the first POST; STOP, "
          "deploySloth and INIT again while it is held (`after_stop`), or with the deploySloth staged during the "
          f"first INIT (`staged`); assembleSloth first took {cell((record.get('build') or {}).get('seconds'), 1)} s:\n",
          "| order | POST hung at STOP (ms) | deploySloth (s), load | old POST sent / released (ms from the new INIT) | "
          "new code's first POST (ms) | a POST open past another's arrival (ms) | rows with a worker waiting | sync (ms), "
          "problem |",
          "|---|---|---|---|---|---|---|---|"]
    for t in record.get("contention") or []:
        if t.get("failed"):
            s.append(lost_row(t, 8))
            continue
        posts, old, new = contention_posts(t)
        held = [p for p in old if p.get("held")]
        end = max([res(t.get("answer"), "stop", "ms") or 0] + [p.get("endedMs") or 0 for p in posts])
        sync = res(t.get("answer"), "sync") or {}
        deploy = t.get("deploy") or {}
        s.append(f"| {t['id']} | {cell(res(t.get('hang'), 'hungAtDoneMs'))} | {cell(deploy.get('seconds'), 1)}, "
                 f"{deploy.get('load')} | "
                 + (f"{cell(held[0].get('receivedMs'))} / {cell(held[0].get('endedMs'))}" if held else "none held")
                 + f" | {cell(min(p['receivedMs'] for p in new)) if new else '-'} | {max_overlap_ms(posts, end):.0f} | "
                 f"{len(waiting_rows(t.get('answer')))} | {cell(sync.get('ms'))}, {sync.get('problem')!r} |")
    s.append("\n- Nothing logs a Limelight POST: a POST after STOP shows only as a LimelightSync thread whose stack is "
             "inside `Limelight3A.sendPostRequest` (the watcher's 1 s and 3 s samples), and against the fake in its "
             "POST log.\n")
    return s


BENCH = rb.Bench(
    name="limelight_reload",
    title="Bench: Limelight Reload",
    flag="--limelight-reload",
    help="Bench: Limelight Reload: LimelightSync across real deploySloth reloads, STOPs in the middle of the INIT sync, "
         "a SnapScript stamp carried by deploySloth and a fake Limelight holding a POST across a reload (about 20 min, "
         "about 15 Gradle runs; needs the Limelight 3A)",
    run=run,
    checks=checks,
    summary=summary,
    needs=needs,
    configs=(rb.CONFIG_LLFAULT,),
    opmodes=(OPMODE,),
)

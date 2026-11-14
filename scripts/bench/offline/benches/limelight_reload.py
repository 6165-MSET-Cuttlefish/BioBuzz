"""Bench: Limelight Reload offline: its verdicts on synthetic results (pass, each way to fail, missing data; each check
also fails when weakened), then the bench end to end on fake_hub.py with deploySloth stubbed: its runs and deploys in
order, the go file, no START, the contention config and the restore, the skips, and Ctrl-C mid-bench. No hub, no adb,
no network. With --gradle it also runs Gradle's generateLimelightFiles on this Mac: no salt and two salts give three
stamps, and the unsalted one is the CRC the build documents.
Run with: python3 -I scripts/bench/offline/benches/limelight_reload.py [--gradle]
"""
import contextlib
import importlib.util
import io
import json
import pathlib
import re
import signal
import subprocess
import sys
import tempfile
import traceback
import zlib

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
OFFLINE = HERE.parent
WORKTREE = OFFLINE.parents[2]
OUT = OFFLINE / "out" / "limelight_reload"
SOURCE = WORKTREE / "scripts/bench/benches/limelight_reload.py"

passed = 0
failed = []


def expect(label, cond):
    global passed
    if cond:
        passed += 1
    else:
        failed.append(label)
        print("FAIL", label)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rb = load("run_bench", WORKTREE / "scripts/bench/run_bench.py")
fake_hub = load("fake_hub", OFFLINE / "fake_hub.py")
BENCH = rb.module_bench("limelight_reload")
ll = BENCH.module
STAMP = 10774700.0
SALTED = 12054810.0
COLUMNS = ["ms", "syncing", "frameStale", "problem", "senderThreads", "sendersInPost", "sendersWaiting",
           "senderPermits", "fakeOpenPosts", "pipelineIndex"]


def thread(tid, in_post=False, waiting=False):
    return {"id": tid, "state": "RUNNABLE", "inPost": in_post, "waitingForSender": waiting}


def watcher(at_stop, one_s, three_s, others=()):
    return {"atStop": [thread(t) for t in at_stop], "done": True,
            "samples": [{"afterStopMs": 1001.0, "fromStop": list(one_s), "others": []},
                        {"afterStopMs": 3002.0, "fromStop": list(three_s), "others": list(others)}]}


def outcome(pid="4000"):
    return {"initSeenSeconds": 0.4, "crashed": False, "errorMessage": "", "pidBefore": pid, "pidAfter": pid,
            "appRestarted": False, "restartedDuringStop": None, "stopSecondsRunnerSide": 0.2, "timedOut": True,
            "doneLine": None, "logMarkers": []}


def code(loader, stamp=STAMP):
    return {"classLoader": f"SlothClassLoader[/sdcard/FIRST/dairy/sloth/{loader}.jar]", "classLoaderId": loader,
            "syncClassId": loader + 7, "stamp": stamp, "fileScriptId": 6166.0}


def stopped(loader, into_ms, stamp=STAMP):
    """A first run the runner STOPped into_ms into its sync; its worker ended by 1 s after the STOP."""
    res = {"mode": "real", "code": code(loader, stamp), "syncStartMs": 210.0,
           "stop": {"ms": 210.0 + into_ms, "syncing": True, "syncStarted": True, "msIntoSync": into_ms,
                    "senderThreads": [thread(31)], "senderPermits": 1},
           "initLoopMs": {"count": 200, "max": 9.0}}
    return {"o": outcome(), "res": res, "runner": {"syncSeenS": 0.3}, "afterStop": watcher([31], [], [])}


def synced(loader, stamp=STAMP, code_stamp=None, done_ms=1910.0, problem=None, first_ms=2020.0, pipeline=4):
    """A run held in INIT whose sync finished and ran HOLD_S more."""
    res = {"mode": "real", "code": code(loader, code_stamp if code_stamp is not None else stamp),
           "sync": {"startMs": 210.0, "doneMs": done_ms, "ms": done_ms - 210.0, "problem": problem},
           "firstFrame": {"ms": first_ms, "sinceSyncDoneMs": first_ms - done_ms, "pipeline": pipeline, "stamp": stamp,
                          "detections": 0},
           "hold": {"afterSyncMs": 5001.0, "senderThreads": []}, "initLoopMs": {"count": 2500, "max": 12.5},
           "pipelineAfterSync": [[done_ms, 4]], "problemsAfterSync": [{"ms": done_ms, "problem": problem}],
           "stop": {"ms": done_ms + 5400.0, "syncing": False, "syncStarted": True, "msIntoSync": None,
                    "senderThreads": [], "senderPermits": 1}}
    runner = {"doneSeenS": 7.2, "adbAfterHold": {"pid": "4000", "method": "run-as cat /proc/<pid>/task/*/comm",
                                                 "threads": 80, "senders": 0, "tried": []},
              "statusAfterSync": {"code": 200, "error": None, "ms": 6.0, "pipelineIndex": 4}}
    return {"o": outcome(), "res": res, "runner": runner}


def deploy(load="processed"):
    return {"seconds": 9.5, "returncode": 0, "gradleArgs": [], "load": load, "stagedS": 0.2, "endS": 4.8}


def contention_trial(order, loader, waited):
    """The hang run's POST held on the fake, then the reloaded code's sync; waited: the new INIT came inside the 15 s."""
    sent, released = (-9000.0, 6000.0) if waited else (-21000.0, -6000.0)
    first_new = released + 4 if waited else 320.0
    posts = [{"line": "POST /pipeline-switch?index=0 HTTP/1.1", "held": True, "receivedMs": sent,
              "endedMs": released, "endedBy": "client closed"}]
    t = first_new
    for line in ("POST /pipeline-switch?index=0", "POST /upload-pipeline?index=4", "POST /upload-python?index=4",
                 "POST /pipeline-switch?index=4"):
        posts.append({"line": line + " HTTP/1.1", "held": False, "receivedMs": t, "endedMs": t + 3,
                      "endedBy": "answered 200"})
        t += 520 if "index=0" in line else 6
    answer = synced(loader + 1, done_ms=t + 400)
    rows = []
    for ms in range(300, int(t + 400), 100):
        waiting = 1 if waited and ms < released else 0
        rows.append([float(ms), 1, 1, 0, 1 + waiting, waiting, waiting, 0 if waiting else 1, 0, 4])
    answer["res"].update(mode="answer", posts=posts, rows=rows, rowColumns=COLUMNS)
    answer["runner"].pop("statusAfterSync")
    hang = {"o": outcome(), "res": {"mode": "hang", "code": code(loader), "hungAtDoneMs": 304.0,
                                    "posts": [dict(posts[0], receivedMs=110.0, endedMs=None, endedBy=None)],
                                    "stop": {"ms": 420.0, "syncing": True, "syncStarted": True, "msIntoSync": 210.0,
                                             "senderThreads": [thread(41, in_post=True)]}},
            "runner": {"hungSeenS": 0.5}, "afterStop": watcher([41], [thread(41, in_post=True)],
                                                                [thread(41, in_post=True)])}
    return {"id": order, "hang": hang, "deploy": deploy(), "answer": answer}


def good_record():
    loader = 100
    reload = []
    for n in range(1, ll.RELOAD_ROUNDS + 1):
        for stop_s in ll.RELOAD_STOPS_S:
            reload.append({"id": f"{round(stop_s * 1000)}ms_{n}", "stopS": stop_s,
                           "first": stopped(loader, stop_s * 1000 + 40),
                           "adbAfterStop": {"method": "comm", "senders": 0, "afterStopS": 3.1},
                           "deploy": deploy(), "second": synced(loader + 1)})
            loader += 1
    mid = []
    for stop_s in ll.MID_SYNC_STOPS_S:
        mid.append({"id": f"at_once_{round(stop_s * 1000)}ms", "stopS": stop_s, "sloth": False,
                    "first": stopped(loader, stop_s * 1000 + 40), "next": synced(loader)})
    mid.append({"id": "sloth_800ms", "stopS": 0.8, "sloth": True, "first": stopped(loader, 840.0),
                "deploy": deploy(), "statusAfterStop": {"pipelineIndex": 1, "afterStopS": 2.1},
                "next": synced(loader + 1)})
    loader += 1
    stamp = {"fileBefore": STAMP, "before": synced(loader), "salt": "limelight-reload-x",
             "saltedDeploy": deploy(), "fileSalted": SALTED, "salted": synced(loader + 1, stamp=SALTED),
             "restoreDeploy": deploy(), "fileRestored": STAMP, "restored": synced(loader + 2)}
    loader += 3
    contention = [contention_trial("after_stop", loader, waited=False),
                  contention_trial("staged", loader + 2, waited=True)]
    return {"reload": reload, "midSync": mid, "stamp": stamp, "contention": contention, "notRun": {},
            "build": {"seconds": 3.0}}


KEYS = (("each first INIT was STOPped", "stop"), ("each second INIT ran the reloaded code", "reloaded"),
        ("every reloaded INIT's sync succeeded", "sync"), ("the app never restarted", "restart"),
        ("after each reloaded sync ended", "leftover"), ("the INIT loop never stalled", "loop"),
        ("mid-sync STOP: each STOP landed", "mid_stop"), ("STOP with deploySloth: the next INIT ran", "mid_reloaded"),
        ("the next INIT's sync finished", "next_sync"), ("no pipeline switch", "switch"),
        ("from the stopped run", "after_stop"), ("salted SCRIPT_ID", "stamp"), ("own stamp back", "restore"),
        ("at the fake Limelight", "contention"), ("seen waiting on the blackboard", "exercised"))


def verdicts(checks_fn, record):
    out = {}
    for name, ok, detail, means in checks_fn(record):
        key = next(k for part, k in KEYS if part in name)
        out[key] = ok
    return out


def only_fails(*keys):
    return lambda v: {k for k, ok in v.items() if not ok} == set(keys) and len(v) == len(KEYS)


def edit(record, fn):
    fn(record)
    return record


def trial(record, i=0):
    return record["reload"][i]


def mid(record, case_id):
    return next(c for c in record["midSync"] if c["id"] == case_id)


def ctrial(record, order):
    return next(t for t in record["contention"] if t["id"] == order)


CASES = [
    ("every trial as the plan expects: all pass", good_record, only_fails()),
    ("a STOP after the sync ended fails the STOP-landed check",
     lambda: edit(good_record(), lambda r: trial(r, 2)["first"]["res"]["stop"].update(syncing=False, msIntoSync=None)),
     only_fails("stop")),
    ("a STOP just after the sync ended (not syncing any more) fails it",
     lambda: edit(good_record(), lambda r: trial(r, 2)["first"]["res"]["stop"].update(syncing=False)),
     only_fails("stop")),
    ("a STOP before the sync started fails it too",
     lambda: edit(good_record(), lambda r: trial(r, 0)["first"]["res"]["stop"].update(
         syncing=False, syncStarted=False, msIntoSync=None)), only_fails("stop")),
    ("a second run in the first run's class loader fails the reload check",
     lambda: edit(good_record(), lambda r: trial(r, 4)["second"]["res"]["code"].update(
         classLoaderId=trial(r, 4)["first"]["res"]["code"]["classLoaderId"])), only_fails("reloaded")),
    ("a load that never logged Processed fails the reload check",
     lambda: edit(good_record(), lambda r: trial(r, 1)["deploy"].update(load="nothing")), only_fails("reloaded")),
    ("a missing class loader id fails the reload check",
     lambda: edit(good_record(), lambda r: trial(r, 1)["second"]["res"]["code"].pop("classLoaderId")),
     only_fails("reloaded")),
    ("a reloaded sync with a problem fails the sync check",
     lambda: edit(good_record(), lambda r: trial(r, 3)["second"]["res"]["sync"].update(
         problem="pipeline 4 never reported the upload's stamp")), only_fails("sync")),
    ("a first frame 5.2 s after INIT fails the sync check",
     lambda: edit(good_record(), lambda r: trial(r, 3)["second"]["res"]["firstFrame"].update(ms=5200.0)),
     only_fails("sync")),
    ("no first frame fails the sync check",
     lambda: edit(good_record(), lambda r: trial(r, 3)["second"]["res"].pop("firstFrame")), only_fails("sync")),
    ("a sync that never finished fails the sync check",
     lambda: edit(good_record(), lambda r: trial(r, 3)["second"]["res"].pop("sync")), only_fails("sync")),
    ("a pid change between the runs fails the restart check",
     lambda: edit(good_record(), lambda r: trial(r, 5)["second"]["o"].update(pidBefore="4001", pidAfter="4001")),
     only_fails("restart")),
    ("a restart the runner flagged fails it even with the same pid",
     lambda: edit(good_record(), lambda r: trial(r, 5)["first"]["o"].update(restartedDuringStop=True)),
     only_fails("restart")),
    ("a LimelightSync thread in the app 5 s after the sync fails the leftover check",
     lambda: edit(good_record(), lambda r: trial(r, 6)["second"]["res"]["hold"].update(senderThreads=[thread(77)])),
     only_fails("leftover")),
    ("one seen only from adb fails it too",
     lambda: edit(good_record(), lambda r: trial(r, 6)["second"]["runner"]["adbAfterHold"].update(senders=1)),
     only_fails("leftover")),
    ("no adb listing on this hub leaves the in-app count to judge: passes",
     lambda: edit(good_record(), lambda r: trial(r, 6)["second"]["runner"]["adbAfterHold"].update(
         method=None, senders=None)), only_fails()),
    ("no hold record fails the leftover check",
     lambda: edit(good_record(), lambda r: trial(r, 6)["second"]["res"].pop("hold")), only_fails("leftover")),
    ("a 55 ms INIT loop fails the loop check",
     lambda: edit(good_record(), lambda r: trial(r, 7)["second"]["res"]["initLoopMs"].update(max=55.0)),
     only_fails("loop")),
    ("no loop record fails the loop check",
     lambda: edit(good_record(), lambda r: trial(r, 7)["second"]["res"].pop("initLoopMs")), only_fails("loop")),
    ("a lost reload trial fails every reload check",
     lambda: edit(good_record(), lambda r: r["reload"][8].update(failed="Bench: Limelight Reload to INIT timed out")),
     only_fails("stop", "reloaded", "sync", "restart", "leftover", "loop")),
    ("a mid-sync STOP after the sync fails its STOP check",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_300ms")["first"]["res"]["stop"].update(
         syncing=False, msIntoSync=None)), only_fails("mid_stop")),
    ("the sloth case's next run on the old loader fails its reload check",
     lambda: edit(good_record(), lambda r: mid(r, "sloth_800ms")["next"]["res"]["code"].update(
         classLoaderId=mid(r, "sloth_800ms")["first"]["res"]["code"]["classLoaderId"])), only_fails("mid_reloaded")),
    ("the next sync done 5.6 s after INIT fails the next-sync check",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_800ms")["next"]["res"]["sync"].update(doneMs=5600.0)),
     only_fails("next_sync")),
    ("the next sync's first frame on pipeline 0 fails it",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_800ms")["next"]["res"]["firstFrame"].update(pipeline=0)),
     only_fails("next_sync")),
    ("a stamp other than the build's fails it",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_800ms")["next"]["res"]["firstFrame"].update(stamp=6166.0)),
     only_fails("next_sync")),
    ("a problem in the next sync fails it",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_1300ms")["next"]["res"]["sync"].update(problem="cancelled")),
     only_fails("next_sync")),
    ("the pipeline leaving 4 after the next sync fails the switch check",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_300ms")["next"]["res"]["pipelineAfterSync"].append(
         [4100.0, 0])), only_fails("switch")),
    ("the source's 'on pipeline 0, want 4' after the sync fails it",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_300ms")["next"]["res"]["problemsAfterSync"].append(
         {"ms": 5200.0, "problem": "on pipeline 0, want 4"})), only_fails("switch")),
    ("/status on another pipeline after the sync fails it",
     lambda: edit(good_record(), lambda r: mid(r, "sloth_800ms")["next"]["runner"]["statusAfterSync"].update(
         pipelineIndex=0)), only_fails("switch")),
    ("no /status reading after the sync fails it",
     lambda: edit(good_record(), lambda r: mid(r, "sloth_800ms")["next"]["runner"].pop("statusAfterSync")),
     only_fails("switch")),
    ("a connection blip after the sync isn't a switch: passes",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_300ms")["next"]["res"]["problemsAfterSync"].append(
         {"ms": 5200.0, "problem": "no data for 0.3 s"})), only_fails()),
    ("a stopped run's thread alive 3 s after STOP outside a POST fails the after-STOP check",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_800ms")["first"].update(
         afterStop=watcher([31], [thread(31)], [thread(31)]))), only_fails("after_stop")),
    ("one still inside a POST is allowed: passes",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_800ms")["first"].update(
         afterStop=watcher([31], [thread(31, in_post=True)], [thread(31, in_post=True)]))), only_fails()),
    ("the next run's own worker 3 s after STOP isn't the stopped run's: passes",
     lambda: edit(good_record(), lambda r: mid(r, "at_once_300ms")["first"].update(
         afterStop=watcher([31], [], [], others=[thread(58)]))), only_fails()),
    ("no watcher file fails the after-STOP check",
     lambda: edit(good_record(), lambda r: mid(r, "sloth_800ms")["first"].update(afterStop=None)),
     only_fails("after_stop")),
    ("a watcher file without its 3 s sample fails it",
     lambda: edit(good_record(), lambda r: mid(r, "sloth_800ms")["first"]["afterStop"].update(
         done=False, samples=mid(r, "sloth_800ms")["first"]["afterStop"]["samples"][:1])), only_fails("after_stop")),
    ("a lost mid-sync case fails all four of its checks",
     lambda: edit(good_record(), lambda r: mid(r, "sloth_800ms").update(failed="deploySloth failed")),
     only_fails("mid_stop", "mid_reloaded", "next_sync", "switch", "after_stop")),
    ("the stamp unchanged by the salted deploySloth fails the stamp check",
     lambda: edit(good_record(), lambda r: r["stamp"]["salted"]["res"]["firstFrame"].update(stamp=STAMP)),
     only_fails("stamp")),
    ("a salt that didn't change the build's stamp fails it",
     lambda: edit(good_record(), lambda r: (r["stamp"].update(fileSalted=STAMP),
                                            r["stamp"]["salted"]["res"]["firstFrame"].update(stamp=STAMP))),
     only_fails("stamp")),
    ("a stamp that isn't the salted build's fails it",
     lambda: edit(good_record(), lambda r: r["stamp"]["salted"]["res"]["firstFrame"].update(stamp=12000000.0)),
     only_fails("stamp")),
    ("no salted build stamp (file not found) fails it",
     lambda: edit(good_record(), lambda r: r["stamp"].update(fileSalted=None)), only_fails("stamp")),
    ("no salted run fails it",
     lambda: edit(good_record(), lambda r: r["stamp"].pop("salted")), only_fails("stamp")),
    ("the salted stamp left after the unsalted deploySloth fails the restore check",
     lambda: edit(good_record(), lambda r: r["stamp"]["restored"]["res"]["firstFrame"].update(stamp=SALTED)),
     only_fails("restore")),
    ("no restored run fails it",
     lambda: edit(good_record(), lambda r: r["stamp"].pop("restored")), only_fails("restore")),
    ("the reloaded code's POST while the hung one is open fails the contention check",
     lambda: edit(good_record(), lambda r: ctrial(r, "staged")["answer"]["res"]["posts"].insert(1, {
         "line": "POST /pipeline-switch?index=0 HTTP/1.1", "held": False, "receivedMs": 2000.0, "endedMs": 2003.0,
         "endedBy": "answered 200"})), only_fails("contention")),
    ("two of the reloaded code's POSTs open at once fail it",
     lambda: edit(good_record(), lambda r: ctrial(r, "staged")["answer"]["res"]["posts"][1].update(endedMs=6700.0)),
     only_fails("contention")),
    ("the old POST never released fails it",
     lambda: edit(good_record(), lambda r: ctrial(r, "staged")["answer"]["res"]["posts"][0].update(endedMs=None)),
     only_fails("contention")),
    ("no held POST from the hang run: not tested, fails it",
     lambda: edit(good_record(), lambda r: ctrial(r, "after_stop")["answer"]["res"]["posts"].pop(0)),
     only_fails("contention")),
    ("an answer run on the hang run's loader fails it, and that trial can't count as waiting",
     lambda: edit(good_record(), lambda r: ctrial(r, "staged")["answer"]["res"]["code"].update(
         classLoaderId=ctrial(r, "staged")["hang"]["res"]["code"]["classLoaderId"])),
     only_fails("contention", "exercised")),
    ("no trial with a worker waiting: semaphore not exercised",
     lambda: edit(good_record(), lambda r: ctrial(r, "staged").update(
         contention_trial("staged", 300, waited=False))), only_fails("exercised")),
    ("rows with a worker waiting but a free permit don't count",
     lambda: edit(good_record(), lambda r: [row.__setitem__(7, 1) for row in ctrial(r, "staged")["answer"]["res"]["rows"]]),
     only_fails("exercised")),
    ("a lost contention trial fails both contention checks when it was the waiting one",
     lambda: edit(good_record(), lambda r: ctrial(r, "staged").update(failed="no BENCH DONE")),
     only_fails("contention", "exercised")),
    ("phases not run fail their checks with the reason",
     lambda: edit(good_record(), lambda r: r.update(midSync=[], stamp={}, contention=[], notRun={
         p: "the bench stopped deploying in reload: deploySloth hung" for p in ("midSync", "stamp", "contention")})),
     only_fails("mid_stop", "mid_reloaded", "next_sync", "switch", "after_stop", "stamp", "restore", "contention",
                "exercised")),
    ("an empty record fails every check", lambda: {}, only_fails(*[k for _, k in KEYS])),
]


def run_cases(checks_fn):
    results = []
    for label, make, predicate in CASES:
        try:
            ok = predicate(verdicts(checks_fn, make()))
        except Exception as e:
            ok = False
            label = f"{label} ({type(e).__name__}: {e})"
        results.append((label, ok))
    return results


def verdict_cases():
    for label, ok in run_cases(BENCH.checks):
        expect(f"verdict: {label}", ok)
    names = [c[0] for c in BENCH.checks(good_record())]
    expect("checks: fifteen, each name once", len(names) == len(KEYS) == len(set(names)))
    details = {next(k for part, k in KEYS if part in c[0]): c[2] for c in BENCH.checks({})}
    expect("a check with nothing recorded says it wasn't run", all(d.startswith("not run") for d in details.values()))
    bad = edit(good_record(), lambda r: ctrial(r, "staged").update(contention_trial("staged", 300, waited=False)))
    exercised = next(c for c in BENCH.checks(bad) if "seen waiting" in c[0])
    expect("not exercised reads 'semaphore not exercised' and isn't a PASS",
           not exercised[1] and exercised[2].startswith("semaphore not exercised"))
    text = "\n".join(BENCH.summary(good_record()))
    expect("summary: a row per reload trial, mid-sync case and contention order, and the three stamps",
           all(f"| {t} |" in text for t in ("300ms_1", "1400ms_3", "at_once_1300ms", "sloth_800ms", "after_stop",
                                            "staged"))
           and "12054810" in text and "10774700" in text)
    lost = edit(good_record(), lambda r: r["reload"][0].update(failed="x" * 300))
    row = next(line for line in BENCH.summary(lost) if line.startswith("| 300ms_1 |"))
    expect("summary: a lost trial keeps the table's column count", row.count("|") == 12)
    json.dumps(good_record())
    expect("the record turns into JSON", True)


MUTATIONS = [
    ("the first-frame limit raised out of reach", "FIRST_FRAME_MS = 5000.0", "FIRST_FRAME_MS = 5e9"),
    ("the next-sync limit raised out of reach", "NEXT_SYNC_MS = 5000.0", "NEXT_SYNC_MS = 5e9"),
    ("the loop limit raised out of reach", "LOOP_MAX_MS = 50.0", "LOOP_MAX_MS = 5e9"),
    ("a STOP outside the sync counts", 'return stop.get("syncing") is True and stop.get("msIntoSync") is not None',
     "return True"),
    ("the class loader isn't compared", "return a is not None and b is not None and a != b", "return True"),
    ("a load that never landed counts", 'return load == "processed" and changed,', "return changed,"),
    ("a sync problem is ignored", 'ok = "doneMs" in sync and sync.get("problem") is None and', 'ok = "doneMs" in sync and'),
    ("pids aren't compared", "return None not in pids and len(set(pids)) == 1 and not any(flags)",
     "return not any(flags)"),
    ("a leftover thread in the app is ignored", 'ok = inside == [] and seen in (0, None)', 'ok = seen in (0, None)'),
    ("the adb listing is ignored", 'ok = inside == [] and seen in (0, None)', 'ok = inside == []'),
    ("the next sync's pipeline isn't checked", "and first.get(\"pipeline\") == rb.BALL_PIPELINE and", "and"),
    ("pipelines after the sync aren't checked", "all(c[1] == rb.BALL_PIPELINE for c in changes)", "True"),
    ("/status after the sync isn't checked", "and status == rb.BALL_PIPELINE)", ")"),
    ("an orphan outside a POST is ignored", 'orphans = [t for t in late[0].get("fromStop") or [] if not t.get("inPost")]',
     "orphans = []"),
    ("an unchanged stamp passes", "and salted == file_salted and salted != before", "and salted == file_salted"),
    ("the restore isn't compared", "restored_ok = restored is not None and restored == file_restored and "
                                   "restored == before", "restored_ok = True"),
    ("overlapping POSTs pass", "ok = overlap <= rb.LLHUNG_OVERLAP_MS", "ok = True"),
    ("a short POST inside an open one passes", "worst = max(worst, a1 - b0)", "worst = max(worst, min(a1, _) - b0)"),
    ("a free permit counts as waiting", 'row[i["senderPermits"]] == 0', "True"),
    ("an answer run on the old loader counts as waiting",
     'ok = bool(rows) and loader_changed(t.get("hang"), t.get("answer"))', "ok = bool(rows)"),
    ("a trial with nothing held passes as tested", '        setup.append("no POST from the hang run was held on the fake")',
     "        pass"),
]


def mutated(old, new):
    text = SOURCE.read_text()
    if text.count(old) != 1:
        return None
    OUT.mkdir(parents=True, exist_ok=True)
    folder = pathlib.Path(tempfile.mkdtemp(prefix="mutant-", dir=OUT))
    (folder / "limelight_reload.py").write_text(text.replace(old, new))
    (folder / "order.txt").write_text("limelight_reload\n")
    return rb.load_benches(folder)[0]


def mutation_cases():
    for label, old, new in MUTATIONS:
        bench = mutated(old, new)
        if bench is None:
            expect(f"weakened: {label}: the text to weaken is in limelight_reload.py once", False)
            continue
        caught = [case for case, ok in run_cases(bench.checks) if not ok]
        expect(f"weakened: {label}: a verdict case fails ({caught[:1]})", bool(caught))


class Station(fake_hub.Hub):
    """fake_hub's Control Hub with a Limelight, Bench: Limelight Reload scripted from the same builders the verdict
    cases use, a class loader that changes at each deploySloth and a stamp that follows -Pbench.scriptSalt."""

    def __init__(self, limelight=True, leftover_from_adb=False, ctrl_c_at=None, **kw):
        super().__init__(**kw)
        if limelight:
            self.probe["limelights"] = [{"status": "ok", "ipAddress": "172.29.0.1",
                                         "serial": "EthernetOverUsb:eth0:172.29.0.22"}]
        self.loader = 100
        self.file_stamp = self.hub_stamp = STAMP
        self.leftover_from_adb = leftover_from_adb
        self.ctrl_c_at = ctrl_c_at
        self.events, self.inits, self.started = [], [], []
        self.current = None
        self.current_loader = None

    def telemetry(self, data):
        self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": [
            {"data": data, "timestamp": self.clock, "fieldOverlay": {"ops": []}}]})

    def result(self, run_id, value):
        self.bench[f"files/bench/{ll.RESULT}{run_id}.json"] = json.dumps(value)

    def on_message(self, session, msg):
        kind = msg.get("type")
        with self.lock:
            active = self.status["activeOpMode"]
            if kind == "INIT_OP_MODE" and msg.get("opModeName") == ll.OPMODE:
                params = json.loads(self.bench.get("files/bench/params.json") or "{}")
                self.inits.append(params)
                self.events.append(("init", params["runId"]))
                self.current = params
                self.current_loader = self.loader
                self.status["errorMessage"] = ""
                self.dash.broadcast({"type": "RECEIVE_TELEMETRY", "telemetry": []})
                self.set_active(ll.OPMODE, "INIT")
                self.on_init(params)
                return
            if kind == "START_OP_MODE" and active == ll.OPMODE:
                self.started.append(self.current)
            if kind == "STOP_OP_MODE" and active == ll.OPMODE:
                self.events.append(("stop", self.current["runId"]))
                self.on_stop(self.current)
        super().on_message(session, msg)

    def answer_result(self, params):
        if params["mode"] == "answer":
            order = params["runId"].split("_answer")[0][len("contention_"):]
            return contention_trial(order, self.loader - 1, waited=order == "staged")["answer"]["res"]
        return synced(self.loader, stamp=self.hub_stamp)["res"]

    def on_init(self, params):
        run_id = params["runId"]
        if params.get("waitForGo"):
            self.telemetry({"Bench phase": "waiting"})
        elif params["mode"] == "hang":
            self.result(run_id, {"mode": "hang", "code": code(self.loader), "hungAtDoneMs": 304.0})
            self.telemetry({"Bench phase": "hung", "BENCH DONE": "a POST has hung 304 ms on the fake; STOP now"})
        elif "holdAfterSyncS" not in params:
            self.telemetry({"Bench phase": "sync"})
        else:
            self.result(run_id, self.answer_result(params))
            self.telemetry({"Bench phase": "synced", "BENCH DONE": "sync done in 1700 ms"})

    def on_stop(self, params):
        run_id = params["runId"]
        if params["mode"] == "real" and "holdAfterSyncS" not in params:
            first = stopped(self.current_loader, 840.0, stamp=self.hub_stamp)
            self.result(run_id, first["res"])
            self.bench[f"files/bench/{ll.RESULT}{run_id}{ll.AFTER_STOP}.json"] = json.dumps(first["afterStop"])

    def _bench_shell(self, inner):
        if "/task/*/comm" in inner:
            names = [f"Binder:{i}" for i in range(20)] + (["LimelightSync"] if self.leftover_from_adb else [])
            return "".join(n + "\n" for n in names), "", 0
        out = super()._bench_shell(inner)
        if inner.startswith("printf") and f"{ll.GO_FILE}.json" in inner:
            self.events.append(("go", self.current["runId"] if self.current else None))
            self.telemetry({"Bench phase": "sync"})
        return out

    def popen(self, argv, env=None, **kw):
        if pathlib.Path(argv[0]).name == "gradlew":
            with self.lock:
                if self.ctrl_c_at is not None and self.ctrl_c_at(self):
                    raise KeyboardInterrupt
                salt = next((a.split("=", 1)[1] for a in argv if a.startswith("-Pbench.scriptSalt=")), None)
                self.events.append(("gradle", argv[1], salt, self.current["runId"] if self.current else None))
                if argv[1] == ":TeamCode:deploySloth":
                    self.loader += 1
                    self.file_stamp = self.hub_stamp = SALTED if salt else STAMP
        return super().popen(argv, env=env, **kw)


def station_run(argv, hub):
    OUT.mkdir(parents=True, exist_ok=True)
    out = pathlib.Path(tempfile.mkdtemp(prefix="run-", dir=OUT)) / "run"
    handlers = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGHUP)
    saved_stamp = ll.built_stamp
    ll.built_stamp = lambda root: hub.file_stamp
    interrupted = False
    try:
        with hub.installed(rb), contextlib.redirect_stdout(io.StringIO()):
            try:
                code = rb.Runner(rb.parse_args(["--out", str(out)] + list(argv))).run()
            except KeyboardInterrupt:
                code, interrupted = None, True
    finally:
        ll.built_stamp = saved_stamp
        signal.signal(signal.SIGINT, handlers[0])
        signal.signal(signal.SIGHUP, handlers[1])
    return code, out, interrupted


def check_line(summary, name_part):
    return next((l for l in summary.splitlines() if l.startswith(("- PASS", "- FAIL")) and name_part in l), "")


def on_the_fake_hub():
    hub = Station()
    code, out, _ = station_run(["--ports-empty", "--limelight-reload"], hub)
    summary = (out / "summary.md").read_text()
    log = (out / "run.log").read_text()
    ids = [p["runId"] for p in hub.inits]
    gradle = [e for e in hub.events if e[0] == "gradle"]
    deploys = [e for e in gradle if e[1] == ":TeamCode:deploySloth"]
    expect("the bench ran and every one of its checks passed on the scripted hub",
           "] === limelight_reload ===" in log and code == 0
           and all(check_line(summary, k).startswith("- PASS") for k, _ in KEYS))
    reload_ids = [f"reload_{round(s * 1000)}ms_{n}_{x}" for n in range(1, 4) for s in ll.RELOAD_STOPS_S
                  for x in ("a", "b")]
    expect("reload: 9 trials, each a run STOPped mid-sync, a deploySloth, then a held run",
           ids[:18] == reload_ids and [e[3] for e in deploys[:9]] == [f"reload_{round(s * 1000)}ms_{n}_a"
                                                                     for n in range(1, 4) for s in ll.RELOAD_STOPS_S])
    expect("mid-sync: three STOPs with the next INIT at once, then the sloth case, in order",
           ids[18:26] == ["midsync_at_once_300ms_a", "midsync_at_once_300ms_b", "midsync_at_once_800ms_a",
                          "midsync_at_once_800ms_b", "midsync_at_once_1300ms_a", "midsync_at_once_1300ms_b",
                          "midsync_sloth_800ms_a", "midsync_sloth_800ms_b"])
    i_init = hub.events.index(("init", "midsync_sloth_800ms_a"))
    i_go = next(i for i, e in enumerate(hub.events) if e[0] == "go")
    i_stop = hub.events.index(("stop", "midsync_sloth_800ms_a"))
    i_dep = next(i for i, e in enumerate(hub.events) if e[0] == "gradle" and e[3] == "midsync_sloth_800ms_a")
    expect("sloth case: deploySloth while its first run waits in INIT, then the go file, then the STOP",
           i_init < i_dep < i_go < i_stop and hub.events[i_go][1] == "midsync_sloth_800ms_a"
           and hub.inits[ids.index("midsync_sloth_800ms_a")].get("waitForGo") == 1)
    at_once = [i for i, e in enumerate(hub.events) if e == ("stop", "midsync_at_once_300ms_a")][0]
    expect("at once: no deploySloth between a mid-sync STOP and the next INIT",
           hub.events[at_once + 1] == ("init", "midsync_at_once_300ms_b"))
    salted = [e for e in deploys if e[2]]
    expect("stamp: one salted deploySloth, the run after it, then an unsalted one and a run after that",
           ids[26:29] == ["stamp_before", "stamp_salted", "stamp_restored"] and len(salted) == 1
           and salted[0][2].startswith("limelight-reload-") and deploys[deploys.index(salted[0]) + 1][2] is None)
    expect("contention: on bench_llfault, assembleSloth first, then for each order a hang run and an answer run",
           ids[29:] == ["contention_after_stop_hang", "contention_after_stop_answer", "contention_staged_hang",
                        "contention_staged_answer"]
           and [e[1] for e in gradle if e[3] and e[3].startswith(("stamp_restored", "contention"))][0]
           == ":TeamCode:assembleSloth"
           and 'name="limelight"' in (out / "logs" / "bench_llfault.xml").read_text())
    staged_hang = hub.events.index(("init", "contention_staged_hang"))
    staged_stop = hub.events.index(("stop", "contention_staged_hang"))
    after_stop = hub.events.index(("stop", "contention_after_stop_hang"))
    expect("contention: staged deploys while the hang run is in INIT; after_stop deploys after its STOP",
           any(e[0] == "gradle" and e[1] == ":TeamCode:deploySloth" for e in hub.events[staged_hang:staged_stop])
           and hub.events[after_stop + 1][0] == "gradle")
    expect("14 deploySloths, each with this hub's serial; never a START", len(deploys) == 14
           and all(d["ANDROID_SERIAL"] == hub.serial for d in hub.deploys if d["argv"][0] == ":TeamCode:deploySloth")
           and not hub.started)
    expect("the answer runs run in answer mode, the hang runs in hang mode",
           [p["mode"] for p in hub.inits[29:]] == ["hang", "answer", "hang", "answer"])
    expect("the run ended on the original config with no bench config left",
           hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")])
    results = json.loads((out / "runner_results.json").read_text()).get("limelight_reload") or {}
    expect("record: 9 reload trials, 4 mid-sync cases, the stamps, 2 contention trials",
           len(results.get("reload") or []) == 9 and len(results.get("midSync") or []) == 4
           and results.get("stamp", {}).get("fileSalted") == SALTED and len(results.get("contention") or []) == 2)

    hub = Station(leftover_from_adb=True)
    hub_rounds, ll.RELOAD_ROUNDS = ll.RELOAD_ROUNDS, 1
    try:
        code, out, _ = station_run(["--ports-empty", "--limelight-reload"], hub)
    finally:
        ll.RELOAD_ROUNDS = hub_rounds
    summary = (out / "summary.md").read_text()
    expect("a LimelightSync thread adb lists after a reloaded sync fails the leftover check and the run exits 1",
           check_line(summary, "after each reloaded sync ended").startswith("- FAIL") and code == 1
           and "from adb" in check_line(summary, "after each reloaded sync ended"))

    hub = Station(limelight=False)
    code, out, _ = station_run(["--ports-empty", "--limelight-reload"], hub)
    summary = (out / "summary.md").read_text()
    expect("no Limelight: the run stops after the probe, saying what to plug in, nothing INITed or built",
           code == 1 and "No Limelight answers" in summary and "or drop --limelight-reload" in summary
           and not hub.inits and not hub.gradle_procs and hub.active["name"] == "betta_decode")

    hub = Station()
    code, out, _ = station_run(["--ports-empty", "--limelight-reload", "--no-sloth"], hub)
    summary = (out / "summary.md").read_text()
    expect("--no-sloth: the run stops before the bench, saying to drop one of the flags",
           code == 1 and "drop --no-sloth" in summary and not hub.inits and not hub.gradle_procs)

    def at_first_contention_deploy(h):
        return h.current is not None and h.current["runId"].startswith("contention_after_stop")

    hub = Station(ctrl_c_at=at_first_contention_deploy)
    rounds, ll.RELOAD_ROUNDS = ll.RELOAD_ROUNDS, 1
    try:
        code, out, interrupted = station_run(["--ports-empty", "--limelight-reload"], hub)
    finally:
        ll.RELOAD_ROUNDS = rounds
    log = (out / "run.log").read_text()
    expect("Ctrl-C during the contention phase (on bench_llfault): the original config is put back, no bench config "
           "left, the summary written", interrupted and "putting the hardware config back" in log
           and hub.active["name"] == "betta_decode" and not [n for n in hub.configs if n.startswith("bench_")]
           and (out / "summary.md").exists())



def gradle_stamp(salt=None):
    cmd = [str(WORKTREE / "gradlew"), ":TeamCode:generateLimelightFiles", "--console=plain", "--offline", "-q"]
    if salt is not None:
        cmd.append(f"-Pbench.scriptSalt={salt}")
    r = subprocess.run(cmd, cwd=WORKTREE, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"generateLimelightFiles failed: {(r.stderr or r.stdout)[-500:]}")
    text = next((WORKTREE / ll.GENERATED).rglob("LimelightFiles.java")).read_text()
    stamp = ll.STAMP_LINE.search(text)
    uploaded = re.search(r"SCRIPT_ID = (\d+)\.0  # the robot's upload stamp", text.encode().decode("unicode_escape"))
    return float(stamp.group(1)), float(uploaded.group(1)) if uploaded else None


def documented_stamp(salt=None):
    script = (WORKTREE / "limelight/ball_contour_snapscript.py").read_text().replace("\r\n", "\n")
    pipeline = (WORKTREE / "limelight/ball_pipeline.vpr").read_text().replace("\r\n", "\n").strip()
    crc = zlib.crc32((script + "\0" + pipeline + ("" if salt is None else "\0" + salt)).encode("utf-8"))
    return float(0x800000 | (crc & 0x7FFFFF))


def gradle_check():
    plain = gradle_stamp()
    a = gradle_stamp("a")
    b = gradle_stamp("b")
    again = gradle_stamp()
    expect(f"gradle: no salt gives the documented CRC stamp ({plain[0]:.0f})", plain[0] == documented_stamp())
    expect(f"gradle: salts a and b give two more stamps ({a[0]:.0f}, {b[0]:.0f})",
           len({plain[0], a[0], b[0]}) == 3 and a[0] == documented_stamp("a") and b[0] == documented_stamp("b"))
    expect("gradle: the uploaded script's SCRIPT_ID line carries each build's stamp",
           all(s == u for s, u in (plain, a, b, again)))
    expect("gradle: dropping the salt puts the stamp back", again == plain)


def declaration():
    expect("declared: flag-only, no person, drives nothing, Bench: Limelight Reload, writes bench_llfault",
           not BENCH.default and BENCH.person is None and not BENCH.drives and BENCH.statement is None
           and BENCH.opmodes == (ll.OPMODE,) and BENCH.configs == (rb.CONFIG_LLFAULT,))
    expect("its OpMode is required at connect", ll.OPMODE in rb.BENCH_OPMODES)
    expect("--limelight-reload parses", rb.parse_args(["--limelight-reload"]).limelight_reload)
    java = (WORKTREE / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/bench/"
            "LimelightReloadBench.java").read_text()
    expect("the OpMode's row columns are the ones the checks read",
           all(f'"{c}"' in java for c in COLUMNS) and f'"{ll.GO_FILE}.json"' in java and '"_afterstop"' in java)


def main():
    sections = [verdict_cases, mutation_cases, declaration, on_the_fake_hub]
    if "--gradle" in sys.argv[1:]:
        sections.append(gradle_check)
    for section in sections:
        try:
            section()
        except Exception as e:
            expect(f"{section.__name__} ran to its end ({type(e).__name__}: {e})", False)
            traceback.print_exc()
    print(f"{passed} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

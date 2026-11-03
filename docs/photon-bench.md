# Photon bench status

Photon (`architecture/hardware/photon/`) is the team's fork of SolversLib Photon. It sends motor and servo writes
without waiting for each one's ACK, which on the bench cut a loop with 12 changing writes from 30.7 ms to 16.0 ms. No
robot OpMode uses it yet, and none may until the hub bench's Photon block says **SAFE TO TRY ON THE ROBOT** for the
robot's wiring. This page is where that stands and how to run the rest. How the block works and what each check
proves: [`scripts/bench/README.md`, The Photon block](../scripts/bench/README.md#the-photon-block).

## Why a fork

Stock SolversLib Photon 0.3.6 is unsafe on our hubs. Never put that jar, or the SolversLib Photon dependency, back on
the robot:

- A lost ACK leaks an in-flight slot until the app restarts; once enough have leaked, the next write spins forever.
- It never retransmits, so a lost `setPower(0)` leaves a wheel running.
- It takes over every OpMode's hubs, and with a USB Expansion Hub it registers the Control Hub's replacement with the
  wrong USB device, so the Control Hub's replies go to an orphaned hub object, even with Photon disabled.
- After a USB detach it keeps writing to the stale device and the OpMode hangs.
- Its writes skip STOP's gate for the rest of that loop and through `stop()`, up to about 1 s.

## Where it stands

| date | Photon at | wiring | run | verdict | notes |
|---|---|---|---|---|---|
| 2026-10-28 | stock SolversLib Photon 0.3.6 | USB | speed only, earlier bench | none | write block 25.9 to 6.0 ms p50 at 8 parallel, loop 31.8 to 11.5 ms; showed the misrouted replies |
| 2026-10-29 | `c825493` | USB | short, no I2C | NOT SAFE YET | 3 problems, all fixed since (below); write block 24.96 to 10.84 ms p50, loop 30.7 to 16.0 ms |

The 2026-10-29 problems, and what changed:

1. **A USB write that throws** took 27-53 ms to fall back to the SDK's path, because Photon logged a full stack trace
   while holding the write lock. It now logs one line, outside the lock.
2. **Linear STOP** was a bench bug: LinearOpMode interrupts its thread at STOP, and the post-STOP read ran on that
   interrupted thread. The bench now clears the interrupt, and adds a Photon-off linear control and a linear negative
   control.
3. **One SDK `abandoning lock`** in a linear STOP trial: the SDK's own lock leak from that interrupt, which happens
   without Photon too. It's now judged by a Fisher test against the Photon-off linear trials.

Everything else passed: exact read-backs, routing over 8 app restarts in both swap orders, lost writes, a crash with
writes in flight, deploySloth, a hub reset, and 40 + 40 iterative STOPs.

Also changed since that run:

- Photon's hub objects share the SDK's per-hub I2C lock.
- A reply to a message number both hub objects hold is dropped and counted (`numberClashes`).
- `setMaxParallelWrites` takes 1 to 64.
- The ping timer is reset at most every 100 ms instead of on every write. A desktop comparison ranked that reset as the
  biggest reason the fork was slower than stock Photon (10.84 against 6.0 ms).
- Per-step write profiling (`timing on8 profiled` in the summary).
- The bench reads the Pinpoint and the OctoQuad over I2C on both hubs (`--photon-i2c`).

**None of this has run on a hub yet.** Offline it passes the suite in
[`scripts/bench/photon-offline/`](../scripts/bench/photon-offline/README.md).

## Before every run

**Install.** Full install from Android Studio (TeamCode configuration, Run button), from the same checkout you run the
runner in, and again after any change to `architecture/hardware/photon/`. deploySloth never reloads Photon's `@Pinned`
classes; the block's first check compares the hub's Photon with your sources and fails on a mismatch. Run from a
committed tree: the summary records the git hash.

**Wiring.** Unplug everything not in this table.

| | Control Hub | Expansion Hub |
|---|---|---|
| power | 12 V battery | from the Control Hub's second XT30 port |
| motor ports 0-3, servo ports 0-1 | **empty** | **empty** |
| I2C | OctoQuad in port 2 | goBILDA Pinpoint in port 1 |
| link | | runs 1 and 3: USB cable from the Control Hub to its mini-USB, no RS485 cable; run 2: RS485 cable only, address 2 |

- With no Expansion Hub, the Pinpoint goes in Control Hub I2C port 1.
- No Brushland laser rangefinders: ours are stored in analog mode, and I2C traffic in their first second after
  power-up switches them out of it.
- The bench drives the empty ports for real (its config calls them `fl`/`fr`/`bl`/`br`, `benchServo`/`benchServo2`,
  and `efl`/`efr`/`ebl`/`ebr`, `eServo`/`eServo2` on the Expansion Hub). `--ports-empty` says you've emptied them:
  without it the runner refuses to start while any hardware config on the hub names a device there; with it, in a
  terminal, it asks you to confirm. Either way, Bench: Photon Stock drives each motor port at 0.1 for 0.2 s and stops
  the run if one draws over 0.1 A. Servo ports can't be probed: check them by eye.

**Connect.** Join the Control Hub's Wi-Fi: the runner drives the hub through the dashboard over it, even with adb on a
cable. Connect adb to the Control Hub with Wi-Fi ADB (the runner's default, [Run it](../scripts/bench/README.md#run-it)
step 2) or a USB cable; on a cable, add `--serial <serial>` to every command, with the serial `adb devices` lists.

**Then:**

- Close every dashboard tab.
- `./gradlew --version` must work in that terminal: the block runs deploySloth itself. `--no-sloth` skips that, but then
  the verdict lists deploySloth as not run and can't pass.
- Don't start OpModes yourself during a run. The stock baseline must be the first OpMode after an app restart; the
  runner does that itself by switching hardware configs, which restarts the app, right before Bench: Photon Stock.

## Next runs, in order

Each from the repo root.

### 1. USB block (about 50 min)

```bash
python3 scripts/bench/run_bench.py --photon --usb-expansion --photon-i2c --ports-empty
```

In a terminal it asks twice at the start (ports empty; plug in the Pinpoint and OctoQuad) and once at the end (move
the Pinpoint to Control Hub I2C port 1). A short run says **PASSED THIS RUN, NOT YET SAFE** at best.

### 2. RS485 block (about 50 min)

Recable the Expansion Hub to RS485 only (no USB cable), address 2, or add `--expansion-address N`. The runner asks for
the Pinpoint back in Expansion Hub I2C port 1.

```bash
python3 scripts/bench/run_bench.py --photon --photon-rs485 --photon-i2c --ports-empty
```

On RS485, Photon speeds up only the Control Hub's writes: the Expansion Hub's stay on the SDK's path, and this run
checks they do.

### 3. Go run (6-7 h), only after 1 and 2 pass

Only this run, for the wiring the robot will use, can say SAFE. USB wiring (recable the Expansion Hub to USB only
first):

```bash
python3 scripts/bench/run_bench.py --photon --usb-expansion --photon-soak --photon-i2c --interactive --ports-empty
```

The RS485 go command is in the README's [run table](../scripts/bench/README.md#the-photon-block) (no `--interactive`:
the cable pulls are USB only).

**Check the soak rate first.** The go run's soak stops at 10 million writes per hub or after 4 hours, and its loss bound
needs 6,818,182 writes per hub: about 474 a second for 4 hours. The short run for the same wiring soaks for 60 s with
the same I2C readers: in its `summary.md`, **Write soak** gives `writes per hub`. Under about 28,400 there, that rate
can't reach the bound in 4 hours, so the go run would end NOT SAFE however well Photon behaves: record it and don't
start the go run. The only soak on a hub so far (USB, no I2C) did 30,510. On RS485 the Expansion Hub's writes go the
SDK's way at about 2.5 ms each, so expect that wiring to fall short.

The go run adds 200 STOP trials per condition, more lost-write trials and the long soak. `--interactive` needs a
terminal: it asks twice at the start and five times near the end, about 6 h in (four cable pulls, then the Pinpoint).
It waits at each prompt until you press Enter; for each pull, wait for `>>> PULL NOW` and do what it says.

## After each run

Results land in `bench-results/<timestamp>/` at the repo root, and the runner prints the path of its `summary.md` when
it finishes. That folder is gitignored, so the laptop that ran it has the only copy: keep it.

The verdict ends the Photon section of `summary.md`: **SAFE TO TRY ON THE ROBOT**, **PASSED THIS RUN, NOT YET SAFE**
(it lists what the go run still needs) or **NOT SAFE YET** (it lists the blockers).

When something fails:

- `Run stopped early: ...` at the top, or `FAILED: ...` under the photon heading, means the runner gave up there; the
  message says why (a port drawing current, an I2C device not answering at INIT names its port, an Expansion Hub on both
  USB and RS485, ...). Fix it and rerun.
- Each blocker matches a `FAIL` line in the test sections above it, followed by what that FAIL means.
- Each step's logcat is in `logs/<step>.log`, its raw results in `raw/<step>.json`, the runner's own log in `run.log`.
- `the hub's installed Photon was built from the photon sources in this checkout` failing: full install from this
  checkout and rerun.
- `the loss bound reaches the target` failing with `lost 0`: the soak ran out of time, not a lost write.
- `RESTORE FAILED`: see [What it changes on the hub](../scripts/bench/README.md#what-it-changes-on-the-hub).
- Don't loosen a check to make it pass. A real Photon failure needs a fix in `architecture/hardware/photon/`, the
  offline suite, a full install, and every run here again from step 1.

Record every run:

- a row in the table under [Where it stands](#where-it-stands): the date, the hash from the summary's `source: git`
  line, the wiring, the run, the verdict, and the blockers or anything odd;
- the stock, off and on8 rows of the summary's **Write block per loop** table, dated and with the wiring, in the Photon
  section of [`docs/loop-time-findings.md`](loop-time-findings.md#photon).

## After SAFE

Photon may then go on the robot, for the wiring that said SAFE only: annotate the OpMode `@Photon` and call
`PhotonCore.enable()`. The rules are in the Photon paragraph of [CLAUDE.md](../CLAUDE.md#framework-layout). Not before:
PASSED or NOT SAFE means no Photon on the robot.

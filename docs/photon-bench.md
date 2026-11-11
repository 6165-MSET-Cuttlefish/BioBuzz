# Photon bench status

Photon (`architecture/hardware/photon/`) is the team's fork of SolversLib Photon. It sends motor and servo writes
without waiting for each one's ACK (the hub's acknowledgement), which on the bench cut a loop with 12 changing writes
from 30.7 ms to 16.0 ms. No robot OpMode uses it yet, and none may until the hub bench's Photon block says **SAFE TO
TRY ON THE ROBOT** for the robot's wiring. This page is where that stands, how to run the rest, and what the one hub
run of the fork measured. How the block works and what each check proves:
[`scripts/bench/README.md`, The Photon block](../scripts/bench/README.md#the-photon-block).

## Why a fork

Stock SolversLib Photon 0.3.6 is unsafe on our hubs. Never put that jar, or the SolversLib Photon dependency
(`org.solverslib:photon`), back on the robot:

- A lost ACK leaks an in-flight slot until the app restarts; once enough have leaked, the next write spins forever.
- It never retransmits, so a lost `setPower(0)` leaves a wheel running.
- It takes over every OpMode's hubs, and with a USB Expansion Hub it registers the Control Hub's replacement with the
  wrong USB device, so the Control Hub's replies go to an orphaned hub object, even with Photon disabled.
- After a USB detach it keeps writing to the stale device and the OpMode hangs.
- Its writes skip STOP's gate (once STOP is pressed the SDK refuses commands that could move hardware,
  `OpModeManagerImpl.shouldPreventDangerousHardwareAccess()`) for the rest of that loop and through `stop()`, up to
  about 1 s.
- It acknowledges each motor and servo write itself right after the USB write (`PhotonCore`,
  `shouldAckImmediately`), so the caller never sees the hub's real reply: a write the hub refuses with a NACK still
  looks done.
- While enabled, it skips the SDK's network transmission lock (one per USB link, held by `LynxRespondable.send()`
  from sending a command until the hub answers) for every motor and servo write, even one it can't parallelize: an
  RS485 Expansion Hub's writes take the SDK's path without that lock
  (`PhotonLynxModule.acquireNetworkTransmissionLock`).
- When it swaps the hubs at INIT it re-points every I2C device at the replacement hubs by reflection inside
  `catch (Exception ignored)` (`PhotonCore`): a device it fails to re-point stays on the replaced hub object,
  silently.

The last three are read from the 0.3.6 source (`org.solverslib:photon:0.3.6`, package
`com.seattlesolvers.solverslib.photon`).

**Spotting the misrouting in logcat.** At INIT, tag `LynxUsb` logs `removeConfiguredModule(): mod#=173 wasn't there`
(173 is the Control Hub's module address; seen on 2026-10-28 with stock Photon and a USB Expansion Hub). After that, a
Control Hub command that times out logs `Marking module #173 as unresponsive until we receive some data back` (tag
`LynxModule`) with no `REV Hub #173 has reconnected` after it, and every later Control Hub command is given up at
once: the SDK clears the mark only when a datagram reaches the hub object it marked
(`LynxModule.noteDatagramReceived`), and the Control Hub's datagrams now reach the orphan. That part is read from the
SDK 12 source (`LynxRespondable.awaitAndRetransmit`); no Control Hub command timed out after the swap on 2026-10-28.

## Where it stands

| date | Photon at | wiring | run | verdict | notes |
|---|---|---|---|---|---|
| 2026-10-28 | stock SolversLib Photon 0.3.6 | USB | speed only, earlier bench | none | write block 25.9 to 6.0 ms p50 at 8 parallel, loop 31.8 to 11.5 ms; showed the misrouted replies |
| 2026-10-29 | `c825493` | USB | short, no I2C | NOT SAFE YET | 3 problems, all fixed since (below); write block 24.96 to 10.84 ms p50, loop 30.7 to 16.0 ms; other timings [below](#how-the-fork-behaved-on-a-hub-2026-10-29) |

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
start the go run. The only soak on a hub so far (2026-10-29, USB, no I2C) did 30,510 in 60.01 s, about 508 a second
per hub, 7% above what the bound needs: the bound in about 3.7 h, 16 minutes inside the cap. The `--photon-i2c`
readers keep running through the soak, and on RS485 the Expansion Hub's writes go the SDK's way at about 2.5 ms each,
so expect both go runs to fall short.

Whether to raise `PHOTON_SOAK_MAX_S` or re-derive the loss target from a measured loop rate is not decided. Settle it
before the first go run, not after one fails. Where the numbers come from (`scripts/bench/run_bench.py`):

- **Loss target:** `PHOTON_SOAK_LOSS_TARGET` = 4.4e-7 lost writes per write (line 1388), at most 0.1 lost writes per
  match. It assumes about 6 changing writes a loop on the Control Hub at 150 Hz for 150 s, 1.35e5 parallel writes a
  match, or 2.25e5 with a USB Expansion Hub (0.1 / 2.25e5 = 4.4e-7). That is a design estimate, not a measurement: the
  fork's bench loop with 12 changing writes ran at 16 ms, about 62 Hz
  ([`loop-time-findings.md`](loop-time-findings.md#photon)). A write lost anyway through `EnhancedMotor` or
  `EnhancedServo` goes out again once 250 ms have passed since its last send (`WriteCache.RESEND_NANOS`).
- **Writes needed:** with zero losses in n writes the 95% upper bound on the loss rate is 3/n (the rule of three,
  `photon_soak_checks`), so the bound needs ceil(3 / 4.4e-7) = 6,818,182 writes per hub.
- **Caps:** `PHOTON_SOAK_MAX_S` = 4 h and `PHOTON_SOAK_WRITES_PER_HUB` = 1e7 (lines 105-106), whichever comes first.
  At 508 writes a second, 1e7 takes about 5.5 h, so the 4 h cap ends the soak.

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

A USB-connected Expansion Hub is legal in BIOBUZZ (R707; the rule reading is in
[`game-findings.md`](game-findings.md#usb-r707-r708)). Without Photon it gains nothing: a changed write cost about
2.6 ms on a USB Expansion Hub (2026-10-28 and 2026-10-29, [`loop-time-findings.md`](loop-time-findings.md#photon))
against about 2.5 ms over RS485
([`loop-time-findings.md`](loop-time-findings.md#estimated-per-loop-costs-of-the-planned-hardware)). Recable it to USB
only when Photon goes on the robot for that wiring.

## How the fork behaved on a hub (2026-10-29)

The write-block and loop numbers are in [`loop-time-findings.md`](loop-time-findings.md#photon); these are the rest.
Measured on 2026-10-29: Control Hub plus an Expansion Hub on USB, both firmware 1.8.2, SDK 12.0.0, nothing on the
ports, no I2C devices, 4 motors and 2 servos changing on each hub every loop, Photon at `c825493`. Photon has changed
since (above), so rerun before relying on any of these.

- **Parallel limit:** write block p50 for both hubs was 10.96 ms at 2 parallel, 10.66 at 4 and 10.84 at 8, loop period
  about 16 ms at each: with 6 writes per hub per loop, more than 2 in flight gained nothing. Installed but disabled:
  22.95 ms against 24.96 stock, so no cost. With a second thread doing a Control Hub bulk read every 10 ms: 25.01 ms
  disabled, 10.52 at 8 parallel.
- **Lost writes** (ACK swallowed on purpose, 12 trials over both hubs): retransmitted at 101-106 and 203-210 ms, given
  up at 253-296 ms; the write call returned in 0.58-1.13 ms and the next identical write reached the hub. 104 more per
  hub with 0-3 other writes in flight were all handled. With the limit (4) full of lost writes the next write still
  went out (calls up to 6.7 ms on the Control Hub, 8.1 ms on the Expansion Hub), and all four were given up at about
  264 ms.
- **Write storm:** 3000 writes at a limit of 1 stalled the writer 6.8 ms at most.
- **Hub reset** (a raw fail-safe): the slowest output was back after 194.7 ms Photon off and 198.8 ms at 8 parallel on
  the Control Hub, 197.2 and 189.7 ms on the Expansion Hub. Each hub flagged the reset in its ACK and the SDK dropped
  its caches at once, so the cache-expiry path wasn't exercised.
- **`WriteCache` after `failSafe()`, Photon on** (Bench: Write Recovery Photon, one trial): the wrapped servo was back
  after 1.9 ms, the wrapped motor after 232.8 ms. Without Photon (Bench: Write Recovery, one trial each) the wrapped
  motor and servo came back in the same loop, 1.9 ms after `failSafe()` on 2026-10-09 and 21.5 ms after on 2026-10-24.
  `WriteCache` sends an unchanged value again only once 250 ms have passed since its last send
  (`architecture/hardware/WriteCache.java`), so up to 250 ms plus a loop is expected with or without Photon
  ([`hub-bench-findings.md`](hub-bench-findings.md#design-around-these)); the motor lagging the servo is not.
- **STOP and crashes:** after an iterative STOP, the hub's fail-safe was seen 0-37 ms after STOP's gate rose, Photon
  off or at 8 parallel. After a crash with 4 parallel writes in flight, none of them ever acknowledged, it was seen
  539-542 ms after the gate; every output was off, and the next INIT started with nothing in flight. The bench's
  sampler reads every channel of a hub every 8-12 ms (Control Hub) or 14-24 ms (Expansion Hub), which limits how
  finely these are timed.

## Still unknown

- **Whether Photon delays a motor's recovery after `failSafe()`.** Bench: Write Recovery writes the wrapped motor and
  servo in the same loop from START, yet with Photon on the motor came back 231 ms after the servo. Test: run Bench:
  Write Recovery and Bench: Write Recovery Photon several times each with the current fork, logging per loop whether
  each wrapped write went to the hub or was skipped by `WriteCache`.
- **Whether a crash takes about 540 ms to reach the hub's fail-safe without Photon too.** Test: Bench: Photon Crash
  with Photon left off, same sampler. It always enables Photon at 8 parallel today, so it needs a Photon-off option
  first.
- **Whether the go run's soak can reach the loss bound** with `--photon-i2c`, and on RS485. Test: the short run's
  **Write soak** rate for that wiring (see [Check the soak rate first](#3-go-run-6-7-h-only-after-1-and-2-pass)).

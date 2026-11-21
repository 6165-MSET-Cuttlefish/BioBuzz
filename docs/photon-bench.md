# Photon bench status

Photon (`architecture/hardware/photon/`) is the team's fork of SolversLib Photon. It sends motor and servo writes
without waiting for each one's ACK (the hub's acknowledgement), which on the bench cut a loop with 12 changing writes
from 31.4 ms to 13.8 ms (2026-11-17). No robot OpMode uses it yet, and none may until the hub bench's Photon block
says **SAFE TO TRY ON THE ROBOT** for the robot's wiring. This page is where that stands, how to run the rest, and
what the fork measured on hubs. How the block works and what each check proves:
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
| 2026-11-11 | `709f1cf` | USB | short, with I2C | NOT SAFE YET | 3 failed checks: 2 bench bugs, fixed since, and 1 unexplained read-back (below); write block 22.62 to 7.75 ms p50; soak 26,070 writes per hub in 60 s |
| 2026-11-17 | the 2026-11-14 bench build, not on main (Photon sources the same as `709f1cf`'s and main's) | USB | short, with I2C | NOT SAFE YET | 2 blockers, one failed trial each (below); write block 24.19 to 7.82 ms p50, loop 31.38 to 13.76 ms; soak 24,138 writes per hub in 60 s |
| 2026-11-20 | the bench build `92016c1`, not on main (Photon sources `6aea840d`, the same as main's) | Control Hub alone, then USB | STOP race only, twice | none: this run gives no verdict | Photon on, enable cache stale: 0 violations in 6 and 12 trials; Photon off (the SDK's path): FAIL, violations in 4 of 6 and 10 of 12; the 20 ms controls clean ([below](#the-2026-11-20-stop-race)) |
| 2026-11-20 | the bench build `92016c1`, not on main (Photon sources `6aea840d`) | USB | short, with I2C, 25 lost writes per depth (`2026-11-20_235008`) | NOT SAFE YET | 240 PASS and 36 FAIL lines (18 failed checks, each listed twice); the Control Hub stopped answering for about 1.5 to 2 min mid-run and 15 of the 18 ran inside that, so they prove nothing either way; blocker 1 recurred at 22.44 ms ([below](#the-2026-11-20-usb-block)) |

### The 2026-11-17 blockers

The run and its hardware: [`hub-bench-findings.md`, Setup](hub-bench-findings.md#setup). Results are from the 16:58
run, on the laptop that ran it. Blocker 1 isn't explained yet. Blocker 2's cause was reproduced on 2026-11-20: it is
the SDK's own path, not Photon's, but as coded it still counts as a blocker in the verdict
([The 2026-11-20 STOP race](#the-2026-11-20-stop-race)). The bench changes behind both were made the same evening, on
the bench build only, not on main ([Next runs, step 0](#0-two-bench-changes-made-on-the-bench-build)). The USB block
that uses them ran on 2026-11-20 and settled neither: blocker 1 recurred, and the Control Hub went silent mid-run
([The 2026-11-20 USB block](#the-2026-11-20-usb-block)).

**1. Lost writes at depth, Control Hub: 1 of 104 trials failed on time, not on handling.**

- Measured (`raw/photon_faults.json`, the depth-2 trial on motor `bl`): the lost write was handled correctly. It was
  sent again at 105.9 and 207.1 ms and given up at 285.7 ms, its call took 0.84 ms, the next write to it landed,
  nothing was left in flight, and the counters showed only this give-up and its one fallback to the SDK's path. What
  failed: while it waited, the bench wrote the other 5 Control Hub channels 225 times, and one of those calls took
  22.36 ms, over the bench's 20 ms limit
  (`PHOTON_WRITE_CALL_LIMIT_MS`, `scripts/bench/run_bench.py:2924`). The next slowest was 4.75 ms, p99 3.49.
- Measured, all 208 trials: each trial's slowest neighbouring write was 2.63 to 11.39 ms on the Control Hub (p50
  4.67) apart from this one, and 3.99 to 10.10 ms on the Expansion Hub. So 1 call in about 26,000 on the Control Hub
  (42,500 across both hubs) went over 20 ms.
- Not the 2026-11-11 tap bug (measured): the tap swallowed exactly one message, sent 3 times under the same number,
  and Photon gave up on exactly one write.
- Measured in logcat: right after the give-up came the SDK's `Control Hub is currently unresponsive` warning, a
  telemetry warning and a sound. The same lines came in several other Control Hub trials whose writes all stayed
  under 11.4 ms, so they don't explain it alone.
- From the code: when Photon gives up on a write, it marks the hub not responding and also forgets the SDK's cached
  motor and servo values (`PhotonLynxModule.abandon`, `forgetLastKnown()`). The SDK's own 250 ms timeout only marks the
  hub (SDK 12 `LynxRespondable.awaitAndRetransmit`). So after a give-up, each motor's next `setPower` first asks the hub
  for the motor's mode and then sends a motor enable, both on the SDK's path, each waiting for its ACK
  (`LynxDcMotorController.internalSetMotorPower`, `internalGetPublicMotorMode`). The SDK alone would have blocked the
  lost write's own call for the whole 250 ms instead.
- Inferred, the likeliest reading (about 45%): the slow call was one of those SDK-path round trips after the give-up,
  slowed by a late ACK or a stalled thread. A plain Photon write takes about 0.5 ms (p50 0.53), so the few 2 to 5 ms
  writes in each trial are probably those round trips too, and the 20 ms limit is in effect judging the SDK's path. Less
  likely: a random stall of any call on the hub (about 25%), the call doing the give-up itself (about 20%), or a limit
  set too tight (about 10%: it came in with the bench with no reason written down, but it is about twice the worst
  normal value).
- The give-up-call reading isn't ruled out: 7 ms after the give-up, logcat shows only a mode-query reply (`0x9009`)
  on the SDK's receive thread that the SDK couldn't match to a command, and such unmatched replies appear 26 times in
  that trial set's log, in Expansion Hub trials too. No reading is proven: the bench that ran recorded neither when
  each neighbouring write started nor which path it took. The bench build records both now (step 0).
- On the robot, if real (inferred): only after an ACK is really lost, one loop stalls by about 15 to 35 ms while every
  motor on that hub pays two SDK round trips. Nothing is lost or left on (seen on the bench's RUN_WITHOUT_ENCODER
  motors; see the next point for RUN_USING_ENCODER).
- A second risk follows from the code and hasn't been seen on a hub: if that mode query is refused while the hub is
  marked not responding, the SDK returns a placeholder RUN_WITHOUT_ENCODER (`internalGetPublicMotorMode`), so a motor in
  RUN_USING_ENCODER, such as a flywheel, would get a constant power instead of a velocity target, and the SDK's cache
  can then skip an unchanged `setPower` after it. What the hub does with a constant power in velocity mode isn't known.
  Main's bench can't see it: its motors run without encoders (`opmodes/test/bench/PhotonRig.java`). The bench build's
  RUN_USING_ENCODER trials look for it (step 0); on 2026-11-20 they didn't see it in 25 of 25 trials per hub, with
  empty ports, so registers only ([below](#the-2026-11-20-usb-block)).
- Coverage gap (measured): depths 0 to 7 were asked for, but at most 5 writes on the Control Hub and 4 on the Expansion
  Hub were really in flight before a lost write (4 and 3 on 2026-11-20, `summary.md:89-90`).

**2. STOP, iterative, Photon off: 1 of 40 trials switched a motor back on after STOP's fail-safe.**

- Measured (`raw/photon_stop_iter_off_028.json` and that trial's logcat): STOP's gate rose 46.5 ms into a 50 ms loop.
  Photon was disabled (all its counters 0), so every command took the SDK's path on Photon's hub objects. The Expansion
  Hub's fail-safe arrived: `efr`, `ebl`, `ebr` and both servos read disabled. But `efl` read enabled at raw −6553 (this
  loop's 0.2 power; the other three still held the last loop's −8191) in every sampler sweep from 30 ms after the gate,
  and in `stop()`'s own read, until `stop()` set it to 0 about 151 to 166 ms after the gate. So it stayed at 20%
  until 151 to 166 ms after the gate, about 121 to 174 ms after the fail-safe reached the other channels (in the
  sweep from −7.8 to 30.1 ms). Logcat has no warning.
- No other STOP trial had it: none of the other 159 on this date (iterative and linear, Photon on and off), none on
  2026-10-29 or 2026-11-11.
- From the code (SDK 12): `setPower` sends the power command, then a motor enable whenever the cached enable has
  expired, 500 ms after it was last set (`LynxDcMotorController.internalSetMotorPower`, `LastKnown`). A motor enable
  isn't a dangerous command, so STOP's gate lets it through
  ([`sdk-hub-findings.md`](sdk-hub-findings.md#opmode-lifecycle)). STOP raises the gate, then fail-safes each hub
  (`OpModeManagerImpl.callActiveOpModeStop`), and the fail-safe waits for the hub's link lock, which `efl`'s power write
  held until its ACK.
- Inferred, by far the likeliest now: the fail-safe went out between `efl`'s power write and its enable, and the
  enable switched `efl` back on at the power still in its register. The 2026-11-20 race reproduced exactly that on
  demand, 14 times in 14 ([below](#the-2026-11-20-stop-race)), and 11 times in 11 more in that night's USB block.
  The sampler's reads on the same link widen the window. The two other readings (the fail-safe made the SDK forget
  the cached enable; the firmware left `efl` enabled) weren't tested and are no longer needed to explain it.
- Nothing in that path is Photon's: with Photon off, its hub objects call the SDK's own methods. So robot code without
  Photon is exposed the same way (read from the SDK 12 source; measured only on Photon's hub objects with Photon off,
  not with the hubs never swapped). What that means for robot code:
  [`sdk-hub-findings.md`](sdk-hub-findings.md#opmode-lifecycle). Photon on had 0 of 40 here, which shows nothing
  against 1 in 40 (one-sided Fisher p 0.5); on 2026-11-20 it refused the write in 30 of 30 stale trials over three
  runs, but only in the one case the bench sets up, so don't count Photon as protection yet.
- The Expansion Hub is a little more exposed (measured 2026-11-20): from the power frame to the enable is about 4 ms
  there against about 3 ms on the Control Hub.
- On the robot (inferred): a motor whose last write is in flight when STOP comes, with its enable due, keeps its last
  power until something sets it to 0: our modules' `stop()` in `EnhancedOpMode`'s stop pass, or else the SDK's
  `$Stop$Robot$` after `stop()` returns, up to 0.9 s later. So keep zeroing every motor in `stop()` with
  `EnhancedMotor.stop()` ([`CLAUDE.md`](../CLAUDE.md), Module pattern).
- Why the other checks missed it (from the code): Bench: Photon Stop Race enables the motor 20 ms before the write it
  races (`opmodes/test/bench/PhotonStopRaceBench.java:44-46`), so no enable follows that write, and its 6 of 6 passes
  can't see this case. The sampler counts only sweeps after its first all-safe one, so it can't see a motor switched
  back on before then; only `stop()`'s read caught this one. The summary's "fail-safe seen 1-166 ms" is this trial's
  zeroing, not a late fail-safe: without it the fail-safe was seen 1 to 28 ms after the gate on the Expansion Hub.
  The bench build's stale-cache trials cover this case now (below).

### The 2026-11-20 STOP race

Two runs of Bench: Photon Stop Race on the bench build, made to settle blocker 2.

- **What it proves.** On the SDK's path, a motor whose enable is due is switched back on whenever STOP's fail-safe
  reaches the wire between its power write and that enable: 14 of 14 such trials over both runs, both hubs (measured).
  That is the SDK's behaviour, not Photon's. With Photon on, Photon refused its own power write and the SDK never sent
  the enable: 0 violations in 18 of 18 stale trials (measured), but only for the one moment the bench raises the gate.
- **What it doesn't prove.** That Photon closes the race under a real STOP. The gate can rise at two other moments
  that weren't tested, and Photon doesn't cover velocity writes or writes that fall back to the SDK's path (below).
  Nor that stock SDK 12 with the hubs never swapped does the same: that rests on the SDK source.
- **What it changes.** Blocker 2's cause is known and isn't Photon's. As coded it still counts as a blocker, because
  the Photon-off stale line fails whenever a trial hits the window; whether it should is still undecided
  ([Still unknown](#still-unknown)). Photon stays off the robot: blocker 1, steps 1 to 3 and that decision are open.

**The runs** (measured 2026-11-20, SDK 12.0.0, Control Hub firmware 1.8.2 by the probe, over the USB cable, every
motor, servo and sensor port empty; folders on the laptop that ran them):

- `bench-results/2026-11-20_162756`: `python3 scripts/bench/run_bench.py --usb --photon-stop-race-only --ports-empty`,
  the Control Hub alone, 18 trials.
- `bench-results/2026-11-20_164830`: the same with `--usb-expansion`, the Expansion Hub on its own USB link at address
  2, 36 trials. The summary's probe line "Expansion Hub at address 2: not in the hardware map, so not answering" is an
  artifact of the probe config, which puts that hub on the Control Hub's RS485 link; the race's own config put it on
  its USB link, and its traces carry its own ACKs (measured).

| run | check | trials | judged | violations |
|---|---|---|---|---|
| 1 | enable 20 ms before, Photon off | 3 | 3 | 0, PASS |
| 1 | enable 20 ms before, Photon on | 3 | 3 | 0, PASS |
| 1 | enable cache stale, Photon off | 6 | 4 (2 had the enable on the wire first) | 4, FAIL |
| 1 | enable cache stale, Photon on | 6 | 6 | 0, PASS |
| 1 | installed Photon, port guard (4 ports), logs (18) | | | PASS |
| 2 | enable 20 ms before, Photon off | 6 | 6 | 0, PASS |
| 2 | enable 20 ms before, Photon on | 6 | 6 | 0, PASS |
| 2 | enable cache stale, Photon off | 12 | 10 (2 had the enable on the wire first) | 10 (Control Hub 5 of 6, Expansion Hub 5 of 6), FAIL |
| 2 | enable cache stale, Photon on | 12 | 12 | 0, PASS |
| 2 | installed Photon, port guard (8 ports), logs (36) | | | PASS |

**How a trial works** (from `opmodes/test/bench/PhotonStopRaceBench.java` and the runner, on the bench build): one
OpMode run on motor port 0 of the raced hub, RUN_WITHOUT_ENCODER. It sets 0.2, disables and re-enables the motor,
waits 20 ms (control) or 600 ms (stale: the SDK's cached enable expires after 500 ms), then calls `setPower(0.25)`. A
tap on the hub's USB device raises STOP's gate by reflection the moment that power frame goes out, then a bench thread
calls the hub's fail-safe, the raced hub first. There is no Driver Station STOP. The bench reads the hub's own enable
and power registers at about 30, 80, 150 and 300 ms; a violation is a read with the motor enabled and the register
over 1% (327). The register holds −8191 (0.25 on a reversed goBILDA 5202) through the fail-safe in every trial, passing
ones included (measured), so the enable decides.

**Photon off, one stale Control Hub trial** (run 1, measured, ms after the write call): mode query 0.7, power frame
3.6, gate 3.9, fail-safe called 4.9, power ACK 5.4, fail-safe frame 6.7 (it waited for the link lock the power write
held), fail-safe ACK 8.8, then the enable at 10.8 from the OpMode thread, ACKed at 12.4. The Expansion Hub has the same
shape, slower: power ACK 6.7 to 10.2, fail-safe 7.5 to 11.5, enable 12.2 to 16.3 ms.

- The window (measured): a power round trip took 1.55 to 1.94 ms on the Control Hub and 2.64 to 4.90 ms on the
  Expansion Hub. With nothing in its way, the enable left 0.95 to 1.35 ms after the power ACK, so from power frame to
  enable is 2.9 to 3.2 ms on the Control Hub and about 3.9 ms on the Expansion Hub. A fail-safe that reaches the wire
  in that gap goes out before the enable.
- Every trial whose fail-safe reached the wire before the enable violated (14 of 14); every trial whose enable went
  first stayed off (4 of 4). That split is close to the check's own definition. What it adds is that the hub really
  re-energized: the bench reads the hub's registers, not the SDK's cache. Each violator read enabled at −8191 at about
  35, 85, 155 and 305 ms, and went off only when the bench zeroed its motors after the last read. That the fail-safe
  switched it off first is inferred: no read falls between the fail-safe and the enable.
- Judge by wire order, not by the runner's "queued first" figure (the stopper's call before the power ACK). That
  figure undercounts: 3 of the 6 trials not queued first violated too, and one queued first had its enable on the wire
  first (measured).

**Photon on** (measured): all 36 Photon-on trials, controls included, made one parallel write and one Photon
fail-safe after STOP. Every stale trial's logcat has one `CANCELLED_FOR_SAFETY`, and no enable followed the gate. The
fail-safe was queued first in 5 of 6 trials in run 1 and 3 of 12 in run 2 (Control Hub 0 of 6, Expansion Hub 3 of 6),
which doesn't matter on this branch. Why (read from `PhotonLynxModule.sendParallel`): after sending, Photon checks the
gate; if it is up, Photon refuses its own write with `CANCELLED_FOR_SAFETY`, so `setPower` throws before the SDK
sends the enable, and Photon then sends its own fail-safe.

What the Photon-on pass doesn't cover:

- The tap raises the gate after the USB write and before Photon's check, so every trial took the one branch that
  refuses. On a real STOP the gate can also rise before the send (Photon then falls back to the SDK's path;
  `gateFallbacks` was 0 in every trial, so untested) or after the check (Photon then acknowledges the write itself, the
  SDK sends the enable and STOP's fail-safe can get ahead of it: read from the code, a window probably well under 1 ms,
  not measured, untested). The tested case is probably the rarest of the three (inferred).
- No broken Photon was run as a negative control. The trace rule that fails on any enable after the gate does fire (it
  flagged every Photon-off enable), but judged by motor state alone a broken refusal would probably pass: its enable
  would go out before the bench's fail-safe, which runs 0.8 to 3 ms after the gate (inferred).
- Photon sends in parallel only RUN_WITHOUT_ENCODER `setPower` and servo pulse writes (`wantsParallel`). Velocity
  writes (RUN_USING_ENCODER, `setVelocity`) and any write that falls back to the SDK's path (slot timeout, a hub
  marked not responding) send the same power-then-enable pair with the stock window (read from code). None was raced.

What weakens both runs:

- The gate is raised by reflection and the fail-safe comes from a bench thread. A real STOP raises the gate and
  fail-safes each hub on one thread with nothing in between (read from SDK 12 `OpModeManagerImpl.callActiveOpModeStop`),
  so the bench's 0.8 to 3 ms between gate and fail-safe gives the enable more time to go first than a real STOP does
  (inferred).
- Only motor port 0, RUN_WITHOUT_ENCODER, nothing plugged in. 6 and 12 stale trials per Photon mode. RS485 wiring
  wasn't run.
- Otherwise clean (measured): no retransmits, no log problems, no trace overflow or incomplete trace. The dashboard's
  socket dropped at each config switch, the expected app restart.

### The 2026-11-20 USB block

Step 1 on the bench build `92016c1` (Photon sources `6aea840d`), results in `bench-results/2026-11-20_235008` on the
laptop that ran it: `python3 scripts/bench/run_bench.py --usb --photon --usb-expansion --photon-i2c --ports-empty
--photon-depth-reps 25`. Control Hub plus the Expansion Hub on its own USB link at address 2, both firmware 1.8.2;
motor and servo ports empty; OctoQuad in Control Hub I2C port 2, Pinpoint in Expansion Hub I2C port 1. About 54 min
of bench work. Times below are the laptop's clock unless they say hub time (hub time = laptop time + about 18:03:16,
the hub's clock not being set).

- **Verdict NOT SAFE YET.** `summary.md` has 240 PASS and 36 FAIL lines: 18 failed checks, each listed twice.
- **The Control Hub stopped answering for about 1.5 to 2 min**, in the middle of ordinary Photon writing, then
  answered again. 15 of the 18 failed checks ran inside that time, so this run neither passes nor fails them. What
  caused it and what ended it aren't known.
- **The STOP checks prove nothing this run**, by the runner's own rule: "a STOP negative control didn't register
  every time, so the STOP checks prove nothing" (`summary.md:557`). The once and linear-once controls broke only
  because the Control Hub was silent.
- **Blocker 1 recurred** (one call took 22.44 ms), and the summary missed it. **Blocker 2's race** failed on the SDK's
  path again and passed with Photon on.
- **Next:** three bench changes, then this block again ([Next runs](#before-step-1-again-three-bench-changes)).

**What failed, by cause** (most important first; line numbers are the checks list in `summary.md`):

| cause | failed checks | lines |
|---|---|---|
| the Control Hub silent (below) | 15: the once and linear-once negative controls and the once logs; Bench: Photon Routing h1, h3, h5 and s1 and Bench: Photon Stock h2 not running to the end; routing consecutive 1, 3 and 5 and before deploySloth; the crash's outputs-off and leftovers checks; consecutive OpModes | 120-122, 126-131, 133-134, 144-147 |
| late Control Hub replies in one STOP trial (below) | 1: the logs of STOP, iterative, Photon on8 (`photon_stop_iter_on_005`) | 110 |
| the Faults step's logcat cut short (below) | 1: the logs of Bench: Photon Faults | 102 |
| the SDK's enable race, blocker 2 (known) | 1: Photon off, enable cache stale | 105 |

**1. The Control Hub went silent** (measured unless marked):

- **Start.** About 380 ms into "phase writing" of the first once-control trial (`photon_stop_iter_once_000`), with
  Photon on at 8 parallel. The sampler's last good Control Hub sweep ended at 379.857 ms and its first failed
  one began at 379.863 ms (`raw/photon_stop_iter_once_000.json`): about 00:31:54 (hub time 18:35:10.42). The first
  retransmit is at hub time 18:35:10.518 (`logs/photon_stop_iter_once_000.log:14`), and the hub was marked
  unresponsive at 18:35:10.669 to 10.685 (lines 33-61).
- **The negative controls didn't start it.** The once control's late write goes out only after STOP, which came at
  18:35:13.178 (line 64), 2.76 s after the hub went quiet. The hub still answered after the last held control:
  `raw/photon_stop_iter_held_004.json` has no failed Control Hub sweep through 5322 ms, and once_000's sweeps up to
  379 ms read back right. The STOP bench has no I2C code, so no OctoQuad traffic is shown when it started.
- **During it** the Control Hub sent nothing back at any probe. The routing bench marks each hub not responding,
  sends a voltage read and waits 1 s for any reply (`PhotonRoutingBench.java:103-121`). In h1, h3, h5 and s1 the
  Control Hub never answered (no clear time, 0.0 V); the Expansion Hub cleared in 1.8 to 4.0 ms each time. No
  captured log has a reply from it. The logs don't cover all of it: each step change leaves a 1 to 3 s gap, and
  nothing was captured for 28 s around deploySloth.
- **End.** Between hub time 18:36:43.96, when s1 crashed (`logs/photon_routing_s1.log`), and 18:37:13.49, when s2 got
  its reply (`REV Hub #173 has reconnected`, `logs/photon_routing_s2.log:18`): 00:33:28 to 00:33:58. So it lasted
  about 93 to 123 s. The app wasn't restarted: pid 1321 and the same Control Hub object in h1, s1 and s2. The config
  switch and app restart came after s2 (`run.log:597`), and from 00:33:59 on everything passed again.
- **What ended it isn't known.** deploySloth ran inside that window (staged at hub time 18:36:56.04, processed at
  18:37:06.66), but no logcat was captured from 18:36:43.96 to 18:37:12.30. So neither deploySloth, a new INIT nor
  time alone is shown, and six INITs before it didn't bring the hub back.
- **Not the battery.** It didn't dip: the run's "lowest 12.86 V" is the Expansion Hub's own reading. The dashboard
  shows the highest of the hubs' voltages (slothboard's `FtcDashboard.java`, a `Math.max` over every hub's
  `getInputVoltage`), and the Control Hub reads about 0.13 V above the Expansion Hub every time (12.992 against
  12.865 V in s2, 13.302 against 13.171 V in r01). When the Control Hub's read returned 0, the Expansion Hub's
  normal value showed through: `power.jsonl` reads 12.987 V at 00:31:50, 12.86, 12.856 and 12.869 V while the
  Control Hub read 0, and 12.996 V at 00:33:56.
- **Not the adb drop.** Forwards were found gone only at 23:58:22 and 07:51:15 (`run.log`), and the app's pid was
  1321 from hub time 18:02 to 18:37.
- **The OctoQuad's chip ID 0x00 is a symptom.** Bench: Photon Stock h2's read went to the silent Control Hub and came
  back at once as the SDK's placeholder (`LynxI2cDeviceSynch: placeholder: readTimestamped`,
  `logs/photon_stock_h2.log:51-53, 73, 79`): it never reached the bus. Whether the OctoQuad was fine isn't shown: no
  OctoQuad read ran after the outage.
- **Not every run does it.** The same Photon sources ran the same block with the same OctoQuad and Pinpoint wiring
  on 2026-11-17, on the older bench build, and every held, once and linear-once trial passed with no problems there.
- **A possible early sign** (the link isn't shown): at hub time 18:18:34 to 18:18:36, in `photon_stop_iter_on_005`
  with no fault injected, the Control Hub answered more than 100 ms late twice (cause 2). Outside the Faults step, the
  only Control Hub retransmits in this run are in that trial (3) and once_000 (16). Outside their Faults steps, the
  2026-11-17 run and both 2026-11-20 STOP race runs have none.
- **Not known:** whether the silence was on the hub's side (its Lynx chip, or the Control Hub's internal serial path
  to it) or the app's (the reader thread or the Android serial port stalled): no thread dump or link trace was
  taken. Whether the OctoQuad or its wiring plays any electrical part: no evidence either way. How often it happens:
  1 of 2 runs with this sequence, so no rate.

**2. Late Control Hub replies in one STOP trial** (`photon_stop_iter_on_005`, `logs/photon_stop_iter_on_005.log:4-5,
15-18`). Each `unable to find originating` line refers to a message number retransmitted about 50 ms earlier.
Inferred: the hub's first reply finished the command, and the reply to the copy found nothing waiting. The trial
still had 0 violations, and its counters at STOP read 1 retransmit, 0 orphan datagrams and 0 number clashes. No other
STOP or race trial log in any results folder has that line. The runner fails on it on purpose, because an unmatched
reply can also mean misrouted replies ([Why a fork](#why-a-fork)). The same trial gives the iterative on8 "fail-safe
seen 1-78 ms": one slow Control Hub sampler sweep ran from 3.3 ms before the gate to 78.2 ms after
(`raw/photon_stop_iter_on_005.json`). Without it the range is 1-29 ms.

**3. The Faults step's logcat was cut short.** logd replaced 2,124 of the app's lines with 39 `chatty ... expire`
lines, and the first phase marker left is the 320th of 490, "lost write at depth 2 controlHub 18"
(`logs/photon_faults.log:111`). Logcat for the phases before it is missing, so a real warning there would not have
been seen. The `'init'` warning in that FAIL is a late reply inside a lost-write phase whose marker was dropped (line
110). The bench build's runner sets `logcat -G 8M` only when it connects and when it waits for the hub to come back
(`scripts/bench/run_bench.py:7422, 11750`), not when it finds a forward gone (`:1758-1768`). Inferred: the hub
rebooted between 23:54 and 23:58, while the runner waited for the I2C devices to be plugged in, and the reboot reset
the buffer. The hub clock went back from 18:03:50 at the end of `logs/config_02_*` to 18:01:39 at the start of
`logs/config_03_*`, the app's pid changed from 1697 to 1321, and `chatty` lines appear from config_03 on, none
before. Why the hub rebooted isn't known: a power cycle, a brownout while plugging in, or something else.

**4. The SDK's enable race, again** (`summary.md:432-435`, `raw/photon_race_stale_*.json`):

| check | trials | result |
|---|---|---|
| enable 20 ms before, Photon off / on | 6 / 6 | 0 violations each, PASS |
| enable cache stale, Photon off | 12, 11 judged | 11 violations (Control Hub 5 of 6, Expansion Hub 6 of 6), FAIL |
| enable cache stale, Photon on | 12 | 0 violations, every read disabled, PASS |

- In the 11 Photon-off violators the enable followed the fail-safe frame by 3.1 to 4.9 ms, 5.68 to 8.12 ms after the
  gate, and every read showed the motor enabled. The bench's fail-safe was called 0.64 to 2.99 ms after the gate in
  all 24 stale trials.
- `stale_off_controlHub_5` was rightly left unjudged: its enable went out at 4.70 ms, before the fail-safe at 14.30
  ms, and the motor read disabled at every read.
- Over the three 2026-11-20 runs, a fail-safe on the wire before the enable left the motor on in 25 of 25 trials, and
  an enable first left it off in 5 of 5.
- Each of the 18 Photon-on race logs has exactly one `CANCELLED_FOR_SAFETY`. None of the 36 race logs has a
  retransmit or an unmatched reply.

**Where each earlier blocker stands:**

| blocker or open question | after this run |
|---|---|
| 1. a write over 20 ms after a give-up | Recurred, still open. One Control Hub call to `fr` took 22.44 ms in "lost write velocity controlHub 16"; its power ACK came 11.58 ms after its frame (265.73 to 277.31 ms in `raw/photon_faults_trace_1_*.json`). The next slowest were 9.72 ms (Control Hub) and 11.11 ms (Expansion Hub), both in trials that weren't velocity trials. The summary's "0 over 20 ms" and "no slow call: no reading" (`summary.md:393-399`) are only because its slow-call reading skips the velocity trials (the bench build's `run_bench.py:4086-4093`), and the 20 ms limit is applied only to the depth trials (`:4260-4262`). Which reading, (a), (b) or (c), isn't settled. |
| depth coverage | Still short: at most 4 writes in flight on the Control Hub and 3 on the Expansion Hub. |
| RUN_USING_ENCODER after a give-up | Not seen: 25 of 25 trials per hub kept their target, with 1 Control Hub and 10 Expansion Hub velocity writes sent while the hub was marked not responding (`summary.md:93-94`). Empty ports, so only registers were judged. |
| forget the caches on a give-up or not | Still open: it waits on row 1. |
| 2. STOP, SDK path | The race failed on the SDK's path again (11 of 11 judged) and passed with Photon on (0 of 12). Ordinary STOP had 0 violations in 40 trials for each of iterative on8, iterative off, linear on8 and linear off (`summary.md:418-423`), but the STOP checks prove nothing this run (above). |
| should the stale-race FAIL block Photon | Still open; it failed again. |
| routing, crash, consecutive OpModes, negative controls | Not judged in effect: they failed only inside the outage. They need the rerun. |
| soak rate | Still short: 26,418 writes per hub in 60.04 s, 0 lost and 0 retransmitted (`summary.md:63`), under the 28,400 the go run needs. |

**Measured numbers.** The ones that compare with earlier runs are in the 2026-11-20 column of the table under
[What else the 2026-11-17 run measured](#what-else-the-2026-11-17-run-measured). Besides those:

- The 533 ms top of linear off's "fail-safe seen 2-533 ms" is the SDK's known LinearOpMode STOP-interrupt lock leak:
  5 leaks in 40 trials with Photon off, 0 in 40 with on8 (`summary.md:531`).
- I2C with Photon on: 58,024 byte-exact reads, p95 0.63 to 0.76 times Photon off's (`summary.md:227-228`).
- Two hubs at once at 8 parallel: adds up, 1.00 (hub by hub) and 1.01 (alternating) drained, 1.11 and 1.12 until the
  calls return (`summary.md:273`).
- Framework Loop with Photon ran to the end, but with `driverStationConnected` false, against true on 2026-11-17
  (`raw/framework_loop_photon0.json` and `photon8.json`). That plausibly explains its faster `dsEveryLoop` p50, 7.39
  and 7.12 ms (Photon 0 and 8) against 9.55 and 10.90 ms then.

### What else the 2026-11-17 run measured

Passed (measured, `summary.md`):

- The installed Photon matched the sources; no motor port drew over 0.3 A; the stock baseline ran on the SDK's own hub
  objects with every read-back exact.
- Every write lands: 0 wrong read-backs and 0 retransmits in every phase. Interleave off read 1200 of 1200 right, so
  the 2026-11-11 read-back didn't recur. Framework Loop with Photon ran to the end.
- Routing over 8 restarts in both swap orders, consecutive OpModes, a crash with writes in flight, deploySloth.
- I2C: 58,823 byte-exact reads. With Photon on, reads were faster: p95 0.59 to 0.69 times Photon off's.
- The other lost-write checks: 12 single lost writes, a full limit of 4, 10 + 10 superseded writes, a USB write that
  throws, and the Expansion Hub's 104 trials at depth.
- STOP: iterative on8 40 of 40, linear on8 and linear off 40 of 40 each, every negative control 5 of 5, STOP inside a
  write 6 of 6 off and on. The SDK's lock leaked at a linear STOP in 1 of 40 on8 trials and 4 of 40 off (Fisher p
  0.973).
- Write storm, two writer threads, hub reset, Write Recovery with Photon, the soak (0 lost of 24,138 per hub) and every
  SDK-warnings check.

Changed since 2026-10-29 (measured; the 2026-10-29 figures are [below](#how-the-fork-behaved-on-a-hub-2026-10-29)):

| | 2026-10-29 | 2026-11-17 | 2026-11-20 |
|---|---|---|---|
| write block, both hubs, p50: stock / 8 parallel | 24.96 / 10.84 ms | 24.19 / 7.82 ms | 23.52 / 7.17 ms |
| loop period, p50: stock / 8 parallel | 30.7 / 16.0 ms | 31.38 / 13.76 ms | 30.30 / 13.04 ms |
| write block at 2 / 4 / 8 parallel | 10.96 / 10.66 / 10.84 ms: all equal | 9.71 / 7.65 / 7.82 ms: 2 is slower | 8.86 / 7.19 / 7.17 ms: 2 is slower |
| single lost write: resent, resent, given up, call | 101-106, 203-210, 253-296, 0.58-1.13 ms | 101.4-106.3, 202.4-209.3, 258.1-301.5, 0.41-1.50 ms | 101.26-104.58, 202.52-209.66, 258.02-297.84, 0.39-1.11 ms |
| full limit of lost writes: next writes, given up | up to 6.7 ms CH, 8.1 ms EH; about 264 ms | up to 6.86 ms CH, 8.45 ms EH; 263 and 259 ms | up to 6.75 ms CH, 7.96 ms EH; 260.23 and 261.17 ms |
| write storm, longest stall | 6.8 ms | 7.36 ms | 5.27 ms, 0 slot timeouts |
| hub reset, slowest output, off / on8 | CH 194.7 / 198.8, EH 197.2 / 189.7 ms | CH 195.0 / 195.2, EH 200.1 / 199.8 ms | CH 191.5 / 197.9, EH 198.7 / 202.8 ms |
| crash with 4 writes in flight, fail-safe seen | 539-542 ms | 26.5 ms CH, 38.8 ms EH | 3 in flight; EH 530.8 ms, CH none (its sampler failed): ran while the Control Hub was silent, not comparable |
| iterative STOP, fail-safe seen | 0-37 ms (on and off) | 0-33 ms on8, 1-29 ms off (without the failed trial) | 1-78 ms on8 (1-29 without one slow sampler sweep), 0-30 ms off |
| `WriteCache` after `failSafe()` with Photon | servo 1.9 ms, motor 232.8 ms (1 trial) | both 271.9 ms at worst, together every time (7) | both 258.29 ms at worst (7); raw channels 38.70 ms |
| soak, writes per hub in 60 s | 30,510 (no I2C) | 24,138 (with I2C; 26,070 on 2026-11-11) | 26,418 (with I2C) |

The 2026-11-20 column is the USB block ([The 2026-11-20 USB block](#the-2026-11-20-usb-block)); every row but the
crash ran before or after the Control Hub's silent spell.

- The write path's own steps are now measured on a hub (`timing on8 profiled`, p50): the USB write 112.5 µs on the
  Control Hub and 507.5 µs on the Expansion Hub, the whole parallel send 332.5 and 692.5 µs.
- Writes to the two hubs add up, not overlap (the **Two hubs at once** line, one run, 300 loops per set): drained, 12
  together took 0.99 (hub by hub) and 1.00 (alternating) times 6 + 6 alone at 8 parallel; the lowest ratio in the
  table was 0.94, alternating at 2 parallel. On 2026-11-20 it read 1.00 and 1.01. The table and what it means for the
  wiring:
  [`wiring-findings.md`](wiring-findings.md#estimated-loop-period).

### Earlier runs

The 2026-11-11 failures (`bench-results/2026-11-11_205238` on the laptop that ran it):

1. **Lost writes at depth, Control Hub (7 of 104 trials): bench bug.** The fault tap swallowed by message number alone.
   Photon frees a number when it gives up on a write, and in those trials the hub's counter had wrapped (255 to 262
   writes since the lost one) and handed the same number to the next servo write, which the tap swallowed too. The tap
   now matches number, command and channel. The Expansion Hub wrote 202 to 213 times in the same window, so it never
   wrapped.
2. **Crash with writes in flight: bench bug.** The two swallowed writes per hub were given up 250 ms after they were
   sent, which marks the hub not responding until its next reply (2 ms later); the sampler's read in that gap failed
   and ended its thread 120 ms before the throw, so it never saw the fail-safe. The sampler now drops a sweep whose
   read fails, up to 5 in a row, and the runner fails any trial but the crash one that drops a sweep. The throw came
   366 ms after the lost writes went out, so they had already been given up: the crash had 1 write in flight that
   could still be acknowledged, not the 4 it meant to. What takes those 366 ms isn't known.
3. **Interleave off, 1 of 1200 read-backs: unexplained.** With Photon disabled (the SDK's own write path on Photon's
   hub objects), the Expansion Hub's motor port 2 read disabled right after the SDK's disable and enable, 67 ms after
   its power write, with the right power register and no logcat line in the phase. Stock (1200) and on8 (1200) had
   none. A wrong read-back now records a second read, to tell a wrong register from one wrong reply.

Passed for the first time on a hub: the I2C reads on both hubs (57,521 byte-exact reads), routing over 8 restarts in
both swap orders, deploySloth, 160 STOP trials with every negative control caught. The soak's 26,070 writes per hub in
60 s is under the 28,400 the go run needs ([Next runs](#next-runs-in-order)).

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

All of it ran on a hub on 2026-11-11 (the status table under [Where it stands](#where-it-stands)), and the tap,
sampler and re-read changes made after that run ran on 2026-11-17. Offline it passes the suite in
[`scripts/bench/photon-offline/`](../scripts/bench/photon-offline/README.md).

Run the mutants suite on an idle machine. On 2026-11-12, on a Mac at load average 40 to 140, the harness (108),
runner (414), dry-run and lock-leak (13) suites passed, but mutants said 13 of 16 twice, and 15 of 16 at load average
56. Every mutant failed all the checks named for it; the ones counted missed also failed other checks, not the same
ones each run, so the suite counted them missed. One of those was a timing check: `enabled: 6 writes return before
their 20 ms ACKs` took 42 and 58 ms there, against 1.0 ms in the harness run. Rerun the same day with the load average
between 4 and 11 on 10 cores, all five passed and mutants said 16 of 16
([`photon-offline-results.txt`](photon-offline-results.txt)).

## Before every run

**Install.** Put the stamped bench build on the hub with `scripts/bench/bare_hub.sh`, from the same checkout you run
the runner in, and again after any change to `architecture/hardware/photon/`. `--build --usb` (online) builds and
stamps it. `--install-only bench-results/<folder> --usb` checks the stamp, backs up the hub's app and configs,
installs, starts the app, waits for `INIT FINISH` and prints the command that puts the old app back. It is a full
install: deploySloth never reloads Photon's `@Pinned` classes, and the block's first check compares the hub's Photon
with your sources and fails on a mismatch. Run from a committed tree: the summary records the git hash.

**Wiring.** Unplug everything not in this table. Only the motor and servo ports are empty: the two I2C devices
stay plugged in.

| | Control Hub | Expansion Hub |
|---|---|---|
| power | 12 V battery | from the Control Hub's second XT30 port |
| motor ports 0-3, servo ports 0-1 | **empty** | **empty** |
| I2C (needed for `--photon-i2c`) | **OctoQuad in port 2** | **goBILDA Pinpoint in port 1** |
| link | | runs 1 and 3: USB cable from the Control Hub to its mini-USB, no RS485 cable; run 2: RS485 cable only, address 2 |

- `--ports-empty` is about the motor and servo ports only. With `--photon-i2c` the bench reads both I2C devices, and a
  missing one makes the bench OpMode throw: on 2026-11-20 a read that failed for another reason threw "the OctoQuad
  on controlHub I2C bus 2 read chip ID 0x00, not 0x51: it isn't answering. Plug it in there, or rerun without
  --photon-i2c". The runner asks for both at the start.
- With no Expansion Hub, the Pinpoint goes in Control Hub I2C port 1.
- No Brushland laser rangefinders: ours are stored in analog mode, and I2C traffic in their first second after
  power-up switches them out of it.
- The bench drives the empty ports for real (its config calls them `fl`/`fr`/`bl`/`br`, `benchServo`/`benchServo2`,
  and `efl`/`efr`/`ebl`/`ebr`, `eServo`/`eServo2` on the Expansion Hub). `--ports-empty` says you've emptied them:
  without it the runner refuses to start while any hardware config on the hub names a device there; with it, in a
  terminal, it asks you to confirm. Either way, Bench: Photon Stock drives each motor port at 0.1 for 0.2 s and stops
  the run if one draws over 0.3 A. Servo ports can't be probed: check them by eye.
- With `--ports-empty` the bench build also runs RUN_USING_ENCODER trials on motor ports 2-3 of each USB hub. With no
  encoder there a velocity target can wind a port up to full power, so a motor plugged in there could run at full
  power. `--ports-empty` is the guard; the bench also stops if either port draws over 0.3 A after a trial (the bench
  build's `scripts/bench/README.md`, The Photon block).
- An empty motor port reads current while driven, so the probe's limit sits above that, and a free-spinning motor can
  pass it. Two runs on 2026-11-11 with every bench port empty read the same at 0.1 power, to the milliamp: Control Hub
  ports 0-3 at 0.054, 0.003, 0.000 and 0.142 A, the USB Expansion Hub's at 0.003, 0.003, 0.000 and 0.014 A. The old
  0.1 A limit stopped both runs on Control Hub port 3. One Control Hub and one Expansion Hub, so another hub may read
  higher.

**Connect.** Over a USB cable to the Control Hub's USB-C port, as on 2026-11-17, add `--usb` to every command (the
commands below have it): adb and the dashboard both go over the cable, the Mac stays on its own network and doesn't
join the hub's Wi-Fi ([Over a USB cable](../scripts/bench/README.md#over-a-usb-cable)). `--serial <serial>` is needed
only when adb lists more than one USB device. Over Wi-Fi instead, leave out `--usb`, join the Control Hub's Wi-Fi and
connect adb with Wi-Fi ADB (the runner's default, [Run it](../scripts/bench/README.md#run-it) step 2).

**Then:**

- Close every dashboard tab.
- `./gradlew --version` must work in that terminal: the block runs deploySloth itself. `--no-sloth` skips that, but then
  the verdict lists deploySloth as not run and can't pass.
- Don't start OpModes yourself during a run. The stock baseline must be the first OpMode after an app restart; the
  runner does that itself by switching hardware configs, which restarts the app, right before Bench: Photon Stock.

## Next runs, in order

Each from the repo root.

### 0. Two bench changes, made on the bench build

Done on 2026-11-17, on the bench build only, not on main: main's runner has neither new flag. The changes passed the
offline suites in [`scripts/bench/photon-offline/`](../scripts/bench/photon-offline/README.md), and both bench builds
were rebuilt and re-stamped that night. Photon's own sources didn't change, so the Photon hash the summary checks is
still `6aea840d`. They were made because neither 2026-11-17 blocker can be settled from what the old bench records
([The 2026-11-17 blockers](#the-2026-11-17-blockers)).

What the bench build now does (from its code and its README):

- **Lost writes:** every timed write call gets a row: when it started, how long it took, its hub and role, its path
  (`parallel`, `fallback:<counter>`, `sdk`, or `mixed`) and whether a give-up happened during it. A trace on each
  hub's USB link records every frame and reply, so the runner can say which SDK commands each call sent (a mode query,
  a motor enable) and how long each waited for its reply. The runner reads from logcat which thread logged each
  give-up.
- **RUN_USING_ENCODER trials** (only with `--ports-empty`): motor ports 2-3 of each USB hub go to RUN_USING_ENCODER at
  0.2 and 0.25, a control read 1 s later, then a lost write; the moment Photon gives up, both are written, and their
  mode, target velocity, constant-power register and enable are read back.
- **STOP race:** besides the 3 trials with the enable written 20 ms before the raced write (unchanged, no trace), 6
  trials per hub and Photon mode with it written 600 ms before, so the SDK's enable cache has expired. Those trace the
  link from before the write until the fail-safe is done: the power write's ACK, the fail-safe frame and any motor
  enable after STOP's gate. The raced hub is fail-safed first.
- **Sampler:** a motor that is on again before the first all-safe sweep now counts as a STOP violation, and the STOP
  line says when the fail-safe reached each hub. Replayed on the 2026-11-17 results it flags the trial that failed
  and, as it should, the negative control that switches a motor back on on purpose; nothing else.

New runner flags:

- `--photon-stop-race-only`: the port probe, then only the STOP race, Photon on8 then off, 3 + 6 trials per hub and
  mode: 18 on a bare Control Hub, 36 with `--usb-expansion`. No Photon verdict. It needs `--ports-empty` and no other
  bench flag, and refuses `--photon-soak`, `--photon-extras`, `--photon-pulls-first`, `--photon-soak-last`,
  `--photon-i2c` and `--photon-depth-reps`.
- `--photon-depth-reps N` (1 to 64): lost writes per in-flight depth on each USB hub, and with `--ports-empty` the
  number of RUN_USING_ENCODER trials per hub. Default 13, or 25 with `--photon-soak`. Before, only `--photon-soak`
  set it.

New lines in `summary.md`:

- **Lost writes, timing reading** (record only). It reads (b) if a call over 20 ms was the call that gave up; "no
  reading" if such a call saw a give-up that no log line explains; else (a) if one was an SDK-path call whose reply
  came late; else (c) if any call passed 20 ms with its replies on time; else "no slow call".
- **Lost writes, velocity reading** (record only): (d) if a RUN_USING_ENCODER target went wrong after a give-up in k
  of n trials; "no RUN_USING_ENCODER motor went wrong in n trials"; or "not run" without `--ports-empty`.
- Judged: **every lost-write trial recorded each neighbouring write's start, length and path**. A FAIL here means the
  bench missed something (a row, a trace file, a trace past its 8192 frames, a `mixed` path), not that Photon did.
- Judged: **RUN_USING_ENCODER motors keep their velocity target through a give-up**, the velocity reading's twin.
  NOT JUDGED when no velocity write went out while the hub was marked not responding, or without `--ports-empty`.
- Judged, per Photon mode: **enable cache stale: the motor stays off when STOP lands inside its write**. On the SDK's
  path (Photon off) the SDK sends that enable today, so this line reads FAIL whenever a trial hits the window, and NOT
  JUDGED when none does; it can pass only once nothing sends an enable after STOP's gate. A FAIL there is the answer
  to blocker 2, not a bench fault, and it failed in all three 2026-11-20 runs. But it counts as a blocker in the
  verdict, so the go run's gate would refuse to start ([Still unknown](#still-unknown)).

What the answers mean:

- slow calls that are SDK round trips with a late reply: the hub or link is slow and Photon's forgetting the
  caches exposes it; fix Photon to keep the mode and enable caches on a give-up;
- slow calls that are always the call doing the give-up: move the give-up to Photon's sweeper thread;
- replies on time but a slow call, on any kind of call: the hub's thread scheduling; then write down a reason for
  the 20 ms limit or pick a new one, never just to make the run pass;
- a wrong velocity target after a give-up: a blocker for the robot, whatever the timing shows.

In the timing reading these are (a), (b) and (c); the last is the velocity reading's (d). For the STOP race, run on
2026-11-20: Photon off showed an enable after the gate and the motor on at 0.25 whenever the fail-safe reached the
wire before the enable, and the 20 ms trials stayed clean, as blocker 2's reading predicted
([The 2026-11-20 STOP race](#the-2026-11-20-stop-race)). `--photon-stop-trials 200` measures the rate, not the cause.

### Before step 1: put the new build on the hub and run the STOP race alone (about 5 min plus the install)

Done on 2026-11-20 ([results](#the-2026-11-20-stop-race)). Rerun it after any change to how Photon handles STOP.
From the bench build's checkout, over the cable:

```bash
scripts/bench/bare_hub.sh --install-only bench-results/<folder> --usb
python3 scripts/bench/run_bench.py --usb --photon-stop-race-only --ports-empty
```

The first installs the re-stamped build ([Install](#before-every-run)). The second is the race on a bare Control Hub
(about 2 min, 18 trials, one typed confirmation). Then plug in the Expansion Hub's mini-USB cable, no RS485 cable,
its 12 V from the Control Hub's second XT30 port, and run it again with the Expansion Hub (about 3 min, 36 trials):

```bash
python3 scripts/bench/run_bench.py --usb --photon-stop-race-only --usb-expansion --ports-empty
```

Motor ports 0-3 and servo ports 0-1 empty on both hubs, as for the block. Read the **enable cache stale** line for
each Photon mode.

### Before step 1 again: three bench changes

Not done yet. Step 1 ran on 2026-11-20 ([results](#the-2026-11-20-usb-block)) and has to run again: by the runner's
own rule its STOP checks prove nothing, and its routing, crash and consecutive-OpMode checks were never judged
against a working Control Hub. Don't run step 2 first: it runs the same STOP, routing and crash steps on the same
Control Hub, so if the outage comes back it voids that run's 52 minutes too. Make these changes on the bench build
first:

1. Set `logcat -G 8M` again whenever the runner finds a forward gone or the hub back (the bench build's
   `run_bench.py:1758-1768`), so the Faults log isn't cut again.
2. Keep one logcat capture running across step boundaries and through deploySloth, so a recovery can't land in a gap
   with no log.
3. Count the velocity trials in the slow-call reading (**Lost writes, timing reading**), or give them their own line.

If it's cheap, also take a thread dump (as `opmodes/test/bench/FreezeDumpBench.java` does) the first time the
Control Hub (module 173) is marked unresponsive outside the Faults step: that would tell the hub's side from the
app's.

Then rerun step 1 with the same command. If it is clean, go on to step 2. If the outage comes back, chase the Control
Hub or its link before step 2 or 3: the thread dump, the OctoQuad's cable or port swapped, the hub's firmware checked.

### 1. USB block (about 60 min)

Ran on 2026-11-20 (about 54 min of bench work, [results](#the-2026-11-20-usb-block)). Rerun it after the changes
above.

```bash
python3 scripts/bench/run_bench.py --usb --photon --usb-expansion --photon-i2c --ports-empty --photon-depth-reps 25
```

About 58 min at the default 13 lost writes per depth, 60 with the soak's 25 (the bench build's README; estimated from
the 2026-11-17 Faults step and the added trials). Motor ports 0-3 and servo ports 0-1 empty on both hubs. In a
terminal it asks twice at the start (ports empty; plug in the Pinpoint and OctoQuad) and once at the end (move the
Pinpoint to Control Hub I2C port 1). A short run says **PASSED THIS RUN, NOT YET SAFE** at best. Read the **Lost
writes, timing reading** and **velocity reading** lines first.

On USB wiring Bench: Photon ends with the two-hubs-at-once phase (about 3 min, no prompts). On 2026-11-17 and
2026-11-20 it measured that Photon's writes to the two hubs add up
([`wiring-findings.md`](wiring-findings.md#estimated-loop-period)).
It runs Photon off and at 8, 4 and 2 parallel, 6 changed writes on each hub alone, all 12 hub by hub and alternating
hubs, and 3 + 3, 5 + 1 and 1 + 5, 300 loops each, each timed until the calls return and until every write is acknowledged. The
**Two hubs at once** line in `summary.md` reads it at 8 parallel: adds up, in between or overlaps. It is a record,
not a check, and never changes the verdict; the phase's read-backs and in-flight counts are judged like every
other phase's.

### 2. RS485 block (about 52 min)

Only after a clean rerun of step 1 ([above](#before-step-1-again-three-bench-changes)). Recable the Expansion Hub
to RS485 only (no USB cable), address 2, or add `--expansion-address N`. The runner asks for the Pinpoint back in
Expansion Hub I2C port 1.

```bash
python3 scripts/bench/run_bench.py --usb --photon --photon-rs485 --photon-i2c --ports-empty
```

On RS485, Photon speeds up only the Control Hub's writes: the Expansion Hub's stay on the SDK's path, and this run
checks they do.

### 3. Go run (6-7 h), only after 1 and 2 pass

Only this run, for the wiring the robot will use, can say SAFE. USB wiring (recable the Expansion Hub to USB only
first):

```bash
python3 scripts/bench/run_bench.py --usb --photon --usb-expansion --photon-soak --photon-i2c --interactive --ports-empty
```

The RS485 go command is in the README's [run table](../scripts/bench/README.md#the-photon-block) (no `--interactive`:
the cable pulls are USB only).

**Check the soak rate first.** The go run's soak stops at 10 million writes per hub or after 4 hours, and its loss bound
needs 6,818,182 writes per hub: about 474 a second for 4 hours. The short run for the same wiring soaks for 60 s with
the same I2C readers: in its `summary.md`, **Write soak** gives `writes per hub`. Under about 28,400 there, that rate
can't reach the bound in 4 hours, so the go run would end NOT SAFE however well Photon behaves: record it and don't
start the go run. With the I2C readers it already falls short on USB: 26,070 on 2026-11-11, 24,138 in 60.03 s on
2026-11-17 (about 402 a second per hub, so the bound would take about 4.7 h) and 26,418 in 60.04 s on 2026-11-20
(about 440 a second, about 4.3 h). Only the 2026-10-29 soak, with no I2C
(30,510 in 60.01 s, about 508 a second), was fast enough. On RS485 the Expansion Hub's writes go the SDK's way at
about 2.5 ms each, so expect that go run to fall short too.

Whether to raise `PHOTON_SOAK_MAX_S` or re-derive the loss target from a measured loop rate is not decided. Settle it
before the first go run, not after one fails. Where the numbers come from (`scripts/bench/run_bench.py`):

- **Loss target:** `PHOTON_SOAK_LOSS_TARGET` = 4.4e-7 lost writes per write (line 2537), at most 0.1 lost writes per
  match. It assumes about 6 changing writes a loop on the Control Hub at 150 Hz for 150 s, 1.35e5 parallel writes a
  match, or 2.25e5 with a USB Expansion Hub (0.1 / 2.25e5 = 4.4e-7). That is a design estimate, not a measurement: the
  fork's bench loop with 12 changing writes ran at 16 ms, about 62 Hz
  ([`loop-time-findings.md`](loop-time-findings.md#photon)). A write lost anyway through `EnhancedMotor` or
  `EnhancedServo` goes out again once 250 ms have passed since its last send (`WriteCache.RESEND_NANOS`).
- **Writes needed:** with zero losses in n writes the 95% upper bound on the loss rate is 3/n (the rule of three,
  `photon_soak_checks`), so the bound needs ceil(3 / 4.4e-7) = 6,818,182 writes per hub.
- **Caps:** `PHOTON_SOAK_MAX_S` = 4 h and `PHOTON_SOAK_WRITES_PER_HUB` = 1e7 (lines 132-133), whichever comes first.
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
  section of [`docs/loop-time-findings.md`](loop-time-findings.md#photon);
- on USB wiring, the **Two hubs at once** line, dated, in
  [`wiring-findings.md`](wiring-findings.md#estimated-loop-period), and the wiring map's split rechecked if it ever
  reads anything but "adds up" (2026-11-17 and 2026-11-20 read adds up).

## After SAFE

Photon may then go on the robot, for the wiring that said SAFE only: annotate the OpMode `@Photon` and call
`PhotonCore.enable()`. The rules are in the Photon paragraph of [CLAUDE.md](../CLAUDE.md#framework-layout). Not before:
PASSED or NOT SAFE means no Photon on the robot.

A USB-connected Expansion Hub is legal in BIOBUZZ (R707; the rule reading is in
[`game-findings.md`](game-findings.md#usb-r707-r708)). Without Photon it gains nothing: a changed write cost about
2.6 ms on a USB Expansion Hub (2026-10-28 and 2026-10-29, [`loop-time-findings.md`](loop-time-findings.md#photon))
against about 2.5 ms over RS485
([`loop-time-findings.md`](loop-time-findings.md#per-command-costs)). Recable it to USB
only when Photon goes on the robot for that wiring.

## How the fork behaved on a hub (2026-10-29)

The write-block and loop numbers are in [`loop-time-findings.md`](loop-time-findings.md#photon); these are the rest.
What changed by 2026-11-17 is in [the table above](#what-else-the-2026-11-17-run-measured). Measured on 2026-10-29:
Control Hub plus an Expansion Hub on USB, both firmware 1.8.2, SDK 12.0.0, nothing on the ports, no I2C devices, 4
motors and 2 servos changing on each hub every loop, Photon at `c825493`. Photon has changed since (above), so rerun
before relying on any of these.

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
  ([`hub-bench-findings.md`](hub-bench-findings.md#design-around-these)); the motor lagging the servo is not. On
  2026-11-17 (7 fail-safes) the motor and servo came back together every time.
- **STOP and crashes:** after an iterative STOP, the hub's fail-safe was seen 0-37 ms after STOP's gate rose, Photon
  off or at 8 parallel. After a crash with 4 parallel writes in flight, none of them ever acknowledged, it was seen
  539-542 ms after the gate (26.5 and 38.8 ms on 2026-11-17); every output was off, and the next INIT started with
  nothing in flight. The bench's
  sampler reads every channel of a hub every 8-12 ms (Control Hub) or 14-24 ms (Expansion Hub), which limits how
  finely these are timed.

## Still unknown

- **What silenced the Control Hub for about 1.5 to 2 min on 2026-11-20** ([The 2026-11-20 USB
  block](#the-2026-11-20-usb-block)): the hub's side (its Lynx chip or its internal serial path) or the app's (the
  reader thread or the Android serial port); what ended it, and exactly when (the 28 s around deploySloth have no
  logcat); whether the Control Hub's late replies about 17 minutes earlier and the 11.58 ms power ACK in the Faults step
  were early signs of it; whether the OctoQuad or its wiring plays any electrical part; and how often it happens (1
  of 2 runs with this sequence). Test: the step 1 rerun with the bench changes
  [before it](#before-step-1-again-three-bench-changes), ideally with the thread dump.
- **Why the hub rebooted at about 23:54 to 23:58 on 2026-11-20**, while the runner waited for the I2C devices to be
  plugged in (inferred from its clock and the app's pid): a power cycle, a brownout while plugging in, or something
  else.
- **What made one Control Hub write take 22.36 ms (2026-11-17) and 22.44 ms (2026-11-20) after a give-up, and
  whether that is Photon's.** On 2026-11-20 it fell in a velocity trial, which the **Lost writes, timing reading**
  line skips, so the reading said "no slow call"; which of (a), (b) or (c) it is wasn't worked out. Test: the step 1
  rerun with velocity trials counted (bench change 3).
- **Whether a give-up can leave a RUN_USING_ENCODER motor on a constant power** (from the code;
  [The 2026-11-17 blockers](#the-2026-11-17-blockers)). Not seen on 2026-11-20: 25 of 25 trials per hub kept their
  target, 11 of them with a velocity write sent while the hub was marked not responding. Empty ports, so what a
  loaded motor does isn't known. Test: the same lines in the step 1 rerun.
- **Routing, a crash with writes in flight, consecutive OpModes and the once and linear-once negative controls on
  the bench build:** on 2026-11-20 they failed only while the Control Hub was silent, so that run neither passes nor
  fails them. Test: the step 1 rerun.
- **Whether Photon should forget the SDK's cached motor and servo values when it gives up on a write**, as it does
  now, or only mark the hub not responding, as the SDK's own timeout does. Decide after the lost-writes test.
- **Whether a FAIL on the SDK path's stale-race line should block Photon.** As coded it does: the line feeds the
  verdict's blockers, and the go run starts only after a USB and an RS485 block that PASSED. The Photon-off line failed
  in all three 2026-11-20 runs and fails whenever a trial hits the window, however Photon behaves, so as coded the go
  run would never start. Decide before step 3.
- **Whether Photon refuses the write when STOP's gate rises at the other two moments**, before the send and after its
  check, and whether the race catches a broken refusal ([The 2026-11-20 STOP race](#the-2026-11-20-stop-race)). Test,
  after a bench change: Photon-on trials with the gate raised after the check, and one set with the refusal disabled
  (both must FAIL), and trials with the gate raised before the send (they must drive `gateFallbacks`).
- **The stale race on a RUN_USING_ENCODER motor with Photon on.** Velocity writes take the SDK's path, so it should
  fail like Photon off (read from code). Test: the race on a velocity write, after a bench change.
- **The stale race with the hubs never swapped** (stock SDK 12 with no Photon hub objects): expected to fail the same
  way (read from source). Test: the race on the SDK's own hub objects, after a bench change.
- **What the lost-write tracing costs.** Its added work is a few array writes per frame, against the 20 ms limit
  (from the code); not measured on a hub.
- **Whether a crash with writes in flight without Photon reaches the fail-safe as fast.** On 2026-10-29 a crash took
  539-542 ms to reach it with Photon on, on 2026-11-17 27-39 ms. On 2026-11-20 the Expansion Hub took 530.8 ms, but
  that trial ran while the Control Hub was silent, so it isn't comparable, and why it took that long isn't known.
  Test: Bench: Photon Crash with Photon left off, same sampler; it always enables Photon at 8 parallel today, so it
  needs a Photon-off option first.
- **How the go run's soak can reach the loss bound** with `--photon-i2c`: at 2026-11-17's rate it needs about 4.7 h
  on USB and at 2026-11-20's about 4.3 h, both past the 4 h cap
  ([Check the soak rate first](#3-go-run-6-7-h-only-after-1-and-2-pass)). The RS485 rate isn't measured yet: the
  RS485 short run's **Write soak** gives it.

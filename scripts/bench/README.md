# Control Hub bench

Measures the framework on a bare Control Hub (no Driver Station, no robot): loop and bus costs, telemetry, the route
planner, the Limelight, webcam, Pinpoint, OctoQuad and Expansion Hub paths, stop and failure paths, dashboard gamepads
and the Sloth hot load. One command runs every bench the plugged-in hardware allows and writes
`bench-results/<timestamp>/` at the repo root (gitignored); its `summary.md` says what each FAIL means. What the results mean
for the code: `docs/hub-bench-findings.md`.

## Bare hub, unattended

For a Control Hub with nothing plugged into any port, run where there is no internet and nobody to ask:

1. While still online, from the worktree root: `scripts/bench/bare_hub.sh --build`. It refuses a dirty tree, builds the
   debug APK, checks Gradle builds offline, checks the build's `BuildConfig.PHOTON_SOURCE_SHA256` is `run_bench.py`'s
   hash of the photon sources, and stamps the APK with HEAD and that hash
   (`TeamCode/build/outputs/apk/debug/.hubtest-build`). Rerun it after every commit.
2. Power the hub from a 12 V battery with nothing plugged in, join its Wi-Fi on the Mac (no internet then), plug the Mac
   in with its lid open, close every dashboard tab, and turn off auto-join for other networks so macOS doesn't hop away.
3. `scripts/bench/bare_hub.sh`. About 45 minutes, at most 90; no keypresses. Extra flags go to `run_bench.py`; a bench
   flag runs only the benches it names. So `scripts/bench/bare_hub.sh --photon` runs only the Photon block, in place of
   the benches above: make it a second run after the plain one, and keep the first run's `REINSTALL.sh`, since the
   second backs up the bench build. On the Control Hub alone its verdict is always NOT SAFE YET (the routing check, at
   least 8 app restarts and both swap orders seen, needs two hubs), so it shows only the other failures; only the
   Expansion Hub runs below can say PASSED, and only their go runs SAFE. With `--ports-empty` it waits for one Enter
   at its start.
4. When it prints `Reconnect to the internet and tell Claude the run finished.` (or `...the run stopped.`), do that.

In order, it: stops unless the APK and its stamp match HEAD, no tracked file changed and the photon sources still hash
to the stamp's; finds adb as `run_bench.py` does and connects to `192.168.43.1:5555` (stops within about 20 s, touching
nothing, if the Mac isn't on the hub's Wi-Fi), and checks it is a Control Hub with the RC app (the SDK's
`persist.ftcandroid.serialasusb`); keeps the Mac awake (`caffeinate`); stops, saying so and how to reconnect, if adb
loses the hub at any step before the runner starts (told apart from a real answer by adb's exit status and `get-state`,
never by empty output); checks Gradle builds offline, adding `--no-sloth` if not; copies the installed RC APKs (with
sha256), its version, every `/sdcard/FIRST/*.xml`, the app's settings and the Sloth folder to
`.git/bench-backups/<time>/` in the main checkout (outside every worktree, so removing one can't delete it), and writes
`REINSTALL.sh` there, which puts that app, its settings and its Sloth jar back; passes the active-config setting from
before the install to the runner (`--original-entry`), so the runner restores that one; installs the stamped APK with
`adb install -r -t -g` (`-g` so no permission prompt waits for a tap; a signature mismatch or downgrade stops it with
the hub unchanged; it never uninstalls); deletes `/storage/emulated/0/FIRST/dairy/sloth/*` as the Run button's
`removeSlothRemote` does; starts the RC's launcher activity and waits for `======= INIT FINISH =======` and the
dashboard port; then runs `run_bench.py --gradle-offline --out bench-results/<time>`, which refuses a hub whose configs
name devices on the bench ports unless you add `--ports-empty` once they are unplugged, with the whole console in
`console.log` there. The runner restores the hardware config in a `finally`, whatever fails.

## Run it

1. Power the Control Hub from a 12 V battery and **unplug everything from motor ports 0-3 and servo ports 0-1**: the
   benches drive them with real power (up to 0.35). Never run it on a robot's hub. The runner refuses to start while a
   config on the hub names a motor or servo there; once they are empty, add `--ports-empty`. Keep I2C bus 3 empty, and
   analog port 0 unwired: the benches read it as an unwired Floodgate.
2. Join the hub's Wi-Fi, then `adb connect 192.168.43.1:5555`.
3. Full install (Android Studio, **TeamCode** run configuration, **Run**) and wait for the RC app. Do it again after
   any change to `architecture/hardware/photon/`: that code is pinned, so deploySloth never changes it on the hub, and
   the Photon block fails its first check when the installed copy doesn't match the sources.
4. Close every dashboard tab: its status requests read hub voltage on the bus being timed, and its Gamepad view sends
   gamepad states.
5. From the repo root: `python3 scripts/bench/run_bench.py` (the Mac's own python3 3.9+, nothing to install). About
   55 to 75 minutes with every device plugged in, about 45 on a bare hub; the Photon block is a separate run
   (`--photon`, below).
   The longest: Thread Lifecycle 6 (2 of them its Sloth reload), Freeze Dump 6 (five dashboard config changes; on
   slothboard 6165.6 or older each one freezes the app until it is killed and relaunched), Config by name 3 (three
   config switches), Turret PWM 2 (two config switches), Limelight Hung 1.5. Don't touch the hub. Ctrl-C once is
   safe: it restores your hardware config and ignores more Ctrl-C until that's done.
6. When it prints `Nothing to send back`, tell Claude.

The Sloth race, Thread Lifecycle's reload and the Photon block's deploySloth step run `./gradlew :TeamCode:deploySloth`
from the terminal: if `./gradlew --version` fails there, add `--no-sloth`; with no internet, add `--gradle-offline`.

### Optional hardware

Missing hardware skips its benches, and the summary says so.

| plug in | where | runs |
|---|---|---|
| a UVC webcam streaming 640x480 MJPEG (the robot's Arducam OV9782 is best) | Control Hub USB | Bench: Webcam, Webcam Stop, Webcam Thread Failure |
| the Limelight 3A, booted | Control Hub USB | Bench: Limelight (the runner restores the pipeline it found) |
| a goBILDA Pinpoint, both pods plugged in and still | Control Hub I2C bus 1 | Bench: Pinpoint (else Pinpoint Absent) |
| an OctoQuad MK2, pods on encoder ports 3 and 4, still | Control Hub I2C bus 2 | Bench: OctoQuad |
| a REV Expansion Hub on its own 12 V | RS485, address 2 (`--expansion-address N`) | Bench: Expansion Hub (reads only) |
| a REV Expansion Hub on 12 V, **nothing on its motor ports 0-3 or servo ports 0-1** (the Photon block drives them) | Control Hub USB to its mini-USB, no RS485 cable | the Photon block with `--usb-expansion` |
| the same Expansion Hub, same empty ports | RS485 only, address 2 (`--expansion-address N`) | the Photon block with `--photon-rs485` |
| the Pinpoint and the OctoQuad, for the Photon block's I2C reads | OctoQuad: Control Hub I2C port 2; Pinpoint: **Expansion Hub I2C port 1** (Control Hub port 1 with no Expansion Hub) | the Photon block with `--photon-i2c` |

`--interactive` adds terminal prompts: pull and replug the Limelight, Pinpoint and OctoQuad cables (pods stay in); move
the Pinpoint and OctoQuad to the Expansion Hub's I2C ports 1 and 2 and back; open http://192.168.43.1:8080 during
Bench: Clock on a hub whose clock still reads about 1970; drive the dashboard's Gamepad view with the keyboard, the
on-screen stick and a USB pad (close the tab afterwards).

### Flags

| flag | does |
|---|---|
| `--ports-empty` | runs although a config names devices on motor ports 0-3 or servo ports 0-1 (only once they're unplugged) |
| `--interactive` | the prompts above |
| `--no-sloth` | skips the Sloth race, Thread Lifecycle's hot reload and the Photon block's deploySloth step (no Gradle builds) |
| `--gradle-offline` | adds `--offline` to every Gradle build the runner makes |
| `--out DIR` | writes the results there instead of `bench-results/<timestamp>/` |
| `--original-entry JSON` | the app's active-config setting from before a reinstall (`bare_hub.sh` passes it): restored at the end instead of the one read after the install |
| `--photon` | only the Photon block (below), with or without `--all`; a run without bench flags, or with `--all` alone, leaves it out |
| `--usb-expansion` / `--photon-rs485` | the Photon block with the Expansion Hub on USB / on RS485 (implies `--photon`) |
| `--photon-soak` | the Photon block's long soak, 200 STOP trials per condition and more lost-write trials (implies `--photon`) |
| `--photon-stop-trials N`, `--photon-seed N` | STOP trials per condition (default 40); the seed for the STOP delays (default: the time, logged) |
| `--photon-i2c` | the Photon block also reads the Pinpoint and the OctoQuad over I2C on both hubs (about 2 min more); a wiring's verdict can only say SAFE with it |
| `--quick` | shorter phases, noisier numbers |
| `--all` | every bench but the Photon block; stops if no webcam, Limelight or Expansion Hub is plugged in |
| `--keep-configs` | leaves the `bench_*` hardware configs on the hub |

`--help` lists the rest: one flag per bench (`--hub`, `--loop`, `--threads`, `--edge`, `--turret`, `--config-by-name`,
`--freeze-dump`, `--limelight`, ...), `--serial`, `--host`, `--other-pipeline`.

## The Photon block

Where Photon stands and the hub runs still to do: [`docs/photon-bench.md`](../../docs/photon-bench.md).

`--photon` checks `architecture/hardware/photon/` against everything the robot code could do with a hub, and ends its
section of `summary.md` with a **Photon verdict**, for the wiring it ran on:

- **SAFE TO TRY ON THE ROBOT**: every Photon check passed, every STOP negative control fired, the routing check saw both
  hub swap orders, the soak met its loss bound, there were 200 STOP trials per condition, the I2C reads ran right on
  every hub (`--photon-i2c`) and, on USB wiring, the cable pulls passed. Only the go runs in the table below can say it.
- **PASSED THIS RUN, NOT YET SAFE**: nothing failed, but the run left out something the go needs; it lists what.
- **NOT SAFE YET**: it lists the blockers.

Each test's section says what it proves, the numbers, and what a FAIL means.

**Setup.** Control Hub on 12 V; the Expansion Hub powered from the Control Hub's second XT30 port; nothing on either
hub's motor ports 0-3 or servo ports 0-1 (the block drives them at 0.15-0.3 and moves servos between 0.35 and 0.6; it
prints exactly what before it starts). For `--usb-expansion` the Expansion Hub is on a USB cable only (no RS485 cable);
for `--photon-rs485` on the RS485 cable only. Pass `--ports-empty` once the ports are empty: a config on the hub that
names devices on either hub's bench ports stops the run without it, and from a terminal it asks you to confirm. The
stock bench then drives each motor port at 0.1 for 0.2 s and refuses to go on if one draws over 0.1 A. It can't probe
servo ports, and a free-spinning motor may draw less than that: check every bench port by eye.

**I2C (`--photon-i2c`).** Plug the OctoQuad into Control Hub I2C port 2, where the robot has it, and the goBILDA
Pinpoint into **Expansion Hub I2C port 1** (with no Expansion Hub, Control Hub port 1), nothing else on those ports; the
same config serves USB and RS485 wiring. From a terminal the runner asks you to plug them in before the block and to move
the Pinpoint back to Control Hub port 1 after it. Both devices are read only: the bench reads the Pinpoint's device ID and version registers
(1 and 2) and the OctoQuad's chip ID and firmware version (registers 0-3), byte for byte against what each driver read at
INIT; the only I2C writes are the OctoQuad driver's own read-parameter commands (its uptime at INIT, its diagnostics at
that OpMode's STOP). A device that isn't answering stops its OpMode at INIT with a message that names its port, and the
I2C checks fail; nothing passes without reads.

| run | command | about |
|---|---|---|
| USB Expansion Hub, short (PASSED at best) | `python3 scripts/bench/run_bench.py --photon --usb-expansion --photon-i2c --ports-empty` | 50 min; from a terminal, 2 prompts at the start and 1 at the end |
| RS485 Expansion Hub, short (recable first) | `python3 scripts/bench/run_bench.py --photon --photon-rs485 --photon-i2c --ports-empty` | 50 min; from a terminal, 2 prompts at the start and 1 at the end |
| **go, USB wiring** | `python3 scripts/bench/run_bench.py --photon --usb-expansion --photon-soak --photon-i2c --interactive --ports-empty` | 6-7 h, 2 prompts at the start and 5 at the end |
| **go, RS485 wiring** | `python3 scripts/bench/run_bench.py --photon --photon-rs485 --photon-soak --photon-i2c --ports-empty` | 6-7 h; from a terminal, 2 prompts at the start and 1 at the end |

`summary.md` is written with the Photon verdict before the last prompt, so an unattended run's results don't wait on it.

The steps, in order (each OpMode is `Bench: Photon ...`):

1. A config switch restarts the app; **Stock** runs first, so its hubs are the SDK's own: the installed Photon's
   source hash (from `BuildConfig`) against the checkout's, the port probe, then the same write workload as Photon's
   with exact read-backs, and a check that `PhotonCore.enable()` throws outside `@Photon`.
2. **Routing**, the first Photon INIT: every `LynxUsbDeviceImpl.knownModules` entry is the hardware map's
   `PhotonLynxModule`, every motor/servo controller points at it, nothing is in flight; then each hub is marked not
   responding and must clear within 50 ms of its next command, Photon off and on; then a parallel burst.
3. **Photon**: write block per hub and loop period off and at 8/4/2 parallel (a 10 ms poller thread off and at 8; the
   block times only the SDK calls: each value is chosen before it, writes in flight are sampled after each call off the clock, and the rest of the bookkeeping comes after), once more at 8 with
   Photon's per-step write timing on (`timing on8 profiled`, summarized per hub in `summary.md`), undrained bursts at 8/2/1, three writes per channel back to back (the last must win), SDK-path enable toggles between
   parallel writes, bulk/voltage reads, servo writes from a second thread while the OpMode thread writes and reads back
   the motors and a third toggles Photon, 3000 writes at a limit of 1 under a 50 ms stall watchdog, and a
   production-rate soak (60 s; with `--photon-soak` up to 1e7 writes per hub, about 3 h). With `--photon-i2c`: I2C
   phases off, at 2 and at 8, where two threads per device (the driver's own client and a second one, every 10 ms, the
   robot's sensor-thread period and rules) read their device's constant registers while the OpMode thread writes every
   channel; with Photon off the second readers go through the replaced hub objects Photon keeps, so they must share the
   hub's I2C lock; the soak keeps all four readers running. Then a negative control: 200 raw I2C write-reads per device
   sent without the hub's I2C lock beside a locked reader at full speed, reported as caught of attempted (zero caught
   on a hub means it tolerated them and the lock check couldn't be shown to fail on that hub), and a clean I2C phase
   after it. Every read must be byte-exact (never the SDK's all-zero placeholder), every reader must reach 50 reads, and
   `summary.md` prints the read times per phase against Photon off and the rule-of-three bound on per-read corruption.
   The stock bench runs the same I2C phase on the SDK's own hubs. Every phase must have zero
   wrong read-backs, finite drains under 50 ms, nothing left after (Photon's slots and map, and its stand-ins in the
   SDK's own command map), in-flight never over the limit by either count, no write pending 100 ms or more, no
   retransmit, abandoned write, NACK or unexpected fallback, and exactly one parallel write per write it should have
   parallelized.
4. **Faults**: a tap on the hub's USB device swallows chosen parallel writes and their retransmits, so their ACK never
   comes: each must be retransmitted at ~100 and ~200 ms, given up at ~250 ms, never block the caller, free its slot and
   its SDK entry, and the next identical write must reach the hub. Single lost writes, then 13 per in-flight depth 0-7
   per hub (25 with `--photon-soak`) with that many other writes in flight and the hub kept busy, then 10 per hub
   followed by a newer write to the same channel, which must end superseded (never re-sent or given up); a full limit
   of lost writes plus one more write; a USB write that throws; and a hub reset (a raw fail-safe) timed Photon off and
   on, labelled by whether the SDK learned of the reset from the hub.
5. **Stop Race** (6 runs per hub): STOP's gate raised the instant a motor write goes out; the motor must stay off.
6. **Stop** and **Stop Linear**: 40 STOPs each (200 with `--photon-soak`) at a random 1-3 s, seeded (`--photon-seed`),
   iterative and linear, each at 8 parallel and off. The linear bench clears the interrupt LinearOpMode sends at STOP
   (and records that it came), then keeps writing for 200 ms, so its late writes reach both STOP gates and its read
   runs. `stop()` reads every
   channel on both hubs after the SDK's fail-safe and before zeroing the motors, and a sampler thread reads them until
   1.5 s after the OpMode ends: any motor enabled above 1% power, or servo enabled, in that read or in a sweep after the
   first one that read every channel safe, fails, and so does a trial that crashed or restarted the app. Three negative
   controls, five runs each, re-energize motor port 0 of each hub after the fail-safe on purpose: once, the way a real
   Photon failure would, iterative and linear (`stop()`'s read must catch every one), and for 150 ms (the sampler must
   catch every one). The SDK itself can leak its network lock for 500 ms when the STOP interrupt lands at the end of a
   command (logcat's `abandoning lock`); in every linear trial, the linear negative control included, each such leak
   must fall in that window after STOP, none may be a fail-safe's (STOP's own fail-safes run on the event loop, which
   that interrupt never reaches, so only Photon's post-STOP fail-safe can leak one), and Photon on must not leak more
   often than Photon off (one-sided Fisher, p >= 0.05). That comparison is weak: the check prints the smallest rise it
   catches in half of all blocks at the trial count run (about 8x at 40 trials, 3x at 200, at the SDK's own 1-in-40
   rate), so a short block's pass rules out only a large rise.
7. Photon, non-Photon, Photon, **Crash** (throws with writes in flight, two per hub swallowed so they're never
   acknowledged), Photon: every INIT must start clean, with each swallowed write finished (superseded, given up or
   cleared at INIT) and nothing left in the SDK's command map.
8. deploySloth (skipped with `--no-sloth`), then a Photon OpMode must still enable the same pinned `PhotonCore`.
9. App restarts with **Routing** after each, at least 8 and until both hub swap orders are seen (at most 16).
10. With `--interactive` (USB wiring only), **Replug** four times: the Expansion Hub's USB cable, then its power, each
    once with Photon off and once on. Each time the runner names the cable and waits for Enter, starts the bench, then
    prints `>>> PULL NOW`: for USB, unplug the cable, count to two and plug it back in; for power, pull its XT30 lead,
    count to one and plug it back in. Its motor ports are empty, so nothing moves; the step ends about 3 s after the hub
    is back (or after 90 s). With Photon the loop must stall no longer than without (+50 ms) and no single write call
    over 300 ms, its Expansion Hub writes must fall back while the hub is gone, the Control Hub's must stay right
    throughout, and the Expansion Hub's must be right again within 1 s.
11. Framework Loop off and on, and Write Recovery on (their `Photon` variants; Write Recovery is judged as Bench: Write
    Recovery is, below), then an app restart so the benches after the block get the SDK's own hubs.

The SDK warnings in logcat (lock, ownership, checksum, abandon, I2C and placeholder lines) are judged for every step but
the cable pulls, except the ones a step causes on purpose (the lost-write and reset phases, the routing bench's own
not-responding test, the I2C negative control), the linear STOP trials' lock leaks (judged against their Photon-off
control instead) and a hub answering the firmware-version read late after a restart, which Bench: Probe shows without
Photon too (counted, not judged); a step whose log is empty or lacks one of its phase markers fails. The soak's step streams logcat to a file instead of relying on the hub's ring
buffer.

With `--photon-rs485`, Bench: Photon adds a shared-link phase, off and on, with the poller running: the Control Hub's
writes (parallel when on) and the RS485 Expansion Hub's (which must never go parallel) every loop, with exact
read-backs on both and no SDK lock warning.

## What it changes on the hub

It writes `bench_*` hardware configs to `/sdcard/FIRST` and switches between them by restarting the RC app; at the end
it re-activates the config that was active (writing its XML back from `original_config.xml` first when its name is a
`bench_*` one, which the run overwrote), deletes the `bench_*` files and restarts the app once more. Config by name
saves settings that name a shipped config by another one's resource id, and a name the build doesn't ship, then
switches back. The configs put `fl`/`fr`/`bl`/`br` on motor ports 0-3, `benchServo`/`benchServo2` on servo ports 0-1
(`TurretL`/`TurretR` in `bench_turret`), `floodgate` on analog 0 and `analog0`/`analog1` on analog 2 and 3; only the
Photon block's config also names the Expansion Hub's motor ports 0-3 (`efl`/`efr`/`ebl`/`ebr`) and servo ports 0-1
(`eServo`/`eServo2`). Bench: Freeze Dump, last, sends slothboard's SET (three times), WRITE (the active config's own
contents) and DELETE (a `bench_freeze_scratch` it pushes first) from idle; on slothboard 6165.6 or older each one
freezes the app, which the Control Hub kills and relaunches. It also raises the hub's log buffer to 8 MB until the next
reboot. It never deletes the active config, and a run that stops before writing the first `bench_*` config leaves the
configs untouched. With `bare_hub.sh` the hub runs the bench build from its install on; only `REINSTALL.sh` puts the
old app back.
With `--interactive`, opening the RC web page during Bench: Clock sets the hub's clock until it powers off.

If it prints `RESTORE FAILED`: open http://192.168.43.1:8080/dash, add the Hardware Config view and activate the config
the message names (a Control Hub has no Configure Robot screen without a Driver Station).

## The benches

| OpMode | needs | measures |
|---|---|---|
| Bench: Probe | nothing | the IMU chip, attached webcams and Limelights, whether an Expansion Hub answers, the hardware map's device names; whether the original shipped config's saved resource id still names its XML on this install |
| (config baseline, no OpMode) | nothing | slothboard's config baseline lists `Optimization Toggles` |
| Bench: Hub | nothing | empty-loop overhead; bus call costs; telemetry, field map and packet costs; the floodgate pin's volts; sliced planner latency, slice lengths and loop period for 0 to 5 balls; LLResult parse; Limelight POST and poller costs against a fake Limelight |
| Bench: Framework Loop | nothing | the EnhancedOpMode loop with Drivetrain and Pedro (capped at 0.2), one telemetry toggle per phase; the current limiter stays at 1.0 on an unwired floodgate |
| Bench: Raw Loop | nothing | the loop of a bare LinearOpMode running Pedro (capped at 0.2) without the framework, idle and then following Close Flower's first leg |
| Bench: Failure Path | nothing | a hook throwing in each lifecycle stage: which `stop()`s run, Suppressed failures, the dashboard Error line, and on the hub's clock how long the SDK takes to log the throw; the previous crash's Error line gone at the next INIT, judged only when a status showed the line empty (or stale) before the run's own crash |
| Bench: Thread Lifecycle | nothing | `biobuzz/modules/TransferCounter`'s poller: one daemon thread, `stop()` within 50 ms, 90+ polls/s; no poller left at the next INIT after a normal STOP, a STOP from INIT, a crash in `loop()` and a throwing `stop()` (each also seen from adb); a failing poller stops the OpMode within 50 ms; whether `interrupt()` mid-command leaks the hub lock; 10 back-to-back runs; a Sloth reload while it runs |
| Bench: Edge Through Start | nothing | BioBuzz Tele's `EdgeBooleanSupplier` pattern with gamepad1 LB held from INIT through START: no edge at START, one per later press |
| Bench: Turret PWM | nothing (`bench_turret`) | BettaTurret's PWM, read from the hub's registers, through CENTER, `stop()` with writes off and on, writes back on, IDLE and RightState.FORWARD |
| Bench: Clock | nothing | `PathCommands.timeout` (15 s) against wall-clock jumps |
| Bench: Pinpoint, Pinpoint Absent | a Pinpoint / none | firmware, status, and whether a start pose set in INIT survives the IMU recalibration / that a missing Pinpoint raises a fault or keeps the pose `setPose` set |
| Bench: OctoQuad | an OctoQuad | GuardedOctoQuadLocalizer's read costs, status changes, CRC failures after the first RUNNING read, start-pose survival |
| Bench: OctoQuad Absent | an empty I2C bus 3 | GuardedOctoQuadLocalizer with an `octoquad` configured and nothing answering: builds within 1 s, the `OctoQuad` fault raised in INIT within 5 s, INIT keeps looping, START throws naming the OctoQuad |
| Bench: Expansion Hub | an Expansion Hub | the same Lynx commands and odometry reads on each hub |
| Bench: Write Recovery | nothing | seven `failSafe()`s, 0 to 235 ms after a write: EnhancedMotor and EnhancedServo (back only on WriteCache's re-send) reach the hub again within the re-send period + 2 loops, raw devices within 2 loops; the worst seen |
| Bench: Drive Handover | nothing | `fl`'s SDK power matches Drivetrain's command from the 2nd loop after Pedro hands back |
| Bench: Photon Stock, Photon, Photon Faults, Photon Routing, Photon Stop, Photon Stop Linear, Photon Stop Race, Photon Crash, Photon Replug, Framework Loop Photon, Write Recovery Photon (`--photon`) | see the Photon block | see the Photon block |
| Hardware OpMode, then Bench: Power Readback | nothing | a stale Hardware-view Power edit doesn't drive `fl` under the next OpMode |
| Bench: Gamepad, Gamepad Linear | nothing | dashboard gamepad delivery, one edge per press, sticks resting within 1 s of silence |
| Bench: Limelight Fault | nothing | the `Limelight` fault with nothing answering: raised in INIT within 5 s, loop keeps running, collection ends SKIPPED |
| Bench: Limelight Hung | nothing | the real Camera against a fake Limelight that serves results and never answers a POST: the `Limelight` fault raised in INIT within the 15 s POST timeout + 2 s, INIT and the loop keep running, one POST and no retry; STOP during a hung POST isn't held up; the next INIT doesn't wait for it, its POSTs never overlap the old one (`reinit` reuses the stopped run's worker; only `reinit_fresh`, with a second worker on the shared blackboard semaphore as a reloaded LimelightSync would use, tests the shared sender), and its sync completes once the fake answers |
| Bench: Webcam Stop (absent) | nothing | INIT fails with CellTipCamera's "failed to open" |
| Bench: Webcam | a webcam | open/close times, per-frame and stream costs, frame age against `CellTipCamera.maxStalenessMs` |
| Bench: Webcam Stop | a webcam | STOP at several points with the webcam open |
| Bench: Webcam Thread Failure | a webcam | a camera-thread exception reaches the Error line within 2 s |
| Bench: Limelight | the Limelight | slothboard's proxy, `LimelightSync` time, INIT to first frame, POST costs, frame rate and latency |
| Bench: Planner Stop | nothing | STOP during 4- and 5-ball sliced plans: no app restart, hub-side STOP to idle within 900 ms, every 4-ball slice under 300 ms |
| Bench: Overlay NaN | nothing | a NaN in a field overlay doesn't kill dashboard telemetry |
| (config by name, Bench: Probe) | run-as | a shipped config saved with another shipped config's resource id loads its own XML; a name the build doesn't ship loads no config (both FAIL on a build without re-resolving by name at robot setup) |
| Bench: Freeze Dump | nothing | slothboard ignores a config switch during INIT; SET (3 runs), WRITE and DELETE from idle each restart the robot in the same process: no SDK `deadline expired ... EventLoop`, no Control Hub `Relaunching the RC`; thread dumps every 2 s from 3.5 s without an RC alive broadcast, a SIGQUIT and ART's dump of the frozen pid (runs last) |
| Bench: Sloth Probe | Gradle | INIT racing deploySloth, the OpMode list during a load, the load signal, the jar surviving an app restart |

The OpModes also run from the dashboard, but they expect the runner's configs and write results only to the hub.

**Bench: Analog Poll** is run by hand only: `laser_rangefinder.xml` active, both rangefinders plugged in (it commands no
motor). About 80 s after START the DS and dashboard show its summary; the rest is in `analog_poll.json` in the hub's
bench folder. It measures how often each analog value refreshes and what a poller thread (`pollMs` in that folder's
`params.json`, default 10) costs a stand-in robot loop.

## Notes for changing the bench

- The active config is read from the hub, never from slothboard's broadcast lists, which a restart can drop. To switch,
  the runner stops the app, writes `RobotConfigFile`'s Gson form under `pref_hardware_config_filename` in its default
  SharedPreferences with `run-as com.qualcomm.ftcrobotcontroller` (needs the Run button's debuggable APK; a running app
  saves its own copy over it), and restarts it; a switch is done at logcat's `======= INIT FINISH =======`. Only without
  run-as does it switch through slothboard, which on slothboard 6165.6 or older freezes the app until the hub's
  watchdog relaunches it about 10 s later.
- A shipped `res/xml` config is stored by resource id, which an install that adds, renames or removes a `res/xml` file
  can renumber: the probe checks the stored id (from before the install with `--original-entry`) still names that XML,
  and the restore re-activates shipped configs by name, which fixes that hub only, so `summary.md`'s Restore line says
  when it did. Config by name picks two shipped configs with Control Hub devices alone, each with a device the other
  lacks, saves the first under the second's id (and `bench_unshipped` under that id), restarts the app and reads the
  hardware map's device names to tell which XML loaded; a load of no config reports no battery, which its switch allows.
- When the original config's name is a `bench_*` one (a hub left on `bench_probe` by an earlier run), the restore
  writes its XML back from `original_config.xml`: the run overwrote that file with its own. Every distinct XML the run
  writes under one name is kept in `logs/` (`<name>.xml`, `<name>.2.xml`, ...).
- Params and results go through the app's own `files/bench/` via run-as (`/sdcard/FIRST/bench/` only without it), and
  each write is read back: an OpMode once read a just-pushed `/sdcard` `params.json` as empty.
- Each config switch's logcat is in `logs/config_*.logcat.txt`, each step's in `logs/<step>.log`; the 8 MB log buffer
  keeps the hub's `Lights Hal` spam from pushing them out. `logcat -T` starts on a whole second, so a step's file can
  open on the previous step's STOP-time lines: every step, the Photon block's included, is judged on its log from its
  own `begin <step>` mark on. The runner passes `-T` a dated time when the hub's logcat takes one: logcat takes a
  missing year as the current one, so a start before a clock set or local midnight would read nothing.
- The websocket is `ws://192.168.43.1:8000/`. NanoHTTPD drops a socket silent for 5 s, so the runner pings
  (`GET_ROBOT_STATUS` would put hub voltage reads in the timings). An OpMode list within 1 s of a socket opening is
  slothboard's on-connect copy, not a broadcast. A restarting app accepts a socket and drops it again within a second or
  two: the runner waits 0.5 s, then 1, 2 and up to 4 s before each reconnect while connections keep dying within 5 s,
  and between failed connects.
- SDK 12 only logs an OpMode's exception (`User code threw an uncaught exception`, tag `OpModeManager`). The runner takes
  it from slothboard's `RECEIVE_LOGCAT_ERRORS` (after the step began, until STOP), from a status back at `$Stop$Robot$`
  without BENCH DONE, or from adb logcat while waiting for INIT. `errorMessage` is the status's, else that line;
  `statusErrorMessage` is exactly the dashboard's Error line. The SDK logs that line about 0.45 s after the throw, so
  the runner's crash and Error-line times count from its own detection; Failure Path reports the hub's throw-to-line
  time from logcat separately. slothboard's logcat monitor sends its whole buffer again whenever it restarts,
  so `summary.md` counts its lines by (timestamp, message); the crash watch ignores lines from before the step.
- An app restart takes about 40 s, during which slothboard's status names no OpMode and a request can close the socket:
  the runner re-sends and waits for idle wherever it stops, starts or recovers a step, and `summary.md` lists them.
- `BenchReport` writes the result file and logs `BENCH DONE <file>` under `BENCH` before the `BENCH DONE` packet: when
  the dashboard goes quiet the runner polls logcat every 5 s and finishes from the file. No packet within 5 s of INIT
  means slothboard's telemetry thread died: the runner logs why and restarts the app after the step.
- slothboard's Limelight proxy holds `*:5807` from app start and Linux refuses `127.0.0.1:5807` beside it, so Bench:
  Hub's fake Limelight is on `127.0.0.1:15807`, each loopback `Limelight3A`'s private `baseUrl` repointed by reflection.
- With no Driver Station the SDK drops telemetry before serializing it, so DS-frame rows leave out serialization and the
  UDP send; `summary.md` labels them.
- The probe names the IMU by `LynxModuleImuType.name()` (`BHI260`, not `toString()`'s `BHI260AP`) and stops if it can't.
- The Sloth race sets `ANDROID_SERIAL` (the Load plugin runs bare `adb`), kills a deploySloth still waiting on the Sloth
  lock after 600 s, and times each load from the hub's logcat after its `BENCH sloth pushed <run>` mark.
- The gamepad benches take their phase from gamepad2's right trigger (phase / 100) and a message counter on its left
  trigger, so an all-zero rest state leaves the phase alone; phase 99 ends the bench.
- Bench: OctoQuad Absent holds INIT 6 s, then STARTs, expecting the guard's OctoQuad error.
- Bench: Pinpoint Absent passes on a fault whose key or message names the Pinpoint, an error naming it, or every pose
  read after `setPose` within 0.5 in and 1° of it.
- Bench: Write Recovery moves every command by 0.002 before each `failSafe()`, so all four devices write at a known
  moment, and reads WriteCache's re-send period from the class by reflection.
- Bench: Limelight Hung runs right after Bench: Limelight Fault on its config, repointing the config's `limelight` at
  the fake, which stays so until the app restarts (the switch back to the main config does that). Its fake outlives a
  `stop_hung` run in a static, and the runner INITs `reinit` right after with nothing between, so the INIT lands inside
  the stopped run's 15 s POST timeout; a slower turnaround FAILs the setup check, not the robot. `reinit_fresh` clears
  LimelightSync's private `running` flag by reflection, as a Sloth reload would leave it: a real deploySloth can't land
  in that window. A Bench: Hub after a crashed hung run, before an app restart, can't bind `127.0.0.1:15807`.
- Bench: Thread Lifecycle runs on the main config (`analog0`/`analog1`). Each run's result file is
  `thread_lifecycle_<mode>`, pulled to `raw/thread_lifecycle_<run>.json`; the next run's `leftoverAtInit` judges the one
  before (no hook runs after a crash), and a pid change between them skips that check. After each run the runner lists
  the app's threads over adb, trying `run-as ... cat /proc/<pid>/task/*/comm`, plain `cat`, `ps -T -p` and `ps -t`
  (`TransferCounter` is exactly comm's 15 characters); a listing during the first run is the positive control, and
  `summary.md` names the method that worked. Its Sloth reload deploys during a 600 s run: Sloth stages the load and
  releases it from the idle OpMode, so it lands after the runner's STOP.
- Bench: Edge Through Start needs the hold unbroken across START: a background sender (`GamepadHold`) repeats LB every
  50 ms from INIT until the RUNNING phase is going, since slothboard rests both pads after 500 ms without a message.
- Bench: Freeze Dump's watch thread outlives the OpMode (slothboard ignores a config switch unless idle): for each of
  SET, SET, SET, WRITE and DELETE the runner INITs and STOPs it (arming a fresh watch), sends the message from idle,
  waits up to 120 s for the robot to set up again in the same pid or for a new pid (else force-stops and starts the
  app itself), and pulls `freeze_watch.json` and every `freeze_dump_*.txt` into `raw/freeze/<run>/`, with the logcat
  in `logs/freeze_dump_<run>.log`. It copies `/data/anr/traces.txt` as soon as the frozen pid logs `Wrote stack
  traces`: the relaunched app's own startup ANR rewrites the file about 25 s after it starts. Lock owners come only
  from a `----- pid <frozen pid>` section dated after the change (the file also holds that pid's own startup ANR).
  A run passes only with the setup in the same pid and neither `deadline expired waiting for service termination:
  EventLoop` nor the Control Hub's `Relaunching the RC`: the watch's dumps can't decide it, since every robot setup
  waits 5 s for USB between two alive broadcasts, and the SDK's `awaiting shutdown` comes with any teardown over 2.5 s.

# DECODE teams' code: loop time, ball sensing, TeleOp automation, architecture

What 23 FTC teams' public DECODE-season (2025-26) code does in four areas, and what our robot should take from it.
Read the section for the area you are about to work on. How the same teams choose a shot and control a flywheel is
in [`decode-shooter-survey.md`](decode-shooter-survey.md), which also lists each repository and the commit read.

Read from source on 2026-11-10. One reader per repository traced a full loop from the competition TeleOp and main
Auto; each section's counts were then recomputed from those 23 write-ups, and its ideas and every statement about
our own code were checked a second time against the repositories and against our code. Apart from three loop times
in commit messages, no team recorded a measurement for anything here, so "what to copy" means worth trying, and
every gain still has to be measured on our robot.

## Take first

1. **Make `Turret`'s AUTOAIM aim from this loop's pose.** It aims in `read()`, which runs before
   `follower.update()`, so it uses the previous loop's pose (found while comparing; 9 of the 23 have the same bug).
   Details under [Architecture and tooling](#architecture-and-tooling).
2. **Bring SOTM Test's ready gate into TeleOp**, with a rumble when it refuses a shot and a way past it.
   [TeleOp automation](#teleop-automation).
3. **Carry the Autonomous pose into TeleOp through a file** that refuses a missing or zeroed pose.
   [TeleOp automation](#teleop-automation).
4. **Count balls out, not only in**: a flywheel-dip detector per shooter, and Autonomous pickups that end on the
   transfer counter inside a minimum and a maximum time. [Ball counting and detection](#ball-counting-and-detection).
5. **Record shot-table rows with a button** in the shooter tuning OpMode instead of typing them from telemetry.
   [Architecture and tooling](#architecture-and-tooling).
6. **Decide where the turret and flywheel encoders plug in with the loop cost in view**: an OctoQuad read that
   includes encoders costs about 3.4 ms more per loop than odometry alone. [Loop time](#loop-time) and
   [`flywheel-control-findings.md`](flywheel-control-findings.md).

## Loop time

Our base OpMode already does what the most thorough of the 23 do: MANUAL bulk caching (one command reads all of a
hub's inputs, reused until cleared) cleared once a loop, voltage read every 50 loops, Driver Station lines built only
when a frame is due, and write caches (skip a write within a tolerance of the last one sent) set per device, plus a
deliberate 250 ms re-send that none of the 23 has. Only 3 wrote a loop time down, all in commit messages, so nothing
here has a measured gain. Worth taking: read the turret encoder through a free Control Hub encoder port instead of the
OctoQuad, loosen the hood tolerances, and give `Intran`'s current reads the flag `Drivetrain`'s already have.

### What to copy

| idea | what it is | why it fits us | seen in |
|---|---|---|---|
| Turret encoder on a free motor encoder port | Wire a quadrature encoder to an unused motor port's encoder input, so it arrives in that hub's bulk read. | The plan puts it on the OctoQuad, whose odometry-plus-encoders read took 6.58 ms p50 against 3.20 ms for odometry alone (bench, 2026-10-09); a single-channel read is unmeasured. In the planned layout Control Hub encoder ports 1 and 2 are free and that hub's bulk read already runs for the flywheels and rangefinders. Those two ports count in software, so check counts at full turret speed. [`wiring-findings.md`](wiring-findings.md) puts it on Control Hub encoder 2 and makes the same call for the flywheel encoders. | `GoldenDragons772/Decode` (turret), `FTC11329/11329-2026-repo` (turret), `Meta-Infinity/DecodePublicRelease`, `junkjunk123/RevAmped-Decode-V2`, `23521-Desoto-Technix/2025-decode` |
| Hood tolerance sized in hood angle | Set each servo's tolerance from what one step moves the mechanism: looser on outputs that change every loop, tight on aim. | In shoot-on-the-move (SOTM) `Shooter`'s two hoods (0.001) follow the shot solution and can both change every loop while driving: about 3.1 ms a loop (two servo writes, bench sum). We already set tolerances per device (drive 0.05, flywheels 0.005, servos 0.001), and [`loop-time-findings.md`](loop-time-findings.md) proposes 0.002–0.005 for the hood; the survey adds practice, not measurements. Keep the turret tight: one team's 0.01 servo cache moved its turret in about 3.2° steps. | `Meta-Infinity/DecodePublicRelease` (hood 0.002, turret 0.001), `MOEbo-Sapiens/MOEbo-Sapiens-Decode` (hood 0.005, turret 0.0025), `dani-Nayal/SolarFlare-Decode` (set in servo degrees) |
| Motor current behind a flag (ours already, except `Intran`) | Read current only when a flag asks or from a short cache, never for every-loop telemetry. | Each read is its own hub command outside the bulk read (SDK 12.0.0 bytecode); `Drivetrain`'s four took our bench loop from 6.32 to 12.13 ms p50; `motorCurrentTelemetry` and `currentTelemetry` default off. `Intran.onTelemetry` (lines 106–107) reads intake and transfer current whenever module telemetry runs, which is every loop while dashboard telemetry is on: about 5.2 ms on the Expansion Hub the layout gives them (2.61 ms p50 each). No OpMode uses `Intran` yet. | `BeepBot99/CodeBloodedDecodeV3` (flag), `BaronClaps/16010-Worlds` (200 ms cache), `drwho495/14906Decode` (replaced by the flag below) |
| Hub over-current flag for stall checks | Set the motor's current alert once at INIT; `isOverCurrent()` then comes from the bulk read under MANUAL caching (SDK 12.0.0 bytecode). | A transfer-jam or intake-full check with no current read. Free only on a hub whose bulk read already runs: on the input-free Expansion Hub planned for intake and transfer it starts that hub's bulk read (4.01 ms p50, bench), more than one current read, so check it only while feeding. | `drwho495/14906Decode` (transfer), `SPS-Robotics/DECODE-V2` (intake full), `Skeleton-Army/FtcRobotController-DECODE` (flywheel stall) |

Already ours or not needed: one bulk-cache clear per loop (ours is in the base loop, INIT included); throttled
voltage; less frequent telemetry (dashboard telemetry off took our bench loop from 6.32 to 4.21 ms p50); Limelight
HTTP calls out of the loop (`LimelightSync`); I2C ball sensors read only on a cheap gate (all 8 that have them; ours
are analog); the Pinpoint's `setBulkReadScope` (odometry is moving to the OctoQuad); SRSHubs; off-thread telemetry
senders (both leak a thread per OpMode run); Photon (bench-gated). A localizer-only TeleOp update (2 repos, on Pedro
2.x) is what Pedro 3.0.2 already does: an IDLE follower, as ours is while `Drivetrain` drives, sends no other command.

### Counts

| what the code does | repositories (of 23) |
|---|---|
| bulk caching in the competition TeleOp: MANUAL, cleared once a loop / AUTO / OFF (the SDK default) | 14 / 5 / 4 |
| write dedupe on mechanisms: own cache / library wrappers only / the SDK's exact-match skip only; re-sends on purpose | 12 / 7 / 4; 0 (16 surveys say none) |
| battery voltage: read every loop in some match path / every 200–500 ms or 100 loops / never read | 10 / 5 / 8 |
| motor current: read every loop / behind an off flag / 200 ms cache / hub over-current flag | 4 / 1 / 1 / 3 |
| I2C ball sensors read only on a gate or every Nth loop | 8 (all that read them in match code) |
| telemetry: built every loop, no limit of their own / built every loop, send slowed or on a thread / built less often | 16 / 3 / 4 |
| team-written threads (both telemetry senders); Photon enabled / tried and removed / vendored but off | 2; 1 / 1 / 1 |
| odometry on a goBILDA Pinpoint / OctoQuad; TeleOp updates the localizer only; fixed-rate loop | 23 / 0; 2; 0 |
| loop time shown live / written down anywhere | 18 / 3 |

Counted from the 23 survey results, each code checked against its survey text. The only recorded loop times are
commit messages, with no method: `MOEbo-Sapiens/MOEbo-Sapiens-Decode` 30 ms without Photon and 31 ms with it
(2026-02-14, before that day's caching and telemetry work); `Andover-Robotics/4410-Decode` "lag reduced to 40 ms"
(2026-01-17), then "loop times around 22ms" (2026-01-25) after MANUAL bulk caching, an SRSHub and other changes;
`Skeleton-Army/FtcRobotController-DECODE` 30 ms per scheduler loop (2026-03-11). Nobody recorded another number.

### What didn't work for them

- **Photon** (writes sent without waiting for each reply): one repo's old PhotonCore went 30 → 31 ms by its commit
  messages and registers only parent modules, so RS485 Expansion Hub writes take the stock path (inferred); one tried
  SolversLib Photon for a day and went back (no reason recorded); one vendored it and never enabled it.
- **Reversed:** a MANUAL clear in a TeleOp-only update (back to AUTO the next day), odometry on an SRSHub (the turret
  encoder skipped ticks), a telemetry gate every 10 loops, and servo write caching (off via a −0.1 tolerance).
- **AUTO bulk caching (5):** a repeat read starts a new bulk read; one repo probably pays 4–6 a loop (estimated).
- **Write caches that drop a write:** 3 skip a change to 0 under the tolerance, leaving a small power on; 2 keep
  cache state across OpModes, so a first write can be swallowed (inferred).
- **Hidden per-loop work:** 3 enable Pedro 2.x drive voltage compensation, a voltage read in every `follower.update()`
  (Pedro 3.0.2's mecanum reads none); 3 read odometry twice a loop (1 more plausible); 2 write the Pinpoint every
  loop; 4 run the scheduler or bindings twice a loop (1 only in INIT).
- **Vision:** 5 run a camera pipeline or Limelight poller nothing reads; 4 make a blocking Limelight HTTP request on
  the OpMode thread every loop in some path.
- **Loop structure:** 9 aim from the previous loop's pose, as our `Turret` does in AUTOAIM (it aims in `read()`, which
  runs before `follower.update()`; SOTM aims after). 11 surveys flag per-loop allocations and none measured garbage
  collection (ours: about 63 kB a loop; 17 collections took 1.1 s of 19.6 s in the Ivy Path Test log).

### Per team

| repository | bulk caching | write cache | slow reads | telemetry | notes |
|---|---|---|---|---|---|
| `23521-Desoto-Technix/2025-decode` | OFF (cleared, mode never set) | NextFTC 0.01 | voltage 0.5 s (library) | sent every 250 ms by a worker thread | flywheel encoder on a drive port |
| `junkjunk123/RevAmped-Decode-V2` | MANUAL | servo 1/256, motor 0.01 | no voltage; one colour sensor a loop, intake window only | every loop | TeleOp: localizer only |
| `kleongf/FTC_Decode` | MANUAL | SDK exact only | 2 voltage, 1 current a loop | every loop, twice | turret analog encoder read only to re-zero |
| `FTC11329/11329-2026-repo` | MANUAL | exact; hood 0.004; drive, flywheel, turret uncached | voltage 300 ms; I2C colour gated by an analog distance pin | debug toggle | turret encoder on a motor port |
| `Meta-Infinity/DecodePublicRelease` | MANUAL | 0.001–0.02 per device | voltage every loop | built every 300 ms | flywheel encoder on a drive port |
| `GoldenDragons772/Decode` | AUTO (MANUAL reverted) | SDK exact only | voltage every loop | every loop | turret encoder on a roller motor's port |
| `Dubble-Bubble/HIVEDecode` | OFF in TeleOp | turret 0.001, auto drive 0.01; flywheel, hood uncached | voltage every loop (250 ms in drive) | every loop | |
| `MOEbo-Sapiens/MOEbo-Sapiens-Decode` | MANUAL | motor 0.05, servo 0.0025–0.01 | voltage every loop | built 125 ms, sent 250 ms | PhotonCore on; TeleOp: localizer only; 30/31 ms |
| `DennNuke/ftcpanheya-worlds` | MANUAL in TeleOp, unset in autos | SolversLib 0.0001, Pedro 0.01 | voltage every loop (Pedro) | dashboard every loop | |
| `Andrei0016/CNapSys-22586-Decode` | OFF | SDK exact only | flywheel encoder about 5 reads a loop | every loop | I2C sensor thread in later commits |
| `GNCE/DECODEv2-Ditto` | MANUAL | exact; SolversLib 0.0001 | voltage 500 ms; 3–5 current reads a loop | every loop | |
| `drwho495/14906Decode` | AUTO | exact | voltage every loop; over-current flag | every loop | flag replaced 2 current reads |
| `Xtendera/OverclockDecode` | OFF | SolversLib 0.01 drive, intake | current every loop; I2C every 160 ms, in fill order | DS every 400 ms | |
| `Skeleton-Army/FtcRobotController-DECODE` | MANUAL | 0.0001 | voltage every loop; I2C only while collecting; over-current flag | DS every 500 ms | Pinpoint bulk-read scope; 30 ms (commit) |
| `SPS-Robotics/DECODE-V2` | MANUAL (NextFTC) | NextFTC 0.01, hood 0.0001 | no voltage; over-current flag | every loop | |
| `BeepBot99/CodeBloodedDecodeV3` | MANUAL | CachingHardware (0.005/0.001 per its README) | none; current behind a flag | every loop, to 3 outputs | |
| `ryalisuchir/Decode` | MANUAL | turret 0.001 | none; analog ball sensors | built every 4th loop, worker thread | |
| `dani-Nayal/SolarFlare-Decode` | MANUAL | power 0.04, servo 0.35° | voltage every 100 loops; colour at most every 2 loops, on demand | every loop | Photon vendored, off |
| `Andover-Robotics/4410-Decode` | MANUAL | SDK exact only | current every loop; 6 colour sensors via SRSHubs, alternate loops | every loop | 40 → ~22 ms; Photon tried |
| `Scythxrz/TheFBI2025Decode` | AUTO | SolversLib 0.0001 | voltage 200 ms | every loop | |
| `BaronClaps/22131-State` | AUTO | 0.01 on 3 motors | I2C every 2nd loop while waiting | every loop | |
| `OmicronX-30099/FTC-Decode-v2` | MANUAL (NextFTC) | NextFTC 0.01 motors; servos off | voltage every loop | every loop | |
| `BaronClaps/16010-Worlds` | AUTO | 0.01 on 4 motors | current cached 200 ms; I2C every ~3rd loop | every loop | |

## Ball counting and detection

Our transfer counter (`biobuzz/modules/TransferCounter`, so far run only by Transfer Thread Count Test) counts balls
in: a distance edge with hysteresis across two analog rangefinders, read every 10 ms. Most of the 23 sensed balls
going in too; almost none sensed them going out: 19 of 23 fed for a fixed time, while a button was held or by
mechanism position, and the only per-ball exit detector in match code (2 of 23) watches for a flywheel speed dip. Add
one per flywheel (the shooter that dips says which ball type left), time every sensor state in milliseconds, and let
the count drive the intake, pre-spin and Autonomous exits. No surveyed robot fed two shooters from one transfer or
told two ball sizes apart, and none recorded a miss rate, so all of this needs our own bench numbers.

### What to copy

| idea | what it is | why it fits us | seen in |
|---|---|---|---|
| Flywheel dip per shooter | Wait for the flywheel to hold steady (3 loops within 30 ticks/s of its peak, their encoder's units), count a ball when it falls more than 100 ticks/s below the peak, ignore the next 150 ms, then take the recovered speed as the new peak (`BallDetector.java`). | Each shooter has its own flywheel, so a dip counts a ball out and says which type it was. Nobody ran it in rapid fire: one team pauses the feed after each detected ball, the other caps the wait at 0.1 s. `Shooter` averages RPM over `rpmWindowMs` (50 ms), which may flatten a short dip (unmeasured); time the steady test in ms, not loops. | `Scythxrz/TheFBI2025Decode`, `dani-Nayal/SolarFlare-Decode` |
| "Transfer clear" ends a volley | End shooting once no sensor sees a ball, wait 200–500 ms, then reset the count. One repo reads "clear" raw while its count stays latched, and latches a position only after 7 in-range reads 15 ms apart, so a ball sliding past should not latch (inferred from the timing). | Needs a sensor where the last ball waits: our two rangefinders sit together (one ball covers both), so their "clear" says only that one spot is empty. "Clear" must be the exact negation of "ball present": 2 repos got that wrong. | `Andrei0016/CNapSys-22586-Decode` (origin/master, after the surveyed commit), `23521-Desoto-Technix/2025-decode`, `GNCE/DECODEv2-Ditto` |
| Time-qualified sensor states | A state flips only after the raw reading has disagreed for a set time, with separate on and off times (250/400 ms, 1/400 ms). A ball held at the latch for 80 ms stops the transfer so it does not grind. One count advances only at the next expected beam and reads levels, so it re-converges after a reset. | Our 10 ms thread gives a clean time base; 5 of the 18 with a ball signal filter by sample or loop count, which drifts with loop time. Ours counts A-to-B edges, so a missed edge stays wrong until reset: re-converging needs a sensor at each place a ball rests. | `GoldenDragons772/Decode`, `Meta-Infinity/DecodePublicRelease`, `kleongf/FTC_Decode`, `BeepBot99/CodeBloodedDecodeV3` |
| Count-driven Autonomous exits | Leave a collection spot when full, but not before a minimum (500 ms) or after a cap (1.5 s); partial counts get later floors (2 balls plus 150 ms, 1 ball after 1.4 s, 2.5 s cap). Empty after the timeout: shift the target 0.4 in and try again. | Fits `BallCollection` as an Ivy race; build the waits with `PathCommands.waitMs`, not Ivy's wall-clock `Commands.waitMs` (CLAUDE.md). 13 of 23 end collection on their ball signal. | `BaronClaps/16010-Worlds`, `junkjunk123/RevAmped-Decode-V2`, `Meta-Infinity/DecodePublicRelease`, `GNCE/DECODEv2-Ditto` |
| Ball type on the analog path | A Brushland colour rangefinder, configured once over I2C, puts hue or distance on an analog pin that rides the bulk read. Another team reads its I2C colour sensor only after an analog distance sees a ball. | One transfer feeds both shooters, so pollen and nectar must be told apart. Our rangefinders are already analog; whether distance alone separates a 2.8 in from a 3.6 in ball is untested. An I2C read costs several hub commands; all 8 repos with I2C ball sensors throttled or gated them. | `ryalisuchir/Decode`, `FTC11329/11329-2026-repo` |
| Intake, transfer and flywheels follow the count | Stop or slow the transfer when the first ball reaches the top sensor so the intake stacks the rest; at full, stop the intake 0.5 s later, or run a light reverse (−0.3) to push extras away; spin the flywheel from idle to speed as soon as a ball is sensed. | Stops jams at full and gives both flywheels a head start; 11 and 3 of 23 do these. | `23521-Desoto-Technix/2025-decode`, `GoldenDragons772/Decode`, `FTC11329/11329-2026-repo`, `GNCE/DECODEv2-Ditto`, `ryalisuchir/Decode`, `drwho495/14906Decode` |
| Sensor faults and a recount button | 5 implausible reads in a row disable a sensor and post a Driver Station warning; a sensor still covered after 3 kicks over 5 s is ignored, marked on the LEDs and re-enabled by a button; a driver button clears and rebuilds the count. | Our fail-fast rule (CLAUDE.md) allows a fault and fallback only for the webcam, Limelight and Photon, so falling back on a dead chute sensor is a team decision. Test "stuck" after the mechanism moves, not before (one repo's bug). | `Skeleton-Army/FtcRobotController-DECODE`, `Andover-Robotics/4410-Decode`, `FTC11329/11329-2026-repo` |
| Over-current flag as a jam check | Set the hub's current alert once, read `isOverCurrent()` each loop, and require it to hold (650 ms). It says "stalled", not how many. | SDK 12.0.0 serves the flag from the hub's bulk read unless caching is OFF, and `setMotorCurrentAlert` sends a command on every call (read in `LynxDcMotorController`). Free only on a hub whose bulk read already runs: our planned layout keeps the Expansion Hub, with intake and transfer, free of inputs, so there it adds that hub's bulk read, about 4 ms a loop (`docs/loop-time-findings.md`). | `drwho495/14906Decode`, `SPS-Robotics/DECODE-V2` |
| Camera: clusters and snapshots | Drive at the densest cluster, lane or heading rather than the nearest ball; fall back to a fixed path after 0.5–3 s; save Limelight snapshots for review. | We already drop stale results (`staleFrameSeconds`) and end `BallCollection` SKIPPED with no frame; new to us are picking which balls to plan by cluster, a fixed-path fallback and snapshots. | `kleongf/FTC_Decode`, `Dubble-Bubble/HIVEDecode`, `dani-Nayal/SolarFlare-Decode`, `FTC11329/11329-2026-repo`, `Xtendera/OverclockDecode` |

### Counts

Of the 23 repositories, at the commit each survey pinned; one repository can count in several rows and columns.

| | repos |
|---|---|
| ball signal in match code: a dedicated sensor / motor current or encoder speed only / none | 15 / 3 / 5 of 23 |
| break beams / I2C colour sensor / digital sensor of unstated type | 6 / 8 / 2 of 23 |
| REV 2m ToF / Brushland rangefinder read as analog / motor current / encoder speed | 2 / 2 / 3 / 2 of 23 |
| I2C ball sensors read on a throttle or only when needed | 8 of 8 |
| classify colour inside the robot | 7 of 23 |
| keep a count or per-position state / only a "full" flag | 12 / 5 of 23 |
| of those with a ball signal, filter in time (hold, lockout, settle) / by sample or loop count / latch only or none (2 do the first two) | 10 / 5 / 5 of 18 |
| sense each ball leaving (flywheel dip) / end a volley when sensors read empty / neither (one repo does both) | 2 / 3 / 19 of 23 |
| clear or decrement the count without checking the balls left | 6 of 23 |
| the ball signal changes intake or transfer power / rumbles / lights an LED / pre-spins / ends Auto collection | 11 / 9 / 9 / 3 / 13 of 23 |
| camera: ball pickup in a match Auto (5 Limelight, 1 webcam) / counts scored balls / code unused in matches | 6 / 2 / 3 of 23 |
| recorded a miss or false-count rate for a ball sensor | 0 of 23 |

### What didn't work for them

- **Spindexers:** 5 swapped a spindexer with colour sensing for an inline magazine or passthrough during the season;
  3 more have ball-sensor code that no match OpMode uses.
- **Sensor-ended shots:** 3 moved shot ends from a sensor to a fixed time ("use static delays instead of
  breakbeams"), and a fourth pads its "empty" with 400 and 500 ms waits. No commit says what failed.
- **Clearing the count blind:** 6 clear or decrement the count on a timer or mechanism position, so a jammed ball
  drops out of the count; one decrements every loop while feeding.
- **Motor current as the count:** 2 replaced a current "full" rule with beams; 3 still read current every loop for
  nothing (a separate hub command, about 1.4 ms on our Control Hub bench). Over-current alone gives one "stalling"
  bit: one team's current-based count could never reach 3, another's threshold went 4500, 4300, 5000, 3900 mA.
- **Filters:** the 5 that filter by sample or loop count get a filter time that moves with loop time (one window
  went 10, 20, 13, 5 samples); 2 bypass their own debounce; 2 report empty with a ball still inside and pad with fixed
  waits.
- **Colour classification:** 4 of the 7 that classify colour reworked it repeatedly or broke it: five weeks of
  rework; raw 0–65535 counts compared against 0–1 references (broken by the survey's reading of the code, untested on
  hardware); hysteresis and averaging deleted in the last push.
- **Rumble every loop:** 4 resend it every loop while full instead of on the change; the cost is unmeasured.

### Per team

| repository | sensors in match code | count | ball leaving | the signal drives | field balls |
|---|---|---|---|---|---|
| `23521-Desoto-Technix/2025-decode` | 4 beams, 2 OR'd | ordered fill sequence | beams read clear + 500 ms; Auto fixed delays | intake stop, rumble, Auto exit | none |
| `junkjunk123/RevAmped-Decode-V2` | digital presence, 2 REV colour | spindexer slots + 1, 1 s lockout | spindexer encoder; count cleared | auto transfer, spin-up, Auto exit at 1/2/3 | webcam blobs, disabled |
| `kleongf/FTC_Decode` | 3 beams | ordered, level-based | timed 400–800 ms; reset | LED, Auto exit | webcam cluster, far autos |
| `FTC11329/11329-2026-repo` | analog colour rangefinder, REV colour, beam | colour per slot, 2 reads | none; `hasShot()` unused | reverse at full, LEDs, sorting, Auto exit | Limelight NN, 0.5 s fallback |
| `Meta-Infinity/DecodePublicRelease` | 3 beams | edge count; 80/100 ms holds | timed; count −1 per loop | intake stop, rumble, LED, Auto exit and retry | none |
| `GoldenDragons772/Decode` | 3 beams | per beam, 250/400 ms hysteresis | timed 500 ms | intake idle, rumble, LED, Auto exit | none |
| `Dubble-Bubble/HIVEDecode` | none | none | timed | nothing | Limelight NN, biggest clump |
| `MOEbo-Sapiens/MOEbo-Sapiens-Decode` | none (dormant and test code only) | none | timed 800 ms | nothing | none |
| `DennNuke/ftcpanheya-worlds` | none (unused colour test) | none | timed | nothing | none |
| `Andrei0016/CNapSys-22586-Decode` | none; 3 ToF on origin/master later | none; later 7-read latch | timed 500 ms; later "clear" | nothing; later intake stop, LED | none |
| `GNCE/DECODEv2-Ditto` | 3 beams | from bottom, mode of 5 | count 0 + fixed waits | intake stop, rumble, Auto exit, skip volley under 2 | none |
| `drwho495/14906Decode` | transfer over-current flag | full bit, 650 ms | timed 650 ms | transfer stop, spin-up | none |
| `Xtendera/OverclockDecode` | ToF, 2 REV colour, intake current | 3 booleans, 160 ms polls | timed | intake stop, rumble, LEDs, Auto exit, ejector | Limelight blob, far auto |
| `Skeleton-Army/FtcRobotController-DECODE` | REV colour as distance, ToF | full, confirmed after 100/200 ms | timed; `justShot()` unused | rumble, Auto exit | Limelight script, far auto |
| `SPS-Robotics/DECODE-V2` | intake over-current flag | raw flag | timed | LED | none |
| `BeepBot99/CodeBloodedDecodeV3` | 1 digital sensor | full after 400 ms | timed | rumble, Auto exit | none |
| `ryalisuchir/Decode` | 3 analog colour rangefinders | per slot, latched full | timed kicks | intake block, spin-up, rumble, sorting, Auto exit | none |
| `dani-Nayal/SolarFlare-Decode` | 3 colour sensors, flywheel speed | per slot, 3 loops | dip in one Auto mode; Auto re-read and retry | intake stop, sorting, Auto exit | Limelight NN sweep; counts scored balls |
| `Andover-Robotics/4410-Decode` | 6 REV colour via 2 SRSHub boards | per slot, no filter | none | intake reverse, rumble, LED matrix, sorting | Limelight counts scored balls |
| `Scythxrz/TheFBI2025Decode` | encoders only | none | flywheel dip, paced fire | paced fire, LED | Limelight clusters, unused |
| `BaronClaps/22131-State` | REV colour, 1.35–1.45 in band | spindexer slots, 0.3 s lockout | timed | indexing | none |
| `OmicronX-30099/FTC-Decode-v2` | 1 beam, count never read | none | timed | nothing | Limelight blobs, test only |
| `BaronClaps/16010-Worlds` | REV colour as distance | full flag, debounce dead | timed | LED, Auto exit | none |

## TeleOp automation

Most of the 23 robots aim and set flywheel speed and hood from odometry on their own but leave firing and the pose to
the drivers: 16 feed with no ready check and 14 have no shot mode that works without a good pose. Our solver, ready
gate and shoot-on-the-move (leading the aim for the robot's motion) run only in SOTM Test; BioBuzz Tele only drives.
Port that gate to TeleOp first, then the ideas below. No survey result reports a measured effect of any of them.

### What to copy

| idea | what it is | why it fits us | seen in |
|---|---|---|---|
| Carry the Auto pose | Auto saves its last good pose every loop, skipping a zero pose, and TeleOp starts from it. | `Robot` puts the follower at a placeholder pose at every INIT (`architecture/core/Robot.java:23-24`) and SOTM Test starts from a dashboard pose: nothing carries the Auto pose. `junkjunk123/RevAmped-Decode-V2` records the Pinpoint reporting (0, 0, 0) at stop (f0c64dd). A static is lost on an app restart and a file isn't; a missing file should raise a fault, not start at (0, 0, 0). | 22 of 23; skip a zero pose: `junkjunk123/RevAmped-Decode-V2`, `DennNuke/ftcpanheya-worlds`, `Xtendera/OverclockDecode`, `SPS-Robotics/DECODE-V2`; file plus a clearing OpMode: `FTC11329/11329-2026-repo` |
| Say why a shot is refused | A press the gate refuses rumbles once and the Driver Station (DS) shows why; a second press within about 1 s forces the shot. | SOTM Test already shows `Valid` and the `ShotSolver` reason on the DS, but a held trigger the gate refuses gives the driver nothing to feel, and 4 of the 7 gated repos have no way past a wrong gate. | rumble: `Xtendera/OverclockDecode` (not ready), `Skeleton-Army/FtcRobotController-DECODE` (outside the launch zone, with the 1 s second press); a way past: `drwho495/14906Decode` (held button), `FTC11329/11329-2026-repo` (panic toggle) |
| Pose-free shot mode | One toggle points the turrets at fixed angles and sets a fixed close shot, with a DS badge while it is on. | In SOTM Test both turrets, flywheels and hoods follow the pose, so a lost pose ends scoring. The fixed states exist (`Turret` BACK/LEFT/RIGHT, `Shooter` ON and hood POSITION); only the toggle and badge are missing. A mode the driver picks and the DS shows is not a silent fallback, so it keeps our fail-fast rule. | 9 of 23, e.g. `23521-Desoto-Technix/2025-decode` (with a DS badge), `FTC11329/11329-2026-repo`, `Andover-Robotics/4410-Decode`, `BaronClaps/22131-State` |
| Stop the intake at a count | Intake and transfer stop a short settle after the count reaches a set number, and the gamepads rumble once on that change. | Transfer Thread Count Test already stops its transfer motor at `stopAtCount` from `TransferCounter`'s 10 ms thread; TeleOp has neither. The count only rises, and only for balls moving from `analog0` to `analog1` (`CLAUDE.md`), so TeleOp must clear it after a volley or an outtake. | 11 act when full, 9 rumble; e.g. `Meta-Infinity/DecodePublicRelease` (100 ms settle, one rumble), `GoldenDragons772/Decode` |
| Trim the aim point | Each d-pad press moves the target a fixed distance in field x or y, mirrored for the alliance. | One nudge corrects both turrets from anywhere; an angle trim is right only where it was set, and ours is one: `ShotTuning`'s `angleTrimDeg`, per shooter, dashboard only. | `MOEbo-Sapiens/MOEbo-Sapiens-Decode` (2 in, replaced a ±3° turret trim after Worlds), `drwho495/14906Decode` (1 in) |
| Auto-fire on a toggle | Fire when the shot is valid, balls are held and no volley is running, checked every loop. | `Valid` already says where a shot works, so no zone polygons. Their two show why it must recheck every loop and require balls: one fires only on entering a zone; the other disabled its kicker after auto-fire kicked repeatedly (886c8f8) and debounced zone exit by 0.3 s (668e2e8). | `Skeleton-Army/FtcRobotController-DECODE` (on by default), `GNCE/DECODEv2-Ditto` |
| Back to intake after a volley | When the volley ends, the robot closes the path to the shooters and runs the intake with no button. | Shoot-then-collect becomes one press, many times a match. | `MOEbo-Sapiens/MOEbo-Sapiens-Decode`, `BaronClaps/22131-State`, `BaronClaps/16010-Worlds`, `GNCE/DECODEv2-Ditto`, `junkjunk123/RevAmped-Decode-V2` |
| A ready cue | A rumble on the operator's gamepad, or a light colour, while the shooters are ready. | SOTM Test computes `ready` per shooter but shows it only as DS telemetry; drivers watch the field, and two shooters need one "both ready" cue. | `Andover-Robotics/4410-Decode`, `BaronClaps/16010-Worlds` (one light: full × ready in 4 colours), `OmicronX-30099/FTC-Decode-v2` |

### Counts

| what the TeleOp does | repositories, counted from the 23 results (of 23 unless a smaller group is named) |
|---|---|
| Aims a turret at the goal from odometry | 20; 1 more holds a turret heading toward a driver-picked spot, 2 turn the drivetrain. None aims from a camera at the surveyed commit; 2 built camera aiming and dropped it |
| Leads the aim for robot motion: shoot-on-the-move / a robot turn-rate term | 16 / 11 |
| Flywheel and hood from a 1-D distance table / fit / 2-D field-position table / physics / presets | 13 / 4 / 2 / 2 / 2 |
| Flywheel always on / switched by ball state or zone / switched by the driver | 11 / 4 / 8 |
| One press fires a volley / feeds while a button is held / fires by itself | 15 / 9 / 2 (1 does both of the first two) |
| Waits for readiness before feeding | 7: 4 also check turret or heading, 3 have a way past it, 2 rumble on a refusal. 7 of the other 16 check readiness only in Auto |
| Slows or spaces the feed for long shots / returns to intake after a volley | 10 (our hood droop points address the same flywheel sag) / 5 |
| Intake or transfer acts by itself when full / automatic unjam | 11 / 1 |
| Field-centric by default / heading lock / automatic position hold / drive to a pose / slower during a volley | 8 (4 more on a toggle) / 9 / 1 / 4 (our rule keeps pathing in Autonomous) / 2 |
| Carries the Auto pose into TeleOp | 22: 2 through a file, 4 skip a zero pose, 7 carry turret state too, 3 have OpModes that only set the TeleOp start |
| Pose reset button at a field landmark / camera correction, automatic / on a button | 22 / 3 / 4 |
| Shot mode that ignores the pose / trims while auto-aim runs | 9 / 15 (2 move the aim point in field coordinates, 4 step every loop while held) |
| Rumble / lights / telemetry only / a ready cue | 10 (9 when full) / 12 / 6 / 3 |
| Fire button on driver 1 / driver 2 / either | 18 / 3 / 2; 3 competition TeleOps use one gamepad |

### What didn't work for them

- **Camera pose correction.** 8 built it, then turned all or part of it off in TeleOp or never called it there (e.g.
  `Skeleton-Army/FtcRobotController-DECODE` 9322ab3 "disable limelight relocalization"). Of the 7 still using it, 5
  reuse a stale tag, frame or yaw, corrupt the heading, or correct only the aim pose.
- **Position hold.** 3 held position automatically when the sticks were released and removed it; 4 more wrote a hold
  and never used it. Only `Skeleton-Army/FtcRobotController-DECODE` runs one.
- **No ready check, or no way past it.** 16 feed whenever the button says, 7 of them despite a readiness check in
  Auto. Of the 7 that gate, 4 have no way past it, so a wrong gate blocks every shot.
- **Automation handed back to the drivers** in 4: an in-zone auto-ready removed the day it was added, an auto-drive to
  the shooting zone set back to off by default five days after it was turned on, pose carry and a camera reset removed
  after Worlds, and a close/far choice by field position moved to a button. 2 commented out an intake stop when full.
- **Feedback that didn't last.** 3 removed rumble or lights during the season (one in its final commit, reason not
  recorded), 3 have rumble or blink code that never runs, and 4 re-send the rumble every loop instead of on a change.
- **Trims.** 4 of 15 add a step every loop while the button is held, so the rate follows loop time; 3 keep trims in
  statics that carry into later OpModes; one goal nudge did nothing at an event because the new `Pose` was discarded.
- **Aiming a loop late.** 9 compute aim or shooter targets before that loop's odometry update. So does our `Turret`
  AUTOAIM state (read from source): it aims in `read()` (`biobuzz/modules/Turret.java:110-116`), which runs before
  `follower.update()` (`architecture/core/EnhancedOpMode.java:305-308`), so it lags one loop, 15.3 ms mean in
  `docs/loop-time-findings.md`. Turret Aim Test aims this way; SOTM Test's default aim solves in `gameLoop()`.

### Per team

lead = shoot-on-the-move; yaw = turn-rate term; incl. aim = also checks turret or heading; acts = reacts when full.

| repository | aim | setpoints, flywheel | firing | when full | drive assists | pose in TeleOp | fallback | feedback |
|---|---|---|---|---|---|---|---|---|
| `23521-Desoto-Technix/2025-decode` | turret, lead, yaw | 1-D table, always on | press, no gate | acts | heading lock, to pose | carried | pose-free mode | rumble |
| `junkjunk123/RevAmped-Decode-V2` | turret, heading only, yaw | presets, auto on/off | press, no gate | acts | - | carried, guarded | pose-free mode | rumble, lights |
| `kleongf/FTC_Decode` | turret, lead | 1-D table, always on | press, no gate | - | heading lock, to pose | carried, vision button | trims | lights |
| `FTC11329/11329-2026-repo` | turret, lead, yaw | 1-D table, driver on/off | press, gate incl. aim, override | acts | brake | carried (file), vision button | pose-free mode | lights |
| `Meta-Infinity/DecodePublicRelease` | turret, lead, yaw | 1-D table, driver on/off | hold, no gate | acts | heading lock | carried, vision button | trims | rumble, lights |
| `GoldenDragons772/Decode` | turret, lead, yaw | physics, always on | press + hold, no gate | acts | - | carried, vision auto | - | rumble, lights |
| `Dubble-Bubble/HIVEDecode` | turret | physics, driver on/off | hold, no gate | - | slow in volley | not carried | trims | telemetry |
| `MOEbo-Sapiens/MOEbo-Sapiens-Decode` | turret, yaw | 1-D table, always on | press, no gate | - | heading lock | carried | trims | telemetry |
| `DennNuke/ftcpanheya-worlds` | turret, lead, yaw | 1-D table, always on | hold, no gate | - | heading lock | carried, guarded | - | lights |
| `Andrei0016/CNapSys-22586-Decode` | turret, lead | 1-D table, auto on/off | press, gate | - | - | carried | trims | telemetry |
| `GNCE/DECODEv2-Ditto` | turret | 1-D table, always on | press, gate, auto-fire | acts | - | carried, vision auto (one of 2 TeleOps) | trims | rumble |
| `drwho495/14906Decode` | drivetrain, lead | fit, auto on/off | hold, gate, override | acts | heading lock | carried | trims | telemetry |
| `Xtendera/OverclockDecode` | turret, lead | 1-D table, always on | hold, gate incl. aim | acts | - | carried, guarded | pose-free mode | rumble, lights |
| `Skeleton-Army/FtcRobotController-DECODE` | turret, lead | fit, always on | press, gate incl. aim, override, auto-fire | - | auto hold, to pose, slow in volley | carried | trims | rumble |
| `SPS-Robotics/DECODE-V2` | turret, lead, yaw | 1-D table, driver on/off | hold, no gate | - | heading lock | carried, guarded | pose-free mode | lights |
| `BeepBot99/CodeBloodedDecodeV3` | turret, lead, yaw | 2-D table, driver on/off | hold, no gate | - | heading lock | carried, vision auto | trims | rumble |
| `ryalisuchir/Decode` | turret, lead, yaw | 2-D table, auto on/off | press, no gate | acts | to pose | carried | pose-free mode | rumble |
| `dani-Nayal/SolarFlare-Decode` | turret, lead, yaw | fit, always on | press, no gate | acts | - | carried | trims | telemetry |
| `Andover-Robotics/4410-Decode` | turret, lead | 1-D table, driver on/off | press, no gate | acts | - | carried, vision button | pose-free mode | rumble, lights |
| `Scythxrz/TheFBI2025Decode` | drivetrain | 1-D table, always on | hold, gate incl. aim | - | heading lock | carried (file) | - | lights |
| `BaronClaps/22131-State` | turret | presets, driver on/off | press, no gate | - | - | carried | pose-free mode | telemetry |
| `OmicronX-30099/FTC-Decode-v2` | turret, lead | 1-D table, always on | press, no gate | - | - | carried | trims | lights |
| `BaronClaps/16010-Worlds` | turret | fit, driver on/off | press, no gate | - | - | carried | pose-free mode | lights |

## Architecture and tooling

Our framework already avoids most of the structural problems that recur in these 23 codebases: one base OpMode runs
the per-loop housekeeping, Modules are built per OpMode and every State binding is reset at INIT, each State's Module
becomes a command requirement, and `FieldPose.forAlliance` mirrors every pose authored through it. One recurring bug
(9 of 23) is ours too: `Turret`'s AUTOAIM aims in `read()`, which `EnhancedOpMode` runs before `follower.update()`, so
it aims from the previous loop's pose (a loop measured 15.3 ms on average and 19.4 ms while driving on the Betta bot,
`loop-time-findings.md`); move it into `write()`, where SOTM already is. What is worth adding is mostly Autonomous
sequencing (end pickups on the transfer counter, shoot before stopping, carry the pose into TeleOp) and tooling.

### What to copy

| idea | what it is | why it fits us | seen in |
|---|---|---|---|
| End pickups on the transfer counter | Race each collection leg or dwell against "chute full", inside a minimum and a maximum time (at least 0.5 s, at most 1.5 s in `BaronClaps/16010-Worlds` `Close.java`). | `TransferCounter` counts on a 10 ms thread (so far only in Transfer Thread Count Test) and Ivy has races; with 40 pollen a match, leaving a pile the moment the chute is full is free cycle time. | `GNCE/DECODEv2-Ditto`, `Xtendera/OverclockDecode`, `BeepBot99/CodeBloodedDecodeV3`, `BaronClaps/16010-Worlds` and 6 more |
| Shoot before the robot stops | While driving the scoring leg, aim and spin up for its end pose, and open the feed when the leg is nearly done (Pedro's path parameter t, 0 to 1, above 0.3 to 0.85 in one repository and 0.85 in another). | Our turrets aim continuously (`Turret` AUTOAIM and SOTM). `PathCommands.remainingBelow` can open the blocker, but it fires early on S-curves (CLAUDE.md), so use it on straight or gentle legs. | `BaronClaps/16010-Worlds`, `BeepBot99/CodeBloodedDecodeV3`, `Xtendera/OverclockDecode`, `dani-Nayal/SolarFlare-Decode` |
| Pose file from Autonomous to TeleOp | Autonomous writes its end pose to a file on the hub and TeleOp loads it at INIT; a small OpMode clears it. Neither repository checks the pose it loads: add a refusal of a missing or zeroed pose (3 other repositories skip saving a zeroed one), and let only Autonomous write it. | `Robot` puts the follower at a placeholder pose every INIT and `BioBuzzTele` sets none, while `Turret` aims from odometry. Statics don't survive deploySloth or an app restart; a file does. It reverses our no-carry rule (CLAUDE.md), so a bad file must show on the Driver Station, not start TeleOp at (0, 0). | `FTC11329/11329-2026-repo`, `Scythxrz/TheFBI2025Decode` |
| Decide the last cycle on the clock | Before each extra cycle, check the match time and park instead if the cycle can't finish (`conditional(time < 27.5 s, cycle, park)`). | Racing the whole auto against a timeout cuts a cycle mid-way and strands the balls it collected. Ivy 1.1.1 has `Commands.conditional` (javap). | `BaronClaps/16010-Worlds`, `junkjunk123/RevAmped-Decode-V2` (28.5 s); `Skeleton-Army/FtcRobotController-DECODE` starts another cycle while any time is left |
| Shot-table recorder | In the shooter tuning OpMode, one button (a dashboard toggle in one repository) saves the current distance, rpm and hood as a row and telemetry prints the rows as paste-ready code; one repository also writes them to `/sdcard`. | We fill two tables plus droop points by reading telemetry and typing values (`sotm-tuning.md`), and dashboard edits are lost on deploySloth or an app restart. | `Meta-Infinity/DecodePublicRelease`, `Xtendera/OverclockDecode` |
| Build steps when they start | Build pose-, count- or vision-dependent steps at run time. Ivy 1.1.1's `Commands.lazy(Supplier<Command>)` calls its supplier and starts the result in its start hook, and declares no requirements (javap of `core-1.1.1.jar`): give it the modules with `.requiring(...)`. | 7 of 23 fixed a step's content when it was built, not when it ran. Our ball collection already plans when it starts; later steps should too. | `Scythxrz/TheFBI2025Decode` (legs from the live pose), `Skeleton-Army/FtcRobotController-DECODE` (`DeferredCommand`), `junkjunk123/RevAmped-Decode-V2` (own `Lazy` for Ivy) |
| Autonomous assembled at INIT | A Driver Station menu during INIT picks a preset routine or its segments (and in 2, start delays) and shows the result. | Matches each alliance partner without a redeploy; the segments would be `RobotActions` and `PathCommands` builders. Alliance stays a dashboard setting made before INIT (CLAUDE.md). | `Andover-Robotics/4410-Decode`, `drwho495/14906Decode`, `Skeleton-Army/FtcRobotController-DECODE`, `dani-Nayal/SolarFlare-Decode`, `Scythxrz/TheFBI2025Decode` |
| Match log file | Write pose, states, counter and shot events each loop to a file for review after a match. | Our only per-loop file log is Ivy Path Test's CSV, so nothing records what the counter or turrets did in a match. Measure its loop cost first. | `Skeleton-Army/FtcRobotController-DECODE` (PsiKit logs for AdvantageScope); `Xtendera/OverclockDecode` saves Limelight snapshots during Autonomous |
| Pit self-test | An OpMode runs each mechanism for a few seconds and prints PASS or FAIL from its current and encoder. | Two flywheels, two turrets, the OctoQuad and the Limelight to check between matches; our hub bench needs a laptop and adb. | `junkjunk123/RevAmped-Decode-V2` (`AutomatedTest`); diagnostic OpModes in 3 more |

### Counts

| what | repositories, of 23 unless stated |
|---|---|
| Path follower in competition autos | Pedro Pathing 20 (one vendored as source), own point-to-point follower 2, Road Runner 1.0 in 1 |
| Command or sequencing library | SolversLib 6, Ivy (Pedro Pathing's scheduler) 5 plus 1 declared but unused, NextFTC 3, FTCLib 2, Road Runner Actions 1, own or none 6 |
| Dashboard | slothboard (the FTC Dashboard fork for Sloth) 9, FTC Dashboard 6, neither 8; Panels (another web dashboard) on the classpath in 22 |
| Live tuning of competition constants | Dashboard `@Config` 11, Panels `@Configurable` 6, both 2, none 4 |
| Hot reload (Sloth) | 14, plus 1 with the plugin applied and the runtime commented out as broken |
| Base OpMode | team base for every OpMode 5, for some OpModes 4, library base only 7, plain SDK OpMode 7 |
| Robot container | object built per OpMode 11, singleton or all-static 6, none 6 |
| Hardware wrappers | library 11, own 6, raw SDK devices 6 |
| Autonomous structure | one command or action tree 18, list of steps 3, hand-written state machine 2 |
| Alliance mirroring | one helper over poses written for one alliance 17 (1 of them later split two autos into red and blue path files); hand-copied red and blue files 6 |
| Routine or segments picked on the Driver Station during INIT | 6 |
| Pickup ends on a ball-full or ball-count signal | 10, plus 2 that retry a step the sensors say failed |
| Routine tied to the match clock | 8, of which 3 check the clock at each cycle's start |
| Pose carried to TeleOp | 22: static field or object 19, file 2, the Pinpoint keeping its own pose 1; 3 of the 21 that save one skip a zeroed pose, and neither file reader checks the pose it loads |
| Tooling | shot-table OpMode 12 (2 record the rows), pit self-test or diagnostic OpMode 4, team docs 4, match log file 1, unit tests 1 |

### What didn't work for them

- **State that outlives the OpMode** (20 of 23): statics and singletons carried modes, trims and poses into the next
  OpMode; 4 mirror or shift alliance poses in statics, and 3 of those flip or shift them again at every later INIT.
- **Aim from the previous loop's pose** (9 of 23): shot logic ran before the odometry update. Ours does too, above.
- **No shared loop**: 18 of 23 have no team base used by every OpMode (7 use plain SDK OpModes), 5 set up bulk
  caching or the voltage read in some OpModes and not others, and 6 run the scheduler or odometry update twice in
  some loops (one of them plausible, not confirmed).
- **Unmanaged command conflicts**: of the 18 with a command or action library, 3 use requirements throughout, 8 on a
  few commands and 7 not at all, so commands and OpMode code overwrite each other's setpoints.
- **Steps fixed when built** (7 of 23): 6 Autonomous routines captured sort orders, a deadzone or paths from the start
  pose when they were built, and 1 TeleOp binding tests the pose once, when it is bound.
- **Live tuning that does nothing** (6 of 23, plus 1 presumed): values copied in a constructor or at class load, or
  reset at INIT. `bindTunable` avoids it for us, unless a `Tuning` value is passed to a State's constructor instead.
- **Hand-copied red and blue autos** (6 of 23, 4 already drifted apart) and **Autonomous waits with no timeout**
  (2 of 23): `FieldPose.forAlliance` and racing every auto against `PathCommands.waitMs` cover both for us.
- **Tried and dropped**: 4 of 23 replaced their command framework mid-season (1 more moved to SolversLib and Photon
  and reverted the next day), 2 dropped Pedro for their own follower, and 2 abandoned YAML-driven autos after a
  week's work.

### Per team

| repository | base OpMode · container | follower · commands · dashboard · reload | Autonomous | worth a look |
|---|---|---|---|---|
| `23521-Desoto-Technix/2025-decode` | NextFTCOpMode · global `object`s | Pedro · NextFTC · Panels · none | one SequentialGroup each; `Pose.mirror()` plus blue overrides | `Tube` state machine that sets outputs on entry |
| `junkjunk123/RevAmped-Decode-V2` | own base · `Robot.INSTANCE` | Pedro · Ivy (local jar) · FTC Dashboard · none | Ivy sequence over a path queue, park decided per cycle; lazy `FuturePose` mirror, later split files | `AutomatedTest`, own `Lazy` |
| `kleongf/FTC_Decode` | plain OpMode · two containers | Pedro · own state machine · FTC Dashboard · none | one state machine per auto, full flag with min/max time; copied red files | pit-check group `!` |
| `FTC11329/11329-2026-repo` | plain OpMode · 970-line `Robot` | vendored Pedro · none · Panels · Sloth | list of steps that chain start poses; static poses flipped in place | `docs/` postmortems, end-of-Auto file |
| `Meta-Infinity/DecodePublicRelease` | TeleOp base · `Robot` | Pedro · none · FTC Dashboard · none | string-switch state machine; copied blue files | `InterpolationTuner` |
| `GoldenDragons772/Decode` | FTCLib `CommandOpMode` · `RootSubsystem` | own PID-to-point · FTCLib · FTC Dashboard · plugin only | command groups; `MirrorHelper` | gamepad binding table |
| `Dubble-Bubble/HIVEDecode` | plain OpMode · none | own SquID follower · FTCLib (autos) · FTC Dashboard · none | command groups; copied red and blue files | SquID controller |
| `MOEbo-Sapiens/MOEbo-Sapiens-Decode` | Tele and Auto bases · `Robot` | Pedro · Ivy · slothboard · Sloth | timed Ivy sequences; red subclasses mirror each pose | swerve tuners |
| `DennNuke/ftcpanheya-worlds` | TeleOp base · eager singleton | Pedro · SolversLib · slothboard · Sloth | timed command groups; `Pose.mirror()` plus red overrides | saved pose skipped near zero |
| `Andrei0016/CNapSys-22586-Decode` | plain OpMode · `Robot` | Pedro · none · Panels · none | `List<Step>` with lambda exits; copied blue files | `shooterCalibration` |
| `GNCE/DECODEv2-Ditto` | own base · `MyRobot` | Pedro · SolversLib · Panels · none | race path against `storage==3`, park at 29.5 s; `Pose.mirror()` | per-OpMode subsystem lists |
| `drwho495/14906Decode` | plain LinearOpMode · `RobotManager` | Pedro · none (blocking loops) · slothboard · Sloth | command list from 12 INIT presets; `getFixedPose` | `ShooterTuner`, `BrokenBot` |
| `Xtendera/OverclockDecode` | SolversLib `CommandOpMode` · singleton | Pedro · SolversLib · Panels · Sloth | race `isFull()`, repeat to 29.5 s, pre-aim at leg end; `fromRed()` | `VelocityTableTuner` |
| `Skeleton-Army/FtcRobotController-DECODE` | own base · none | Pedro · SolversLib · slothboard · Sloth | one OpMode with DS prompts, deferred paths; `getRelative()` | PsiKit logging, JUnit test |
| `SPS-Robotics/DECODE-V2` | NextFTCOpMode · singleton subsystems | Pedro · NextFTC · Panels · Sloth | routine class with three-line red and blue subclasses | `FlywheelTuning` |
| `BeepBot99/CodeBloodedDecodeV3` | own base · subsystem fields | Pedro · Ivy · slothboard · Sloth | fire at t > 0.85, race `hasThree`; copied blue file | one tuner per gain |
| `ryalisuchir/Decode` | SolversLib `CommandOpMode` (TeleOp) · `Halo` | Pedro · SolversLib · slothboard · Sloth | command groups; `alliancePose()` plus blue turret offsets | stall-aware path command |
| `dani-Nayal/SolarFlare-Decode` | plain LinearOpMode · static `Inferno` | Pedro · own commands · FTC Dashboard · none | segment builder in INIT, 29.3 s watchdog; `getPose()` | Diagnostic TeleOp |
| `Andover-Robotics/4410-Decode` | plain LinearOpMode · `Bot` singleton | Road Runner 1.0 · RR Actions · slothboard · Sloth | INIT segment menu with delays; pose-map lambda | per-section loop timer |
| `Scythxrz/TheFBI2025Decode` | SolversLib `CommandOpMode` · singleton | Pedro · SolversLib · Panels · Sloth | INIT sequence menu, legs from the live pose; `forAlliance` | `DriveToPose`, `PiecewiseHeading` |
| `BaronClaps/22131-State` | own base (TeleOp bypasses it) · `Robot` | Pedro · Ivy (Autonomous) · slothboard · Sloth | Ivy sequences, 29.25 s hold (may not stop the sequence); mirror in the paths constructor | `BiLinearTuner` |
| `OmicronX-30099/FTC-Decode-v2` | NextFTCOpMode · global `object`s | Pedro · NextFTC · Panels · Sloth | SequentialGroup scripts; copied red and blue files | none |
| `BaronClaps/16010-Worlds` | own base (Autonomous) · `Robot` | Pedro · Ivy · slothboard · Sloth | fire at t > 0.4, last cycle only before 27.5 s; mirror in the paths constructor | `BiLinearTuner` |

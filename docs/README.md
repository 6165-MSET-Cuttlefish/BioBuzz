# Docs

What the team has measured, researched and decided that the code doesn't show. Read the page for a topic before
working on it. `CLAUDE.md` holds only one-line rules and points here.

## Pages

**The robot and the game**

| page | read it before |
|---|---|
| [`robots-findings.md`](robots-findings.md) | writing code for the BIOBUZZ robot's shooters, turrets or cameras, running anything on the Cuttle bot, or setting a robot's size in code or a `.pp` file |
| [`game-findings.md`](game-findings.md) | packing for an event, adding anything that talks over Wi-Fi or USB, changing HIVE numbers in `BioBuzzField`, or planning routes under the HIVE |
| [`camera-findings.md`](camera-findings.md) | changing the AprilTag webcam, its mount, cable, USB port, resolution or exposure, or trusting a cell-tip verdict |

**Shooting**

| page | read it before |
|---|---|
| [`sotm-tuning.md`](sotm-tuning.md) | tuning SOTM Test, or changing `ShotTuning` |

**Hubs, SDK and loop time**

| page | read it before |
|---|---|
| [`sdk-hub-findings.md`](sdk-hub-findings.md) | writing a hardware wrapper, sensor thread or voltage compensation, or trusting a hub's encoder velocity |
| [`hub-bench-findings.md`](hub-bench-findings.md) | changing the planner, hub or I2C traffic, odometry, the Limelight, background threads or stop paths |
| [`loop-time-findings.md`](loop-time-findings.md) | trying to shorten the loop, or adding per-loop hub commands |
| [`photon-bench.md`](photon-bench.md) | using Photon in a robot OpMode, or running its hub bench |
| [`photon-offline-results.txt`](photon-offline-results.txt) | trusting the offline Photon suites: their last recorded run |

**Framework, pathing and vision**

| page | read it before |
|---|---|
| [`framework-findings.md`](framework-findings.md) | changing `OptimizationToggles` for a match, feeding a Module per-loop setpoints, relying on Module order or `BatteryVoltage`, or deleting framework code that looks unused |
| [`pedro-findings.md`](pedro-findings.md) | building paths or headings by hand, changing the follower setup, or running or bumping AutoTune |
| [`ball-planner-findings.md`](ball-planner-findings.md) | changing the ball route planner, ball selection or the number of balls planned |
| [`limelight-findings.md`](limelight-findings.md) | changing `LimelightSync` or `LimelightBallSource`, moving the Limelight, or reaching it from a laptop |
| [`hsv-tuning-prompt.md`](hsv-tuning-prompt.md) | retuning ball-detection HSV ranges for a venue, ball batch or ball type |

**Tools**

| page | read it before |
|---|---|
| [`toolchain-findings.md`](toolchain-findings.md) | changing AGP, Gradle or a dependency, or when deploySloth, a hardware config or adb misbehaves |
| [`dashboard-findings.md`](dashboard-findings.md) | using the dashboard's Hardware Config View or Enable/Disable switch, replacing or bumping the dashboard, or writing a test OpMode's `@Config` knobs |

**Files**

| page | read it before |
|---|---|
| [`paths/`](paths) | editing an auto's poses: one Pedro visualizer export per auto |
| [`calibration-chessboard-3in-letter.pdf`](calibration-chessboard-3in-letter.pdf) | recalibrating a camera lens: the board to print |
| [`legal/`](legal) | nothing: licence texts the SDK ships |

## Recording a finding

Record it when a session learns something the code doesn't show and the next person would otherwise have to learn
again:

- a measurement on the robot, a hub or a bench rig;
- research: other teams' code, vendor pages, the SDK's source, a simulation;
- a part: its specs, wiring, where to buy it, its CAD, what it was compared against;
- an SDK, hub or library behaviour that surprised us;
- a dead end, with why it failed.

Where it goes:

- **It fits a page above:** add it there.
- **New topic:** a new page, `<topic>-findings.md` for facts or `<topic>-tuning.md` for a procedure, and a row in the
  table above.
- **A rule or footgun every session must know:** one line in `CLAUDE.md` that points at the page. Nothing else goes
  in `CLAUDE.md`.
- **A script that produced the numbers:** `scripts/`, with the command to rerun it written on the page.

How to write it:

- Start with the conclusion and what to do about it. Details and tables come after.
- Say how each fact is known: measured (date, robot or hub, firmware and SDK versions), read from source (file and
  line, or URL), simulated (the assumptions), or someone's claim we haven't checked.
- List what is still unknown and the test that would settle it.
- Write dates in full (2026-11-10), never "last week".
- When a finding is replaced, rewrite it. History is in git.
- This repository is public: no people's names, no network names, passwords or addresses other than the robot's own
  defaults, and no pointers to anything private.

Commit the page to main in the same session, on its own if nothing else changed: `Record <what was found>`.

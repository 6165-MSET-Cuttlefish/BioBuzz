# Offline bench checks

Desktop checks for `run_bench.py`'s check of adb's server port, its saved-configs listing and `--list-configs`,
Bench: Dashboard Off, Bench: OctoQuad's bank phase, Bench: Turret Zero, Bench: Photon's two-hubs-at-once phase,
`--usb`, `session.py` and the benches in `scripts/bench/benches/` ([`../benches/README.md`](../benches/README.md)).
They need no hub: the hub, adb, `lsof` and the dashboard are stubs, nothing binds or connects to adb's port 5037 or the
emulator ports 5554-5585, and nothing reaches the network. They catch regressions before a hub run; they
don't replace it. The rest of the Photon block has its own: [`../photon-offline/README.md`](../photon-offline/README.md).

## Before running

`python3` 3.9+ on `PATH`; for the dashboard check, a JDK's `javap` on `PATH` or in `JAVA_HOME`. Everything they write
goes to `out/` here (gitignored).

## Suites

Run from the repo root.

| suite | command | last line when it passes |
|---|---|---|
| checks: the adb-port verdict, saved configs, Dashboard Off's verdicts, the default run's benches | `python3 -I scripts/bench/offline/test_station_checks.py` | `... passed, 0 failed` |
| `bare_hub.sh`'s adb-port check and the one it writes into `REINSTALL.sh`, run with stub `lsof` and `adb` | `python3 -I scripts/bench/offline/test_bare_hub_preflight.py` | `20 passed, 0 failed` |
| dry run: the runner's connect, `--list-configs` and Bench: Dashboard Off end to end against a fake hub, its Wi-Fi HTTP requests included (about 40 s) | `python3 -I scripts/bench/offline/dryrun_stations.py` | `... passed, 0 failed` |
| dry run: the default step list end to end against `fake_hub.py` (a fake adb with USB devices, `connect`, `forward` on a picked port, `--list`, `--remove` and a device that leaves and returns without its forwards, plus slothboard's socket and the hub's HTTP ports, all on loopback; about 4 min). Over Wi-Fi, on a bare hub and on one with a webcam, the Limelight and an Expansion Hub, the adb calls (in order and byte for byte, polls aside: how many `adb devices`, pidof and logcat reads a wait makes follows how fast the fake answers on a loaded Mac; on the second its sockets and HTTP requests too) and the step selections must be the ones in `fixtures/`, recorded from the runner before `--usb` (`ea69f20`); with `--usb`: no `adb connect`, every socket through a forward, only its own forwards removed, a reboot's and a pulled cable's forwards made again, two USB devices stopping before any device command, an unreadable hub socket table reported as not judged; a Ctrl-C in the first bench putting the config back and removing only its own forwards; a hung RC page forward skipping only that step; deploySloth killed when the runner alone gets Ctrl-C; `--hub-serialno`: another hub after a reboot getting nothing more, not even the config put back | `python3 -I scripts/bench/offline/dryrun_default.py` | `... passed, 0 failed` |
| `bare_hub.sh --usb` end to end from a copy, with stub `lsof`, `git`, `gradlew` and `adb` (about 1.5 min): no `adb connect`, the device rules, the dashboard port read from the hub's socket table, `REINSTALL.sh` run over the cable, the `--build --usb` fallbacks; `--install-only --expect-serialno` reading `ro.serialno` before the backup and before installing, and `REINSTALL.sh` refusing another hub; without `--usb`, its adb calls, runner flags and `REINSTALL.sh` byte for byte the ones recorded from the script before `--usb` (`fixtures/bare_hub_wifi.json`) | `python3 -I scripts/bench/offline/test_bare_hub_usb.py` | `... passed, 0 failed` |
| `session.py` against `fake_hub.py` with every stage's runner stubbed (about 30 s): the plan for a bare hub, an Expansion Hub on USB, one on RS485 and every optional device; every question before the first stage and none after; a failing stage, one that raises, one past its time limit and a hub reboot don't stop the chain; another hub, a hijacked adb port and an Expansion Hub the statement didn't cover do, touching nothing after; Ctrl-C during each stage (and a real child, and one still starting) still puts the hub back, its adb commands with Ctrl-C ignored; a closed terminal ends no stage; the put-back with the app down, an unreadable dashboard switch, a failing `adb forward --list`, others' forwards; another hub before the install; the soak gate; every prompted bench in Needs a person with a command `run_bench.py` parses; `bare_hub.sh --install-only`; `run_bench.py --probe-only` and `--photon-soak-hours` | `python3 -I scripts/bench/offline/test_session.py` | `... passed, 0 failed` |
| the dashboard members the bench uses, by `javap` on the pinned slothboard AAR | `python3 -I scripts/bench/offline/check_dashboard_api.py` | `18 passed, 0 failed` |
| Bench: OctoQuad's bank-phase verdicts, good and broken results | `python3 -I scripts/bench/offline/test_octoquad_bank.py` | `... passed, 0 failed` |
| `run_octoquad` end to end with the hub stubbed | `python3 -I scripts/bench/offline/dryrun_octoquad.py` | `... passed, 0 failed` |
| Bench: Turret Zero's verdicts, good and broken results (`turret_zero_fixtures.py` builds them) | `python3 -I scripts/bench/offline/test_turret_zero_checks.py` | `... passed, 0 failed` |
| `run_turret_zero` end to end with the hub stubbed and the answers scripted: prompt order, config, params, the default run unchanged | `python3 -I scripts/bench/offline/dryrun_turret_zero.py` | `... passed, 0 failed` |
| Bench: Photon's two-hubs-at-once phase: the overlap reading, its record-only line and table | `python3 -I scripts/bench/offline/test_hub_overlap.py` | `... passed, 0 failed` |
| the benches in `scripts/bench/benches/` ([`../benches/README.md`](../benches/README.md)): each declaration, `order.txt`, their flags and `--help`, the typed `--s3-ports` statement, the session's Needs a person, a stub bench on `fake_hub.py`; then each bench's own file in `benches/` | `python3 -I scripts/bench/offline/test_benches.py` | `... passed, 0 failed` |
| one bench's own checks and dry run, e.g. Bench: Config Check's | `python3 -I scripts/bench/offline/benches/config_check.py` | `... passed, 0 failed` |
| Bench: Limelight's ball phases, a bench in `run_bench.py`: their verdicts, each weakened, and `run_limelight` on `fake_hub.py` | `python3 -I scripts/bench/offline/benches/limelight.py` | `... passed, 0 failed` |
| Bench: Webcam's clamped and refused settings, a bench in `run_bench.py`: their verdicts, each weakened, and `run_webcam` on `fake_hub.py` | `python3 -I scripts/bench/offline/benches/webcam.py` | `... passed, 0 failed` |
| `TurretZeroMath`, `PinpointMath`, `OctoQuadMath`, the vision benches' helpers and the private fields they read (JUnit) | `./gradlew :TeamCode:testDebugUnitTest` | `BUILD SUCCESSFUL ...` |

A bench's offline file goes in `benches/`, named for the bench, whether the bench is new or one in `run_bench.py`
that an extension adds to; `test_benches.py` runs every file there.

Run them after changing those parts of `run_bench.py` or `bare_hub.sh`, and `check_dashboard_api.py` after any
slothboard bump. Clean up with `rm -rf scripts/bench/offline/out`.

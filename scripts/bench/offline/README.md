# Offline bench checks

Desktop checks for `run_bench.py`'s check of adb's server port, its saved-configs listing and `--list-configs`,
Bench: Dashboard Off, Bench: OctoQuad's bank phase, Bench: Turret Zero, Bench: Photon's two-hubs-at-once phase and
`--usb`. They need no hub: the hub, adb, `lsof` and the dashboard are stubs, nothing binds or connects to adb's port
5037 or the emulator ports 5554-5585, and nothing reaches the network. They catch regressions before a hub run; they
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
| dry run: the runner's connect, `--list-configs` and Bench: Dashboard Off end to end against a fake hub (about 40 s) | `python3 -I scripts/bench/offline/dryrun_stations.py` | `... passed, 0 failed` |
| dry run: the default step list end to end against `fake_hub.py` (a fake adb with USB devices, `connect`, `forward` on a picked port, `--list`, `--remove` and a device that leaves and returns without its forwards, plus slothboard's socket and the hub's HTTP ports, all on loopback; about 4 min). Over Wi-Fi, on a bare hub and on one with a webcam, the Limelight and an Expansion Hub, the adb calls (and on the second its sockets and HTTP requests) and the step selections must be byte for byte the ones in `fixtures/`, recorded from the runner before `--usb` (`ea69f20`); with `--usb`: no `adb connect`, every socket through a forward, only its own forwards removed, a reboot's and a pulled cable's forwards made again, two USB devices stopping before any device command, an unreadable hub socket table reported as not judged | `python3 -I scripts/bench/offline/dryrun_default.py` | `... passed, 0 failed` |
| `bare_hub.sh --usb` end to end from a copy, with stub `lsof`, `git`, `gradlew` and `adb` (about 1.5 min): no `adb connect`, the device rules, the dashboard port read from the hub's socket table, `REINSTALL.sh` run over the cable, the `--build --usb` fallbacks; without `--usb`, its adb calls, runner flags and `REINSTALL.sh` byte for byte the ones recorded from the script before `--usb` (`fixtures/bare_hub_wifi.json`) | `python3 -I scripts/bench/offline/test_bare_hub_usb.py` | `... passed, 0 failed` |
| the dashboard members the bench uses, by `javap` on the pinned slothboard AAR | `python3 -I scripts/bench/offline/check_dashboard_api.py` | `18 passed, 0 failed` |
| Bench: OctoQuad's bank-phase verdicts, good and broken results | `python3 -I scripts/bench/offline/test_octoquad_bank.py` | `... passed, 0 failed` |
| `run_octoquad` end to end with the hub stubbed | `python3 -I scripts/bench/offline/dryrun_octoquad.py` | `... passed, 0 failed` |
| Bench: Turret Zero's verdicts, good and broken results (`turret_zero_fixtures.py` builds them) | `python3 -I scripts/bench/offline/test_turret_zero_checks.py` | `... passed, 0 failed` |
| `run_turret_zero` end to end with the hub stubbed and the answers scripted: prompt order, config, params, the default run unchanged | `python3 -I scripts/bench/offline/dryrun_turret_zero.py` | `... passed, 0 failed` |
| Bench: Photon's two-hubs-at-once phase: the overlap reading, its record-only line and table | `python3 -I scripts/bench/offline/test_hub_overlap.py` | `... passed, 0 failed` |
| `TurretZeroMath` (JUnit) | `./gradlew :TeamCode:testDebugUnitTest` | `BUILD SUCCESSFUL ...` |

Run them after changing those parts of `run_bench.py` or `bare_hub.sh`, and `check_dashboard_api.py` after any
slothboard bump. Clean up with `rm -rf scripts/bench/offline/out`.

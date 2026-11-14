# Adding a bench

A new bench lives in its own file here, so builders working at the same time don't edit the same lines of
`run_bench.py`. The benches already in `run_bench.py` stay there. The worked example is Bench: Config Check:
[`config_check.py`](config_check.py), its offline file
[`../offline/benches/config_check.py`](../offline/benches/config_check.py) and its section in
[`../README.md`](../README.md#bench-config-check).

## The files of a bench

| file | what it holds |
|---|---|
| `scripts/bench/benches/<name>.py` | the declaration `BENCH = rb.Bench(...)` and the bench's functions |
| `scripts/bench/benches/order.txt` | one line, `<name>`, where the bench runs among the others here |
| `TeamCode/.../opmodes/test/bench/<Name>Bench.java` | the bench OpMode, if it needs one (a runner-only bench, like Config Check, has none) |
| `scripts/bench/offline/benches/<name>.py` | its offline file: check tests, at least one test that fails when a check is weakened, and a dry run on `fake_hub.py`; it ends `N passed, 0 failed` |
| `scripts/bench/README.md` | a row in The benches, a row in Flags for each flag, and a short section: what to plug in, the command, what a person does, what is judged, what only a hub shows |

`order.txt` is the run order: every file here must be listed, once. These benches run after every bench in
`run_bench.py`'s `BENCHES` but Bench: Turret Zero, Bench: Freeze Dump and Bench: Dashboard Off, which stay last. The
Photon block keeps its place. A file `order.txt` doesn't list, or a name without a file, stops `run_bench.py` before it
does anything.

## The declaration

The file gets `rb`, `run_bench.py`'s own globals, before it runs: `rb.Bench`, `rb.Ports`, `rb.Person`, `rb.BenchError`,
`rb.hub_config`, the `CONFIG_*` names and the rest.

| field | meaning |
|---|---|
| `name` | the file's name; also the results key and the `=== name ===` line |
| `title` | `"Bench: ..."`, used in skip lines and prompts |
| `flag`, `help` | `--` + the name with `-` for `_`, and its `--help` line |
| `options` | its own flags, as `(("--name-thing", {argparse keywords}),)`; start each with the bench's flag |
| `default` | `True`: in a run without bench flags and in `--all`. `False` (the usual): only with its flag |
| `needs` | `needs(r)` returns `None`, or one plain sentence, no final period, saying what is missing and where to plug it in. A default run skips the bench with that sentence; a run that names the bench stops, adding ", or drop --flag" |
| `person` | `rb.Person(what, have, flags=None, address=None)` if it waits for someone: `session.py` then lists it under Needs a person with the command and what to have on the table, and never runs it. `address`: the Expansion Hub address the bench must run at; its command then never carries the session's `--expansion-address`, and the line says to set the hub to that address when the session's differs |
| `drives` | `rb.Ports(ch_motors=..., ch_servos=..., eh_motors=..., eh_servos=...)`: every port it drives |
| `powers` | what it commands on those ports, in plain words (logged when it runs) |
| `configs` | the configs it activates: `bench_*` ones it writes (deleted at the end of a run that ran it, and by every `--probe-only` run) and shipped `res/xml` names (checked when the bench is chosen: a run that names it stops if this checkout doesn't ship one, a default run skips it) |
| `opmodes` | the bench OpModes it runs; the runner refuses a hub that lacks one |
| `run` | `run(r, record)`: the hub steps; fills `record`, a dict that must turn into JSON |
| `checks` | `checks(record)`: a list of `(name, ok, detail, what a FAIL means)`. It reads the record only |
| `summary` | `summary(record)`: the lines for `summary.md`, from the record only |

After the probe and the saved-configs listing, the runner activates `bench_main`, then for this bench: logs `powers`,
calls `run`, switches back to `bench_main` if `run` left another config active, then adds `checks(record)` to
`summary.md`'s checks and `summary(record)` to its body. The record is `runner_results.json`'s entry under the name. A
`rb.BenchError` from `run` fails the bench (`<name> bench completed`), skips its checks and lets the run go on.

In `run`, drive the hub only through the runner: `r.activate_config(name, xml)` (or `config=` a settings entry),
`r.run_opmode(opmode, step, params, ...)`, `r.pull(result)`, `r.ask(text)` for Enter, `r.answer(text)` for a typed
reply. Name every step `<name>_...`. Use `rb.time`, never `import time` or `subprocess`: the dry runs swap in a fast
clock and a fake adb. Never catch `rb.ProtocolError` or `rb.OtherHub`. Catch a `rb.BenchError` only to record it as a
result and go on, as Config Check does for each OpMode.

## A bench OpMode

`@TeleOp(name = "Bench: ...", group = "Test")`, in `opmodes/test/bench/`. Read the runner's parameters with
`BenchIO.params()` and `BenchIO.param(...)`. Report through `BenchReport report = new BenchReport("...", "<result>")`:
`report.phase(name)` at each phase (the runner shows it), `report.put(key, value)` and `report.save()` as results come
in, so a crash keeps what finished, and `report.done(summary)` at the end, which writes `<result>.json` and logs
`BENCH DONE <result>: ...`. The runner waits for that line (`result_files=["<result>"]`), then `r.pull("<result>")`
gives the JSON. A LinearOpMode holds with `report.holdUntilStart(this)` and `report.holdUntilStop(this)`. Fetch every
device with `BenchIO.require(...)`, put outputs in a safe state in `stop()` or a `finally`, and catch only what the
bench measures, recording it. Run `./gradlew :TeamCode:compileDebugJavaWithJavac :TeamCode:testDebugUnitTest`.

## Safety rules

| rule | where the code keeps it |
|---|---|
| A bench that drives a motor or servo port declares `drives`; the first statement that covers those ports is the one it needs | `PORT_STATEMENTS` and `bench_layout_problems` in `run_bench.py` |
| Within Control Hub motor ports 0-3 and servo ports 0-1: `--ports-empty`, as the benches in `run_bench.py` | `check_bench_ports` refuses a hub whose configs name devices there without it |
| More than that (servo ports 2-5, either hub's Expansion Hub ports): `--s3-ports`, typed `empty` at the terminal before the first adb command, else the run stops with nothing changed | `port_statements`, and `run_module_bench` refuses a bench whose statement wasn't typed |
| A default-suite bench drives only what `--ports-empty` covers, and never waits for a person | `bench_layout_problems` |
| A bench that asks a person anything runs only with its own flag, declares `person`, states each action as one plain instruction, and never treats a timeout as done (`r.ask` and `r.answer` wait with no time limit) | `bench_layout_problems`; `offline/test_session.py` fails while a file here prompts without a `person` |
| STOP and Ctrl-C put the hub's config back | `Runner.run`'s `finally`; `bench_*` configs a bench declares are deleted there too |
| A default run with no new flags sends exactly the adb commands it sends today | `offline/dryrun_default.py` against `offline/fixtures/` |

A bench in the default suite changes the default run: re-record only what that explains
(`python3 -I scripts/bench/offline/dryrun_default.py --record DIR`) and say so.

## Extending a bench that is in `run_bench.py`

New phases of an existing bench go in its own OpMode and its own `run_<name>` in place. Keep the edit local:

- add new module-level helpers and constants just above that bench's `run_<name>`, not in the shared blocks at the top
  of the file, and put its check functions next to the existing ones for that bench;
- give a new flag `--<bench>-<thing>` and add it right after that bench's own `add_argument` in `parse_args`;
- a new prompt there is a new site in `offline/test_session.py`'s `PROMPT_SITES` and, if it is a new run to do by
  hand, an entry in `session.py`'s `PROMPTED`;
- tests go in a new file in `offline/benches/`, named for the bench, not in the shared suites; `test_benches.py`
  runs it.

## Offline checks

From the repo root:

- `python3 -I scripts/bench/offline/test_benches.py`: every declaration here, `order.txt`, the flags, the session's
  Needs a person, a stub bench on `fake_hub.py`, then each `offline/benches/<name>.py`. Ends `N passed, 0 failed`.
- `python3 -I scripts/bench/offline/benches/<name>.py`: one bench's.
- The suites in [`../offline/README.md`](../offline/README.md), above all `test_session.py`, `dryrun_default.py` and
  `test_station_checks.py`, which must pass untouched; the Photon ones if anything under Photon changed.

Each offline file writes under `scripts/bench/offline/out/` (gitignored).

## Commits

One subject line, imperative, plain words saying what the commit does; no body, no trailers. Never the words audit,
review, cleanup, theme, finding or wave, and no internal id or plan key. One purpose per commit: the bench, its
README rows and its offline file go together; a change to a shared file (`run_bench.py`, `session.py`, a Java helper)
is its own commit. The repository is public: no names or handles, no network names, no addresses but `192.168.43.1`,
`127.0.0.1` and the Limelight's own, no paths from your computer, no email addresses.

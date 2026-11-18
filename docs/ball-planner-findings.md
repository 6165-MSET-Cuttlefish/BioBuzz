# Ball planner findings

What the ball route planner (`architecture/planner/`) will never collect, when it counts a ball as collected, how to
plan 4 balls without restarting the app at STOP, and how to test it. Read it before changing the planner, ball
selection or the ball count. Planner callers: `RobotActions.collectBalls` (Ball Collection Auto), Ball Collection,
Vision Ball Collection and `RobotActions.driveTo` (BioBuzz Tele's shot waypoint, no balls). Planning times and STOP measurements are in [`hub-bench-findings.md`](hub-bench-findings.md)
(Route planner).

Java paths below are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`, other paths from the repository
root. Everything is read from source on 2026-11-10 unless marked otherwise.

## What to do

1. **Don't expect it to collect a ball within about 8.5 in of a wall, the centre line or a HIVE rail.** It drops
   such balls before planning. Leave wall and corner balls to the driver, or change how legs end (below).
2. **A ball half outside the intake counts as collected.** The planner compares the ball's centre with half the
   intake's width. Subtract the ball's radius, or a capture margin measured on the intake, in both checks.
3. **Plan 4 balls, one search slice per loop** (team decision, 2026-10-24): `RouteOptimizer.search(...)`, then one
   `Search.step()` per loop until it returns true. Don't cap at 3 to fix STOP. On a hub its longest step was 325 ms
   (Bench: Planner Stop, 2026-11-17), and no STOP during a sliced plan restarted the app, with or without the webcam.
   But Bench: Planner Stop's webcam trials, which closed the webcam inside `stop()` as Ball Collection Auto then did,
   took up to 839 ms, and that worst case, the longest step plus the slowest webcam close, adds up to about 1138 ms,
   over the 900 ms window (inferred). The fix is to close the webcam after the window: main now leaves the close to
   EasyOpenCV after the OpMode (from the code), which makes it about 360–380 ms (inferred from `stop()` and the stop
   pass measured in Bench: Webcam Close, 2026-11-17, in the bench build). Main's change hasn't run on a hub with the
   planner yet: until Bench: Planner Stop's webcam trials pass on main's build, don't trust a STOP during Ball
   Collection Auto's 4-ball plan.
4. **Keep planning on the hub, never on the Limelight.**
5. **Add an independent geometry check as a unit test before changing the planner**, and sample compound paths just
   before each join, not on it.

## Balls it never collects

`RouteOptimizer.ballProblem` (`architecture/planner/RouteOptimizer.java:64-72`) accepts a ball only if the robot,
centred on the ball at one of 72 headings (every 5°, `RoutePathBuilder.HEADING_STEPS`), keeps its whole footprint
inside the area and more than the gap from every obstacle. The footprint is the rectangle around frame and intake:
half-width `max(widthIn, intakeWidthIn) / 2`, back `lengthIn / 2`, front `max(lengthIn / 2, intakeOffsetIn)`
(`architecture/planner/RobotShape.java:28-30`). With `onField` on, the area is `BioBuzzField.ownHalf(wallGapIn)` and
the obstacles are the two HIVE rails (`biobuzz/BallCollection.java:92-99`). Ball Collection, Vision Ball Collection
and Ball Collection Auto default to `onField` off ([CLAUDE.md](../CLAUDE.md), Testing), so the limits below apply
only with it on.

A ball beside a wall, the centre line or a rail needs the smaller of that half-width and `lengthIn / 2`, plus the
gap. With `RobotGeometry` (16 in long, 15 in wide, intake 15 in wide and 8 in ahead: the Betta bot's, retired 2026-11-13, not
yet the BIOBUZZ robot's) and the default 1 in
`wallGapIn` and `railGapIn`:

| ball | collectable only if |
|---|---|
| beside a wall or the centre line | at least 8.5 in from it |
| in a corner | 8.5 in from one wall and 9 in from the other |
| beside a HIVE rail | more than 8.5 in from it |
| on RED's half, level with the rail (Pedro y 51.3 to 90.2) | x from 8.5 to 37.5, or from 56.5 to 62.25 |

The rows were checked with a reimplementation of the footprint test; the last is computed from the rail constants
(`biobuzz/BioBuzzField.java:148-159`) and the 141.5 in `FieldConfig.fieldWidthInches`. BLUE's is its 180° rotation.
Between the rail and the centre line only a 5.75 in strip is reachable.

- `BallCollection.select` (`biobuzz/BallCollection.java:249-252`) drops these balls before picking the nearest
  `BallCollection.BALLS` (4) and lists each as skipped: "out of reach: every heading over it puts the robot over a
  wall, the centre line or an obstacle". Vision Ball Collection drops and counts them the same way.
- Ways to reach them, none tried: end the leg with the intake's middle on the ball instead of the robot's centre
  (`RobotShape.centreFor` computes that centre; driving head-on at a wall, the footprint's front edge then sits on the
  ball's centre and only has to stay the gap short of the wall), which changes `ballProblem` and the leg ends;
  smaller gaps (each inch of gap is an inch of reach); or leave them to the driver. A side pass with the intake's
  edge over the ball only works on paper, because of the radius problem below.
- The rails are the only obstacles (`BallCollection.obstacles`; Vision Ball Collection adds three dashboard boxes,
  off by default), so routes drive over balls they aren't collecting, 3.6 in Nectar included
  (`biobuzz/modules/vision/BallType.java:5-7`).

## When a ball counts as collected

A ball is *swept up* when the planner counts it as collected because the intake passes over it, without the robot's
centre driving through it. Two checks decide it:

- `IntakeCurvePlanner.essentialStops` picks the balls the intake curve must pass through: from the start and then
  each kept ball it runs straight to the furthest ball whose in-between balls all lie within `intakeWidthIn / 2` of
  that chord (`IntakeCurvePlanner.sweptUp`, `architecture/planner/IntakeCurvePlanner.java:62-71`).
- `RoutePathBuilder` builds Pedro's spline through those stops, traces the intake's middle (the robot's centre plus
  `intakeOffsetIn` along the heading, at `Segment`'s samples at most 1 in apart), and turns any ball farther than
  `intakeWidthIn / 2` from that track into a stop (`architecture/planner/RoutePathBuilder.java:244-274`). A leg
  rebuilt around an obstacle drives the robot's centre through every ball instead (`RoutePathBuilder.java:279`).

Both compare the ball's centre with half the intake width. `Ball.radius` is set but only used for drawing. With the
15 in intake, a ball whose centre is 7.5 in off the intake's middle counts as collected though half of it is outside:
1.4 in of a 2.8 in Pollen, 1.8 in of a 3.6 in Nectar (`BallType`). Plans can pass beside balls the real intake won't
grab, and `RouteRun` drives the plan without checking what went in.

Fix: compare with `intakeWidthIn / 2` minus the ball's radius, or minus a capture margin measured on the intake, in
both places. A narrower reach means more stops per curve, so rerun Bench: Hub afterwards. How much longer plans take
then is unknown; test: `scripts/bench/run_bench.py --hub --hub-planner-baseline bench-results/<a run before the fix>
--ports-empty`, which prints each ball count's and layout's plan times before and after.

## Planning 4 balls without restarting the app at STOP

Decision (2026-10-24): Vision Ball Collection, Ball Collection Auto and Ball Collection always plan 4 balls;
`BallCollection.BALLS` pins it, with no dashboard setting to raise it.

A STOP during a loop waits for it: the SDK calls `stop()` only after the running `init_loop()` or `loop()` returns
(`OpMode.internalRunOpMode`, SDK 12.0.0). If that outlasts the SDK's stop window (900 ms, then 100 ms in which hub
commands throw; `OpModeInternal.MS_BEFORE_FORCE_STOP_AFTER_STOP_REQUESTED` and `OpModeManagerImpl`), the SDK restarts
the Robot Controller app. A plan sends no hub commands, so nothing throws it out first. A 4-ball plan made in one call
took up to 2.4 s on a hub and restarted the app on a STOP; the times and restart rates are in
[`hub-bench-findings.md`](hub-bench-findings.md).

Why plans are slow: the search tries every order of the full ball set (n!) and drops to n − 1 balls only when no full
order is drivable (`RouteOptimizer.java:171-214`). Orders run cheapest lower bound first and stop once the bound
reaches the best route found, but until one order works nothing is pruned: one awkward ball costs a
`RoutePathBuilder.Planner.shape()` call for every order before a smaller set is tried.

How it plans now (read from source):

- Every collection OpMode builds a `RouteOptimizer.Search` (`biobuzz/BallCollection.java:224`,
  `opmodes/test/auto/VisionBallCollectionTest.java:376`, `opmodes/test/auto/BallCollectionTest.java:179`) and calls
  `step()` once per loop until it returns true (`BallCollection.java:198`, `VisionBallCollectionTest.java:252`,
  `BallCollectionTest.java:185`). A step shapes orders until `VisibilityGraphPlanner.routeSearchSliceMs` (20 ms) has
  passed, so it overruns the slice by at most one `shape()` call or the final Pedro path build
  (`RouteOptimizer.java:143-154`). STOP then waits for one step, not a plan.
- Budget: the longest step + the loop's tail + the `stop()` pass under 900 ms. Ball Collection Auto's `stop()` used
  to close the cell-tip webcam too (up to 783 ms in [`hub-bench-findings.md`](hub-bench-findings.md)), which left
  about 117 ms. Main's `CellTipCamera.stop()` now leaves the close to EasyOpenCV after the OpMode, outside the window
  (from the code). In the bench build's version of that (Bench: Webcam Close, 2026-11-17, no planner) `stop()` took
  at most 2.3 ms and the whole stop pass at most 26 ms, so the budget is the step, the tail and about 26 ms of stop
  pass. The longest step on a hub was 325 ms (4 balls on both sides of the HIVE rail, Bench: Planner Stop,
  2026-11-17; Bench: Hub's longest on that layout was 273 ms), so a STOP would need about 360–380 ms with the tail
  (inferred), against about 1138 ms with the close inside `stop()`. Without a webcam the hub logged
  every STOP reaching idle in at most 282 ms (2026-11-14) and 118 ms (2026-11-17); with the webcam closed inside
  `stop()`, in at most 839 ms over 20 trials, none of which met the longest step and the slowest close at once
  (2026-11-17).
- Vision Ball Collection checks for a changed request every `replanIntervalMs` (250 ms) and keeps showing the last
  finished plan meanwhile; a running search is replaced only when the settings change or the robot moves, not when
  the balls do, or churning vision would never let a 4-ball plan finish (`VisionBallCollectionTest.java:249-276`).
- If slices can't be made short enough on a hub, fall back to one worker thread: copied, immutable inputs (the
  `Search` already copies them) and a volatile cancel flag checked between `shape()` calls, like
  `LimelightSync.Request`; started and stopped by [CLAUDE.md](../CLAUDE.md)'s rules for hub-reading threads (Testing):
  loop only while the OpMode thread is alive, stop with the flag and `join`, rethrow its crash on the OpMode thread.

## Why not plan on the Limelight

- The planner, the Pedro curves it scores (`BezierCurve.through`) and our `Segment` sampling are Java on the hub;
  they would need a Python port.
- The hub gets 32 doubles from the SnapScript (`LLResult.getPythonOutput()` returns exactly 32, SDK 12.0.0 source),
  and the script already uses all of them: 2 header slots plus 3 per ball for up to 10 balls
  (`limelight/ball_contour_snapscript.py:9-10` and `:21`). A plan would push out ball reports.
- The ball pipeline is already the slow part of ball vision (about 4.7 results/s, in
  [`hub-bench-findings.md`](hub-bench-findings.md)); planning in the same script would slow it further (reasoning,
  not measured).

## Driving to a waypoint (no balls)

BioBuzz Tele's shot waypoint (`biobuzz/WaypointDrive.java`) runs `RouteOptimizer.search` with no balls, so its plan
is only the return leg: a straight line turning evenly to the waypoint's heading, else a spline through
`VisibilityGraphPlanner`'s detour points, else straight hops with turns in place at the joins. Probed on 2026-11-17
in a desktop unit test on RED's half, with the Betta bot's 16 × 15 in footprint (`RobotGeometry`, not yet the
BIOBUZZ robot's) and 1 in wall and rail gaps:

| start → waypoint (RED inches, heading °) | `boundaryMarginIn` 1.5 (default) | 3.5 | 5 |
|---|---|---|---|
| (30, 30, 90) → (30, 115, 90), west of our rail | straight | straight | straight |
| (60, 30, 90) → (60, 115, 90), under the HIVE | straight | straight | straight |
| (30, 71, 0) → (60, 105, 0), round the rail's rear end | hops, 6 steps | hops, 4 steps | hops, 6 steps |
| (20, 20, 0) → (58, 110, 180), across the rail, ending back to the HIVE | no route | spline | spline |
| (30, 71, 0 or 90) → (60, 71, same), into the slot between our rail and the centre line | no route | no route | no route |
| (60, 71, 0) → (30, 110, 0), out of that slot | no route | no route | no route |

- **Why the corners fail:** the detour graph keeps the robot's centre `width / 2 + railGapIn` (8.5 in) plus
  `boundaryMarginIn` from each rail corner, 10 in at the default, but a 16 × 15 in rectangle's corner reaches
  10.96 in from its centre, so the footprint check fails at the corner points whatever the heading.
- **Why the slot fails:** a turn in place needs 11.96 in of room all round, and the slot between our rail and the
  centre line is 21.75 in wide.
- **What the driver sees:** `refused: no drivable path ...` on the DS and three rumble blips; drive it by hand.
- **Not changed:** raising `boundaryMarginIn` reaches more drives round a rail, but it widens every ball-collection
  detour too, untested there, so the default stays 1.5. Rerun the table once `RobotGeometry` holds the BIOBUZZ
  robot's size.

## Testing the planner

- No automated test checks a plan's geometry. `PlannerWorkloadTest` (`TeamCode/src/test`) checks only how a search
  slices its work and that slicing doesn't change the route; `WaypointDriveTest` only which kind of leg a no-ball
  drive gets and that an unreachable one is reported rather than thrown. Bench: Hub and Bench: Planner Stop time plans and STOP on
  a hub but don't check a plan's geometry either.
- Before changing the planner, add a unit test (`./gradlew :TeamCode:testDebugUnitTest`) that walks every
  `Plan.steps` drive and turn over many layouts and checks, with its own footprint and distance maths rather than
  `RobotShape`'s: the footprint stays inside the area, more than `railGapIn` from each rail, and every ball in
  `route.order` passes under the intake. An independent check like this, over about 10,000 random layouts on a
  desktop, caught real bugs in an earlier version of the planner (2026-10-02 and 2026-10-03).
- The planner classes except `RouteRun` need no hardware. `PlannerWorkload`
  (`opmodes/test/bench/PlannerWorkload.java`) has seeded random layouts and hand-built worst cases to reuse; it is
  package-private, so put the test in its package. Build the area as a `Region` as it does, or call
  `BioBuzzField.configure()` first: `BioBuzzField.ownHalf` throws until the field symmetry is set
  (`FieldConfig.symmetry()`).
- **Trap when sampling headings:** a compound path's `heading(t)` at a join returns the next piece's heading, because
  Pedro's `Piecewise.getSegment` finds the piece with `NavigableMap.floorEntry` (javap on Pedro `core` 3.0.2).
  `Plan.intake` and `Plan.back` are compound when a leg is built from straight hops (`Reroute.POLYLINE`,
  `RoutePathBuilder.java:153-159`); each `Plan.steps` path is one piece. Sample each piece on its own, or a piece's
  end at the join's t minus 1e-9.
- Give the checker's "under the intake" test a few hundredths of an inch of slack (an earlier checker used 0.02 in):
  the planner measures to chords between track samples, which cut inside a curve, so a checker that samples more
  finely reports false misses.

## Still unknown

- **How far off the intake's middle the real intake still captures a ball.** Drive the intake straight over balls
  placed 0, 3, 5, 6 and 7 in off its middle and record which go in; use the result as the capture margin.
- **Whether driving over a 3.6 in Nectar is acceptable** for the robot and under the game manual. Drive over one on
  the robot and check the manual before adding balls as obstacles.
- **STOP during the sliced search with the webcam open, as in Ball Collection Auto.** Without a webcam it was
  measured on 2026-11-14 ([`hub-bench-findings.md`](hub-bench-findings.md)). Test:
  `scripts/bench/run_bench.py --planner-stop --ports-empty` with a UVC webcam on the Control Hub.

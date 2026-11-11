# Ball planner findings

What the ball route planner (`architecture/planner/`) will never collect, when it counts a ball as collected, how to
plan 4 balls without restarting the app at STOP, and how to test it. Read it before changing the planner, ball
selection or the ball count. Planner callers: `RobotActions.collectBalls` (Ball Collection Auto), Ball Collection and
Vision Ball Collection. Planning times and STOP measurements are in [`hub-bench-findings.md`](hub-bench-findings.md)
(Route planner).

Java paths below are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`, other paths from the repository
root. Everything is read from source on 2026-11-10 unless marked otherwise.

## What to do

1. **Don't expect it to collect a ball within about 8.5 in of a wall, the centre line or a HIVE rail.** It drops
   such balls before planning. Leave wall and corner balls to the driver, or change how legs end (below).
2. **A ball half outside the intake counts as collected.** The planner compares the ball's centre with half the
   intake's width. Subtract the ball's radius, or a capture margin measured on the intake, in both checks.
3. **Plan 4 balls, but only once the order search is time-sliced** (team decision, 2026-10-24). Don't cap at 3 to
   fix STOP. Until the sliced search is in, leave `maxBalls` at its default of 3; Ball Collection's 4 slots already
   risk an app restart on STOP.
4. **Keep planning on the hub, never on the Limelight.**
5. **Add an independent geometry check as a unit test before changing the planner**, and sample compound paths just
   before each join, not on it.

## Balls it never collects

`RouteOptimizer.ballProblem` (`architecture/planner/RouteOptimizer.java:63-71`) accepts a ball only if the robot,
centred on the ball at one of 72 headings (every 5°, `RoutePathBuilder.HEADING_STEPS`), keeps its whole footprint
inside the area and more than the gap from every obstacle. The footprint is the rectangle around frame and intake:
half-width `max(widthIn, intakeWidthIn) / 2`, back `lengthIn / 2`, front `max(lengthIn / 2, intakeOffsetIn)`
(`architecture/planner/RobotShape.java:35-37`). With `onField` on, the area is `BioBuzzField.ownHalf(wallGapIn)` and
the obstacles are the two HIVE rails (`biobuzz/BallCollection.java:97-104`). Ball Collection, Vision Ball Collection
and Ball Collection Auto default to `onField` off ([CLAUDE.md](../CLAUDE.md), Testing), so the limits below apply
only with it on.

A ball beside a wall, the centre line or a rail needs the smaller of that half-width and `lengthIn / 2`, plus the
gap. With `RobotGeometry` (16 in long, 15 in wide, intake 15 in wide and 8 in ahead) and the default 1 in
`wallGapIn` and `railGapIn`:

| ball | collectable only if |
|---|---|
| beside a wall or the centre line | at least 8.5 in from it |
| in a corner | 8.5 in from one wall and 9 in from the other |
| beside a HIVE rail | more than 8.5 in from it |
| on RED's half, level with the rail (Pedro y 51.3 to 90.2) | x from 8.5 to 37.5, or from 56.5 to 62.25 |

The rows were checked with a reimplementation of the footprint test; the last is computed from the rail constants
(`biobuzz/BioBuzzField.java:152-166`) and the 141.5 in `FieldConfig.fieldWidthInches`. BLUE's is its 180° rotation.
Between the rail and the centre line only a 5.75 in strip is reachable.

- `BallCollection.select` (`biobuzz/BallCollection.java:244-247`) drops these balls before picking the nearest
  `maxBalls` and lists each as skipped: "out of reach: every heading over it puts the robot over a wall, the centre
  line or an obstacle". Vision Ball Collection drops and counts them the same way.
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
  that chord (`IntakeCurvePlanner.sweptUp`, `architecture/planner/IntakeCurvePlanner.java:75-84`).
- `RoutePathBuilder` builds Pedro's spline through those stops, traces the intake's middle (the robot's centre plus
  `intakeOffsetIn` along the heading, at `Segment`'s samples at most 1 in apart), and turns any ball farther than
  `intakeWidthIn / 2` from that track into a stop (`architecture/planner/RoutePathBuilder.java:242-272`). A leg
  rebuilt around an obstacle drives the robot's centre through every ball instead (`RoutePathBuilder.java:277`).

Both compare the ball's centre with half the intake width. `Ball.radius` is set but only used for drawing. With the
15 in intake, a ball whose centre is 7.5 in off the intake's middle counts as collected though half of it is outside:
1.4 in of a 2.8 in Pollen, 1.8 in of a 3.6 in Nectar (`BallType`). Plans can pass beside balls the real intake won't
grab, and `RouteRun` drives the plan without checking what went in.

Fix: compare with `intakeWidthIn / 2` minus the ball's radius, or minus a capture margin measured on the intake, in
both places. A narrower reach means more stops per curve, so rerun Bench: Hub afterwards.

## Planning 4 balls without restarting the app at STOP

Decision (2026-10-24): Vision Ball Collection, Ball Collection Auto and Ball Collection always plan 4 balls, and the
count stays a setting.

Every collection OpMode plans in one call to `RouteOptimizer.findOptimalRoute` on the OpMode thread
(`biobuzz/BallCollection.java:224`, `opmodes/test/auto/VisionBallCollectionTest.java:338`,
`opmodes/test/auto/BallCollectionTest.java:168`). `maxBalls` defaults to 3 in `BallCollection.Settings` and in Vision
Ball Collection; Ball Collection plans its 4 `ballCount` slots. Vision Ball Collection plans in INIT and whenever it
isn't driving, at most every `replanIntervalMs` (250 ms) once a ball, the robot or a setting changes
(`VisionBallCollectionTest.java:214-236`), so a STOP can land in a plan at any idle moment.

A STOP during a plan waits for it: the SDK calls `stop()` only after the running `init_loop()` or `loop()` returns
(`OpMode.internalRunOpMode`, SDK 12.0.0). If that outlasts the SDK's stop window (900 ms, then 100 ms in which hub
commands throw; `OpModeInternal.MS_BEFORE_FORCE_STOP_AFTER_STOP_REQUESTED` and `OpModeManagerImpl`), the SDK restarts
the Robot Controller app. A plan sends no hub commands, so nothing throws it out first. 4-ball plans can take long
enough; the times and restart rates are in [`hub-bench-findings.md`](hub-bench-findings.md).

Why plans are slow: `findOptimalRoute` tries every order of the full ball set (n!) and drops to n − 1 balls only
when no full order is drivable (`RouteOptimizer.java:107-132`). Orders run cheapest lower bound first and stop once
the bound reaches the best route found, but until one order works nothing is pruned: one awkward ball costs a
`RoutePathBuilder.Planner.shape()` call for every order before a smaller set is tried.

Making 4 balls safe:

- Make the order search resumable and run it in slices on the OpMode thread: a few `shape()` calls per loop, ending a
  slice when its time budget runs out. STOP then waits for at most one slice budget plus one `shape()` call.
- Budget: slice budget + longest `shape()` + `stop()` under 900 ms. Ball Collection Auto's `stop()` also closes the
  cell-tip webcam (up to 581 ms in [`hub-bench-findings.md`](hub-bench-findings.md)), which leaves about 300 ms.
- It must return exactly the route the one-shot `findOptimalRoute` returns: assert that in a unit test over the same
  layouts.
- Raise the `maxBalls` defaults to 4 only after Bench: Hub and Bench: Planner Stop (`scripts/bench/README.md`) pass
  on a hub with the sliced search.
- If slices can't be made short enough, fall back to one worker thread on the hub: copied, immutable inputs and a
  volatile cancel flag checked between `shape()` calls, like `LimelightSync.Request`; started and stopped by
  [CLAUDE.md](../CLAUDE.md)'s rules for hub-reading threads (Testing): loop only while the OpMode thread is alive,
  stop with the flag and `join`, rethrow its crash on the OpMode thread.

## Why not plan on the Limelight

- The planner, the Pedro curves it scores (`BezierCurve.through`) and our `Segment` sampling are Java on the hub;
  they would need a Python port.
- The hub gets 32 doubles from the SnapScript (`LLResult.getPythonOutput()` returns exactly 32, SDK 12.0.0 source),
  and the script already uses all of them: 2 header slots plus 3 per ball for up to 10 balls
  (`limelight/ball_contour_snapscript.py:9-10` and `:21`). A plan would push out ball reports.
- The ball pipeline is already the slow part of ball vision (about 4.7 results/s, in
  [`hub-bench-findings.md`](hub-bench-findings.md)); planning in the same script would slow it further (reasoning,
  not measured).

## Testing the planner

- There is no automated planner test: `TeamCode/src/test` holds only `TargetSelectorTest` and `ShotSolverTest`. Bench:
  Hub and Bench: Planner Stop time plans and STOP on a hub but don't check a plan's geometry.
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
- **The sliced search on a hub:** its longest `shape()` call and slice, its total plan time, and STOP. Run Bench: Hub
  and Bench: Planner Stop once it exists.

# Wiring findings

Where each device on the BIOBUZZ robot should plug in so the loop is short and steady, and what the code must do for
that wiring to be safe. Read it before wiring the robot, writing its hardware config, or moving a device between hubs
or ports.

**Nothing is wired yet (2026-11-10), and no loop time here is measured on this robot.** The device list is the CAD's
([`robots-findings.md`](robots-findings.md)). Command costs are measured on the bench hubs
([`hub-bench-findings.md`](hub-bench-findings.md)). Loop periods are those costs added up by
`scripts/looptimes/wiring_budget.py`, so they are estimates. CH is the Control Hub, EH the Expansion Hub. The nectar
turret encoder's version, gearing and travel are the team's report of 2026-11-11
([The nectar turret encoder](#the-nectar-turret-encoder)). Its wiring, a count on a Control Hub encoder port zeroed
once from the OctoQuad at INIT, was decided 2026-11-12 for now, until Bench: Turret Zero has run on a hub
([Zeroing the nectar turret](#zeroing-the-nectar-turret)).

## What to do

- **Wire to [the map](#the-map).** Three rules decide most of it:
  1. Nothing that is read goes on the Expansion Hub: no encoder, analog, digital or I2C device. One input there adds
     that hub's bulk read to every loop, about 3.6 ms.
  2. Outputs that change every loop go on the Control Hub: both flywheels, the three turret servos, both hoods, and
     the two drive motors that still fit.
  3. Each flywheel's Through Bore goes on Control Hub encoder port 0 or 3, the same number as its motor.
- **Link the hubs by USB, for Photon.** The team is using Photon (decided 2026-11-11), and Photon speeds up
  Expansion Hub writes only when that hub has its own USB link: a mini-USB to USB-A cable in place of RS485
  ([the cable and its route](#where-the-hubs-sit-and-the-usb-cable-between-them)). Photon has not passed its bench's
  go run on this wiring ([`photon-bench.md`](photon-bench.md)), so route one RS485 cable as well and leave it
  unplugged at the Expansion Hub: it is the way back. Never connect both: only USB alone has been benched.
- **Expect about 16 ms parked and 22 to 24 ms while driving and aiming** with Photon on both hubs, about 2 ms less
  with the dashboard off (estimated from the Photon bench of 2026-10-29). On the fallback, RS485 without Photon:
  19 ms and 28 to 37 ms ([Estimated loop period](#estimated-loop-period)). The wiring cannot fix the rest: 11
  outputs change every loop and each costs a hub command.
- **Run the bench tests in [Open choices](#open-choices) before the wiring is final.** Two placements rest on an
  encoder port keeping up, which no vendor documents: the flywheels' on ports 0 and 3, and the nectar turret's count
  on port 2, which counts in software.
- **Make the changes in [What the code must do](#what-the-code-must-do) before the first run.** Several are about
  safety, not speed.
- **Make the hub in the bay without the battery the Control Hub**, and leave room behind both hubs' USB ports. As
  drawn, the battery holder blocks the other hub's USB-C port
  ([Where the hubs sit](#where-the-hubs-sit-and-the-usb-cable-between-them)).
- **Rewrite `TeamCode/src/main/res/xml/biobuzz.xml` in place** with these ports. It exists with an old layout, and
  adding a new config file shifts the stored config ([`toolchain-findings.md`](toolchain-findings.md)).

## The map

**Control Hub**

| port | device | note |
|---|---|---|
| motor 0, encoder 0 | pollen flywheel motor and its Through Bore | port 0 counts in hardware |
| motor 1 | front-left drive | its own motor encoder stays unplugged; encoder 1 is spare |
| motor 2 | back-right drive | its own motor encoder stays unplugged |
| encoder 2 | nectar turret Through Bore's count (A and B) | counted in software; the hub alone powers the encoder; zeroed at INIT from the OctoQuad: [Zeroing the nectar turret](#zeroing-the-nectar-turret) |
| motor 3, encoder 3 | nectar flywheel motor and its Through Bore | port 3 counts in hardware |
| servo 0, 1 | pollen turret servos | injector A |
| servo 2 | nectar turret servo | injector B |
| servo 3 | pollen hood | injector A |
| servo 4 | nectar hood | injector B |
| servo 5 | blocker | injector A |
| analog 0 | Floodgate current output | shares connector 0-1 with rangefinder A through a splitter cable |
| analog 1 | transfer rangefinder A (upstream), if kept | the port the bench config `laser_rangefinder.xml` uses |
| analog 2 | spare | |
| analog 3 | transfer rangefinder B (downstream), if kept | the bench port |
| I2C bus 0 | nothing | the built-in IMU's bus |
| I2C bus 1 | spare | |
| I2C bus 2 | OctoQuad: odometry pods on channels 0 and 1; channel 4 takes the nectar turret encoder's absolute wire | channel 4 is read once at INIT, to zero the turret's count: [Zeroing the nectar turret](#zeroing-the-nectar-turret) |
| I2C bus 3 | Prism | written only when the pattern changes; powered from its own XT30 |
| USB 3.0 | the powered USB hub carrying the Limelight 3A, the Arducam and the Expansion Hub's USB cable | the Expansion Hub behind this hub is untested: [the test](#where-the-hubs-sit-and-the-usb-cable-between-them) |
| USB 2.0 | empty | shares a USB bus with the hub's Wi-Fi radio; the second choice for the Expansion Hub's cable |
| USB-C | laptop only | keeps deploying over a cable possible at events |
| RS485 | one cable routed to the Expansion Hub, unplugged at that end | the fallback if Photon fails its bench run |

**Expansion Hub** (on USB, mini-USB to USB-A, with Photon; address 2 on the RS485 fallback)

| port | device | note |
|---|---|---|
| motor 0 | front-right drive | |
| motor 1 | back-left drive | |
| motor 2 | intake | |
| motor 3 | transfer motor | |
| encoders, analog, digital, I2C | nothing | so this hub's bulk read never runs |
| servo 0 | transfer continuous-rotation servo | injector B |
| servo 1 | flower intake servo | injector B |
| servo 2 and servo 4 | one RGB indicator light each | hub 5 V; one light per port pair |
| servo 3, 5 | spare | |

**Servo power.** Injector A: pollen turret pair, pollen hood, blocker. Injector B: nectar turret, nectar hood,
transfer servo, flower intake. Each injector takes an XT30 lead from the distribution board. Connect hub servo ports
to injector inputs with the leads the injector ships with, which have the red wire removed, so no hub 5 V reaches it.

## Why each placement

Loop period in ms for the map and for the same map with one thing moved, on the RS485 fallback without Photon
(estimated, p50 costs, every drive motor writing every loop). The same comparison under Photon is under
[Estimated loop period](#estimated-loop-period): the map stays the shortest or ties.

| | parked, aiming | driving and aiming | the same, firing |
|---|---|---|---|
| **the map** | **18.7** | **36.6** | **37.2** |
| any input on the Expansion Hub | 23.1 | 40.8 | 41.4 |
| the OctoQuad's encoder block read every loop, for the turret angle | 22.8 | 40.5 | 41.1 |
| turret servos on the Expansion Hub | 18.7 | 39.5 | 40.0 |
| flywheels on the Expansion Hub, all four drive motors on the Control Hub | 20.8 | 36.6 | 37.2 |
| intake and transfer on the Control Hub, all four drive motors on the Expansion Hub | 18.7 | 38.5 | 39.0 |
| battery voltage read every loop | 20.3 | 38.1 | 38.6 |
| the layout planned before the CAD existed | 18.3 | 37.4 | 37.8 |

- **No inputs on the Expansion Hub.** With manual bulk caching a hub's bulk read runs only if something on it is
  read. Both hubs' bulk reads cost 5.72 ms against 2.10 ms for the Control Hub alone (measured). Things that would
  start it without anyone noticing: `Drivetrain.encoderTelemetry` (`biobuzz/modules/Drivetrain.java:256-262`),
  `isOverCurrent()` on an Expansion Hub motor, or an analog input mapped there.
- **Flywheels on Control Hub motors 0 and 3.** Their power changes every loop in closed loop, parked or driving. A
  Control Hub motor write is 1.64 ms (measured); over RS485 it is about 2.5 ms (estimated: the 0.9 ms hop is measured
  on reads only). Ports 0 and 3 are also the only encoder ports that count in hardware
  ([`sdk-hub-findings.md`](sdk-hub-findings.md), Encoder ports).
- **Two drive motors on the Control Hub.** While driving, 6 motors change and a hub has 4 motor ports, so at least
  two changing motor writes cross RS485 whatever the map.
- **A diagonal pair, front-left and back-right.** The loop cost is the same for any pair. The reason is failure: if
  the Expansion Hub drops out, a diagonal pair can still translate without turning, and a same-side pair cannot (from
  Pedro's mecanum wheel equations; untested on a robot).
- **Not all four drive motors on the Control Hub.** That would move both flywheels to the Expansion Hub. Flywheel
  power changes every loop even when parked, and drive power only while driving, so the parked loop grows by 2.1 ms
  (20.8 against 18.7) and a driving loop with drive writes in half the loops by 1.0 ms; at the ceiling the two are
  equal. It would also leave the hardware-counted encoder ports 0 and 3 under drive motors, so anything that resets
  a drive motor would reset a flywheel's count. What it would buy is full driving if the Expansion Hub drops out.
  Shooting stops in that case with either layout, because the transfer is on the Expansion Hub.
- **All five aiming servos and the blocker on the Control Hub.** Turrets and hoods change every loop in
  shoot-on-the-move. The blocker gates each shot: 1.55 ms on the Control Hub (measured) against about 2.45 ms.
- **Rarely changing outputs on the Expansion Hub**: intake, transfer motor, transfer servo, flower intake, lights.
- **Flywheel and turret encoder counts on hub ports, not the OctoQuad.** On the Control Hub they ride the bulk read
  the loop already pays. Reading the OctoQuad's encoder block with its localizer block costs 6.58 ms against
  3.20 ms, +3.4 ms in every loop that reads it (measured): the table's row has it in every loop. The simulation in
  [`flywheel-control-findings.md`](flywheel-control-findings.md#where-to-plug-the-flywheel-encoders) found the
  OctoQuad's cleaner speed reading does not make the shooter fire faster. It also would put odometry, the turret and
  both flywheels behind one I2C cable.
- **The nectar turret's absolute output on the OctoQuad, read once at INIT** (decided 2026-11-12). The count on
  encoder port 2 has no zero of its own and the CAD has no hard stop to home against; one read of the absolute
  output at INIT gives it one, and every loop then has the true angle for nothing. Reading the absolute output
  every loop instead is an alternative under [Open choices](#open-choices), at +3.4 ms in every loop.
- **OctoQuad on Control Hub I2C bus 2.** Every I2C read is several hub commands, and each is assumed to pay the RS485
  hop on the Expansion Hub (not measured there). Bus 0 is the built-in IMU's. Bus 2 is where the bench measured it.
- **Prism on its own bus.** That separates its data lines from the OctoQuad's. It does not separate power: all of a
  hub's I2C ports share one 3.3 V supply (500 mA total, REV's specifications).
- **Analog on the Control Hub, read in the loop.** `Drivetrain.read()` reads the Floodgate after every driving loop
  (`Drivetrain.java:97-99`). `TransferCounter` needs both rangefinders on one hub.
- **Cameras on USB 3.0.** The USB 2.0 port shares a bus with the hub's radio
  ([ftc-docs hub tips](https://ftc-docs.firstinspires.org/tech_tips/tech-tips/tech-tip-hub-tips/tech-tip-hub-tips.html),
  fetched 2026-11-10).
- **Lights on separate servo port pairs.** Ports 0-1, 2-3 and 4-5 each share one 2 A supply (ftc-docs hub tips).
  The lights take hub 5 V because the injectors' 6 V may power only servos (R613(C)). Their current draw is not
  published.
- **Injectors split by shooter.** They carry no hub commands, so the split costs no loop time, and one injector
  failing then stops one shooter's aim instead of both. The two pollen turret servos turn one gear: keep them on one
  injector and one hub so they fail together.

Against the layout planned before the CAD: three turret servos instead of four saves one write per aiming loop; that
frees a Control Hub servo port for the blocker; the nectar turret encoder, the Floodgate, the transfer servo, the
lights, the Prism and the injectors are new.

## Estimated loop period

`python3 -I scripts/looptimes/wiring_budget.py` prints every number in this section and the table above. The model:
period = (hub commands + CPU) / (1 − share of time spent on the SDK's 500 ms refresh).

| per loop, ms | parked, aiming | driving and aiming | source |
|---|---|---|---|
| Control Hub bulk read | 2.22 | 2.22 | measured |
| OctoQuad localizer read | 3.20 | 3.20 | measured |
| 2 flywheel writes | 3.28 | 3.28 | 2 × 1.64 measured |
| 4 drive writes, 2 on each hub | 0 | 8.36 | 2 × 1.64 measured + 2 × 2.54 estimated |
| 3 turret and 2 hood writes | 0 | 7.75 | 5 × 1.55 measured |
| SDK between loops, telemetry, follower and other CPU | 6.93 | 6.93 | measured on the bench and the Betta bot ([`loop-time-findings.md`](loop-time-findings.md)) |
| SDK refresh of every output, per 0.5 s | 82 | 66 | estimated from SDK source |
| **period** | **18.7** | **36.6** | |
| the same with every cost at its p95 | 22.4 | 45.1 | |

- **36.6 ms is a ceiling.** It assumes all four drive motors write every loop. `Drivetrain` skips changes under 0.05
  and Pedro's own motor wrapper under 0.01. With each drive motor writing in 0 / 25 / 50 / 75% of loops the period is
  27.5 / 29.8 / 32.1 / 34.4 ms. The Betta bot's measured cost of driving on the same 2 + 2 split gives 33.5 ms.
- **With the dashboard off**, as at events: about 2.1 ms less (measured on the bench with no Driver Station
  connected): 16.2 parked, 34.2 driving at the ceiling, 29.6 with drive writes in half the loops.
- **The loop that starts a shot** adds about 6.5 ms: blocker, transfer motor, transfer servo.
- **The refresh sets the spikes, not the wiring.** Every output the code keeps writing costs one to three extra hub
  commands every 0.5 s ([`loop-time-findings.md`](loop-time-findings.md#how-often-a-write-re-send-reaches-the-hub)):
  66 to 82 ms of hub time per 0.5 s on this map. Outputs first written together expire together, so that time bunches
  into a few loops. Spreading the re-sends out is a code change nobody has designed.
- **With Photon, which the team is using** ([`photon-bench.md`](photon-bench.md)): 16.0 / 24.2 ms with the
  Expansion Hub on USB, and 22.2 ms driving with drive writes in half the loops; 16.4 / 28.1 ms if it stayed on
  RS485 (estimated from the fork's bench on 2026-10-29). The table above is the fallback without Photon. No device
  moves between the two: only the Expansion Hub's cable changes. Without Photon a USB Expansion Hub write is no
  faster than RS485, so the fallback is RS485.
  The map stays the shortest of the layouts modelled under Photon: with all four drive motors on the Control Hub
  the same two figures are 20.4 / 28.1 on RS485 and 16.5 / 24.2 on USB, and with the turret servos on the
  Expansion Hub 16.4 / 33.5 and 16.0 / 25.0. With Photon on both hubs every layout is within 1 ms of the map.
- **Two swaps could still shorten it under Photon on USB** (estimated 2026-11-11, same model, parked / driving):
  - The pollen turret pair on one port through a Y-cable: 15.9 / 23.2 ms, 1.0 ms less while aiming. It costs the
    per-servo offset ([Open choices](#open-choices)).
  - The nectar turret and nectar hood servos on Expansion Hub servo ports 3 and 5, a 7 + 4 split of the every-loop
    writes: 16.1 / 24.8 ms if the two hubs' write times add up, as modelled, and 16.1 / 20.2 ms if they overlap
    fully, against 16.0 / 21.8 ms for the map in that case. Which it is has not been measured
    ([Still unknown](#still-unknown)). Until it is, wire the map and leave those two signal leads long enough to
    reach either hub; both servos stay on injector B either way.
- **Not counted:** the shot solver's CPU time (unmeasured) and garbage collection (about 5% on the Betta bot).

## Open choices

Each row has a default that the map uses and the test that would change it.

| choice | default | alternative | what the alternative costs or buys | test |
|---|---|---|---|---|
| Flywheel Through Bores | Control Hub encoders 0 and 3 | OctoQuad channels 2 and 3, speed computed on the board at a 10 to 20 ms interval | +3.4 ms every loop, or nothing more if the turret's absolute output were read every loop (an alternative below); removes timestamp noise from the RPM; rated 1,000,000 counts/s where the hub has no published limit | Spin each flywheel unloaded to free speed on a full battery (5800 RPM by the vendor's test, 792,000 counts/s) with the encoder on port 0 or 3 and count for 10 s against the OctoQuad or a tachometer. Any lost counts: move to the OctoQuad. |
| Nectar turret Through Bore | the count on Control Hub encoder 2, zeroed from the absolute output on OctoQuad channel 4 read once at INIT (decided 2026-11-12, until Bench: Turret Zero has run on a hub: [below](#zeroing-the-nectar-turret)) | the absolute output alone on OctoQuad channel 4, either read every loop (as the team first reported on 2026-11-11) or read at INIT and now and then, for example once the servo has settled, with the commanded angle used in between ([The nectar turret encoder](#the-nectar-turret-encoder)) | Every loop: +3.4 ms per loop. Now and then: +3.4 ms only in the loops that read, and between reads an angle that is only the command. Either needs no split cable (the default's is home-made and must be routed across the turret) and keeps no count in software under a drive motor's port number; now and then is the fallback if that cable can't be built or routed | Bench: Turret Zero and Bench: OctoQuad's bank phase ([below](#zeroing-the-nectar-turret)). Then sweep the turret end to end at full servo speed a few hundred times, back to a fixed mark, and log the count: any drift means reading the absolute output instead. |
| Last two Control Hub motor ports | two drive motors | intake and transfer, all four drive motors on the Expansion Hub | 0 to 1.9 ms more per driving loop; puts the turret encoder under a motor that no path follower or tuner touches; makes `isOverCurrent()` jam checks free | Decide by whether the loop needs a jam check every loop. |
| Pollen turret pair | one port each | both on one port through a Y-cable (R505 allows two servos per port) | saves 1.55 ms per aiming loop and frees a servo port; loses the per-servo offset in `Turret` | On the robot: check the two servos do not fight with no offset. |
| Battery voltage for flywheel compensation | every 50 loops, as now | every loop while a flywheel runs, or every 2 to 3 loops | +1.3 ms, or +0.4 to 0.6 ms, per loop; the simulation had the first ball 145 to 160 RPM low under drive sag with the 50-loop read | Fire bursts during hard accelerations at each cadence and log RPM as each ball arrives. |
| Transfer rangefinders | read in the loop | a 10 ms poller thread | measured +3.0 ms on a 14 ms loop; estimated +7 to +10 ms on this one; one failed poll crashes the OpMode | Count 100 balls, including 10-ball bursts, with in-loop counting at the real loop period. A miss means a poller, and then only with the Expansion Hub on its own USB cable. |

Two cautions on the nectar turret's count on encoder port 2:

- Port 2 counts in software, and the only published guidance says encoders over about 4000 counts per revolution
  should not go on ports 1 and 2 if they spin fast
  ([gm0](https://gm0.org/en/latest/docs/software/adv-control-system/control-system-internals.html), fetched
  2026-11-10). The turret is slow: about 36,400 counts/s at the servo's no-load speed on 6 V (672°/s at the turret
  × 54.18 counts per degree). The CAD puts the encoder on the pinion shaft, 50/21 turns per turret turn
  ([`robots-findings.md`](robots-findings.md)).
- It is a relative count under a drive motor's port number. Anything that resets or changes the mode of that motor
  moves the turret's zero until the next INIT reads the absolute output again, and the CAD has no hard stop to home
  against. A mode change also re-sends the motor's last power and enables its port (SDK 12.0.0
  `LynxDcMotorController.setMotorMode`, source).

### Zeroing the nectar turret

The plan (decided 2026-11-12, for now), and untested: the hub counts the turret on encoder port 2 and the loop reads
that count from the bulk read it already pays, so every loop has the true angle for nothing. The encoder's absolute
output, read once at INIT from the OctoQuad, sets the count's zero. Reading the turret from the OctoQuad every loop
would cost +3.4 ms per loop (measured: 6.58 ms against 3.20 ms). The encoder's facts, and the two alternatives that
read only its absolute output, are in [The nectar turret encoder](#the-nectar-turret-encoder).

- **Wire both outputs of the one encoder through one home-made split cable.** A, B, 3.3 V and ground go to Control
  Hub encoder port 2. The absolute (pulse) wire and ground go to the A pin of OctoQuad channel 4; that plug's 3.3 V
  and B pins stay unconnected. Power the encoder from the hub only. No stock cable does this: REV's hub cable
  (REV-31-1815) carries A and B only.
- **Set the OctoQuad's second bank to pulse width at INIT.**
  `setChannelBankConfig(BANK1_QUADRATURE_BANK2_PULSE_WIDTH)` makes channels 4 to 7 pulse-width inputs and leaves 0
  to 3 counting, so the pods on 0 and 1 are untouched. The setting is lost at power-off unless saved to flash, so
  set it at every INIT.
- **Read channel 4 once at INIT**, turn the pulse width into the encoder's angle, and use the hub count from then
  on. The loop's OctoQuad read stays the localizer-only read, 3.20 ms.
- **Pick the turn from a rough angle**, at INIT only. The absolute output repeats every 150° of turret at the team's
  1:2.4 (151.2° at the CAD's 50/21), and the 200° of travel covers 1.33 encoder turns (the servo's full range, 2.1
  to 2.3). A rough angle within 75° of the truth picks the right turn: set the turret by hand within 75° of a mark
  before INIT, or use the commanded servo angle once the servo has settled (the turret follows the servo 1.008 to 1).
- **Expect steps of about 0.15° of turret** if the OctoQuad reports the pulse in whole microseconds (not
  confirmed): 0.35° of encoder on a V1 (1 to 1024 µs), 0.36° on a V2 (3.884 to 998.06 µs).

How each part is known, all read on 2026-11-11 and none of it run:

- The bank setting: SDK 12.0.0 source, `OctoQuad.ChannelBankConfig`. Pedro revhub 3.0.2's `OctoQuadLocalizer`
  never sets it (source). `getChannelBankConfig()` returns ALL_QUADRATURE for any byte it doesn't recognise, the
  all-zero reply of a failed I2C read included (`OctoQuadImpl`, source): only a read-back of the pulse-width setting
  shows the set took.
- The INIT read: `readAllEncoderData` with a `crcOk` check. In the driver's default AUTO caching mode the first
  `readSinglePosition_Caching` call returns the zero-filled cache without reading (`OctoQuadImpl`, source).
- The pulse goes on a channel's A pin, and the inputs are 3.3 V only: OctoQuad datasheet Rev 3.0C, sections 2.1
  and 3.1.
- The encoder puts out the counting and absolute signals together with its switch on A: REV's
  [application examples](https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/v1/application-examples).

Test: the localizer read's cost with the second bank in pulse-width mode against all channels counting, and whether
the bank survives the localizer's setup and reset, are Bench: OctoQuad's last phase, which runs whenever an OctoQuad is
on the bench (`python3 scripts/bench/run_bench.py --octoquad`, or a run without bench flags). The rest, with one
encoder wired this way, is Bench: Turret Zero (`python3 scripts/bench/run_bench.py --turret-zero --ports-empty`,
prompted, about 25 minutes): the hub count against the absolute angle at a dozen rests over two turns, ten re-INITs,
run-mode changes and, with a lab supply, a brownout
([`scripts/bench/README.md`](../scripts/bench/README.md#bench-turret-zero)).

In a loop at 34 to 37 ms, in-loop sampling sees about half of the rangefinders' 20 ms readings. Whether that misses
balls is what the counting test settles.

## The nectar turret encoder

Reported by the team on 2026-11-11: the turret is geared to its encoder 1:2.4, the turret's travel is 200°, and the
encoder is a REV Through Bore V2. Everything else in this section is computed from that, from the parts' pages
([`parts-findings.md`](parts-findings.md)) and from SDK 12.0.0's `OctoQuad` source. Nothing is measured.

The plan reads the absolute output once, at INIT, to zero the hub's count
([Zeroing the nectar turret](#zeroing-the-nectar-turret)). This section describes the encoder, and how the two
alternatives, which are not the plan, would read its absolute output during the match. Both need only the absolute
wire, on OctoQuad channel 4, and no count. One reads it every loop, as the team first reported on 2026-11-11: +3.4 ms
in every loop (measured: 6.58 ms against 3.20 ms). The other reads it at INIT and now and then, for example once the
servo has settled, and uses the commanded angle in between: +3.4 ms only in the loops that read. That one is the
fallback if the split cable can't be built or routed across the turret.

- **The encoder turns 2.4 times for one turn of the turret** (inferred). That matches the CAD's shaft between the
  servo and the turret ([`robots-findings.md`](robots-findings.md)); the other reading, an encoder turning 1/2.4 of
  the turret, matches nothing in the CAD. Confirm with the team before writing the conversion.
- **Use the tooth counts, not 2.4.** In the CAD of 2026-11-10 that shaft turns 150/63 = 2.381 times per turret turn,
  and exactly 48/20 = 2.4 times per servo turn. If the turret stage is still 63:150, converting with 2.4 reads 0.8%
  short: 0.8° at 100° from the middle. Ask for the two tooth counts between the encoder's shaft and the turret.
- **Across 200° of turret the encoder turns 480°, so its absolute output repeats.** One encoder turn is 150° of
  turret. Turret angles 150° apart give the same pulse width, and the first and last 50° of travel overlap.
- **Pick the turn from the commanded angle.** In code:
  `turret° = zero + (pulse − min) / (max − min) × 150 + 150 × k`, with `k` the whole number that lands nearest the
  angle the servo was last sent to (or, at INIT, a hand-set mark). The two candidates are 150° apart, so the pick is
  right while the turret is within 75° of that angle. With the plan the pick is needed only at INIT: the count
  carries the angle from there. Read during the match, as the alternatives do, it runs on every read, needs no
  homing (the CAD has no hard stop), cannot drift, and is right again on the first read after an OctoQuad or hub
  reset.
- **Pulse width needs a bank-2 channel.** The OctoQuad reads pulse widths on channels 4 to 7 with
  `ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH`, which keeps the pods' channels 0 and 1 on quadrature
  (`OctoQuad.java:127-143`). Use channel 4. The bank mode and the channel's pulse limits are lost at power-off unless
  saved to flash: set them at every INIT.
- **Leave the OctoQuad's own wrap tracking off** (`setSingleChannelPulseWidthTracksWrap`). It counts turns from
  wherever the turret stood when the OctoQuad powered up, so it needs the same turn pick at start and loses it on a
  reboot. With it off the channel reports the raw pulse width in microseconds (`OctoQuad.java:368-393`).
- **Measure this encoder's shortest and longest pulse.** REV's V2 page gives 3.884 to 998.06 µs; the SDK's notes say
  Through Bores have been seen about 10 µs off their sheet, which is 1.5° of turret here. Turn the turret slowly
  through the wrap point and note the two values it jumps between.

| | absolute output | counting output (A/B) |
|---|---|---|
| Wired to | OctoQuad channel 4 (pulse width needs 4 to 7) | Control Hub encoder 2 (on the OctoQuad it would need 0 to 3; the pods are on 0 and 1) |
| Per turret degree at 2.4 | 6.63 µs: 0.15° per µs if the register is whole microseconds (not confirmed) | 54.6 counts: 0.018° per count |
| Across 200° | 1.33 encoder turns | 10,923 counts |
| Known at power-up | yes, to within the turn pick | no: needs a zero, and there is no hard stop |
| After an OctoQuad reboot | right on the next read, once the bank is set again | on the hub, untouched; on the OctoQuad, zero lost |
| Sensor accuracy | ±0.5° of encoder (REV), ±0.21° of turret | the same sensor |
| Rate at full servo speed | 1 kHz pulse train: a new value each 1 ms | about 36,400 counts/s (estimated from the vendor's servo speed): under 4% of the OctoQuad's rating; port 2 has no published rating ([the cautions](#open-choices)) |

- **The encoder does not see backlash in the last gear pair.** It sits before the turret's ring gear, so slop there
  is an aim error it cannot report. Measure it: hold the servo still, rock the turret by hand, and compare the swing
  at the flywheel with the swing in the encoder's angle.
- **Reading it during the match**, as both alternatives do, takes the encoder block (above: +3.4 ms p50 in each loop
  that reads it) in one transaction with the localizer, which Pedro's `OctoQuadLocalizer` does not do. The plan's
  one read is at INIT, outside the loop, and leaves the loop's OctoQuad read localizer-only.

## What the code must do

For this wiring to work:

- **Read the turret count raw:** `br.getController().getMotorCurrentPosition(br.getPortNumber())`, not
  `br.getCurrentPosition()`. The SDK negates `getCurrentPosition()` by the motor's direction and type; with the
  goBILDA 5202 type every config uses, that happens when the direction is `FORWARD`, which is today's setting (read
  from SDK 12.0.0 source, `DcMotorImpl`). The raw read still comes from the bulk cache. Bench: Turret Zero's sign
  phase checks the sign, and the hub's own read against the bulk read, on a hub.
- **Never reset or change the mode of the drive motor that shares the turret's port.** What can: Pedro's two-wheel
  and three-wheel localizers and the vendored tuners built on them (they `STOP_AND_RESET_ENCODER`), `setVelocity`
  (switches to `RUN_USING_ENCODER`), the dashboard's Hardware OpMode run-mode control, and the Driver Station's
  manual-control encoder reset. Check `RUN_WITHOUT_ENCODER` at INIT and fail if not.
- **Zero the turret in software at INIT, and cross-check it.** Turn channel 4's one pulse into the turret angle as
  in [Zeroing the nectar turret](#zeroing-the-nectar-turret), with the gear tooth counts, the measured pulse limits
  and a zero offset as tunables ([The nectar turret encoder](#the-nectar-turret-encoder)), and set the count's zero
  from it. Never zero from a pulse outside the limits or a read that fails its `crcOk` check. Then compare the
  counted angle with the commanded angle once the servo has settled; on a jump, raise a Fault and fall back to the
  commanded angle.
- **One Control Hub bulk read at the top of the loop, with a timestamp.** Call `getBulkData()` on the Control Hub
  where `EnhancedOpMode` now clears caches (`architecture/core/EnhancedOpMode.java:297`, `:407`), stamp it, and have
  `Shooter.measure()` (`biobuzz/modules/Shooter.java:333-343`) use that stamp. Keep `clearBulkCache()` on the
  Expansion Hub: a hub that is never cleared returns its first snapshot for the rest of the OpMode.
- **One failed-read flag per loop.** A failed bulk read returns all zeros without an exception
  ([`sdk-hub-findings.md`](sdk-hub-findings.md)). Set a flag from `isFake()` and have every reader skip that loop:
  the flywheel RPM history, the turret angle, the transfer counter (it would count a ball twice) and the Floodgate
  limiter.
- **Fetch the battery sensor by name** ("Control Hub"), not `hardwareMap.voltageSensor.iterator().next()`
  (`EnhancedOpMode.java:114`), which returns the Control Hub only by the order of a hash map. Ignore readings under
  6 V: a failed read returns 0 and is held for 50 loops.
- **Detect a lost Expansion Hub.** With nothing read from it the code cannot see it drop. Poll
  `LynxModule.isNotResponding()` on both hubs each loop (it sends no command) and raise a Fault.
- **Keep current reads off the Expansion Hub in a match.** `Intran`'s two (`biobuzz/modules/Intran.java:106-107`)
  would cost 2 × 2.61 = 5.2 ms per telemetry loop there (measured per read).
- **Count transfers in the loop.** `TransferCounter` runs only as a thread today.
- **OctoQuad:** a localizer that times out with a named error at INIT and checks every read
  ([`hub-bench-findings.md`](hub-bench-findings.md)); pods on channels 0 and 1 (`OctoQuadLocalizerTest` uses 3 and
  4). At INIT it must set the bank mode and the turret channel's pulse limits, read the bank back to see the set
  took, and read channel 4 once (`readAllEncoderData`); in the loop it reads the localizer block only. Either
  alternative would instead read the localizer and encoder blocks in one transaction
  (`readLocalizerDataAndAllEncoderData`) in each loop that reads the turret, and hand the turret its channel. What a
  bad read does mid-match (crash, or Fault and carry on) is a team decision.

Worth doing with it:

- Send each flywheel power when it is computed. Today it is computed in `read()` (`Shooter.java:205`) and sent in
  `write()` (`:345`), with the drive writes in between.
- Loosen the hood write tolerance from 0.001 to 0.002-0.005 (`Shooter.java:174-175`): at one hood write per loop
  that is about 1.8 ms off the driving loop.
- Remove or gate `Drivetrain.encoderTelemetry`'s lines (`Drivetrain.java:256-262`): the back-right velocity would be
  the turret's, and the Expansion Hub lines would start its bulk read.

## What fails, and how this wiring behaves

- **Expansion Hub or its cable lost:** both shooters still spin and aim, and the blocker can still stop feeding.
  Intake, transfer and two drive motors are gone. The first write to the lost hub blocks the loop for about 250 ms
  and later ones fail at once without an error (read from SDK source, without Photon). On USB the one mini-USB plug
  is the weak point: tie the cable to the frame beside it. On the RS485 fallback, two cables are the main
  protection. How long the lost hub's motors keep their last power is unverified.
- **Control Hub reset or brownout:** the turret's count is lost until the next INIT reads the absolute output again
  (inferred, not tested), and nothing reports it; hence the cross-check above. The OctoQuad is powered from the hub's
  I2C port, so it may reboot too and come back with its bank mode on the default (inferred, not tested): channel 4
  then gives no pulse width until the mode is set again, which is why every INIT sets it and reads it back. Saving
  the settings to flash would avoid that. Bench: Turret Zero's brownout phase tests the count through a dip and the
  INIT read after it; it needs the Control Hub on a lab supply
  (`python3 scripts/bench/run_bench.py --turret-zero --ports-empty`).
- **OctoQuad unplugged or rebooted:** odometry is lost for the rest of the match (its localizer restarts at the
  origin and its settings are volatile). The turret keeps its angle, which comes from the hub's count, and the
  flywheels are unaffected. The restart and the lost settings are untested: Bench: OctoQuad's chain pulls the
  cable mid-run and reads what comes back
  (`python3 scripts/bench/run_bench.py --octoquad --octoquad-chain --interactive --ports-empty`).
- **Camera hub's power bank flat:** the webcam fails to open, `WebcamSession.update()` throws, and every OpMode that
  builds `BioBuzzRobot` stops in INIT. Check the power bank before each match. Whether a webcam failure should be a
  Fault instead is a team decision, as it was for the Limelight.
- **A camera cable crosses each turret**, and the nectar turret's travel is 200°: strain-relieve both, and the
  encoder's split cable wherever it crosses the turret.

## Where the hubs sit, and the USB cable between them

Read from the CAD model on 2026-11-11 unless marked. The model draws both hubs as Control Hubs with no ports, so it
does not say which one is the Expansion Hub, and port positions are estimated (±0.15 in) from REV's
[dimension drawing](https://docs.revrobotics.com/duo-control/control-system-overview/dimensions).

- One hub stands in each wheel bay, between the inner and outer side plates, with its base on the inner plate and its
  connectors facing outboard. On both, the edge with the USB ports faces the rear. A window in each outer plate lets
  a hand reach the ports (measured on a render, ±0.1 in).
- **The hub in the bay without the battery is clear:** nothing sits within 2.2 in behind its USB ports (render), so
  a straight plug fits on any of them.
- **The hub in the battery's bay is blocked as drawn.** The battery holder's front wall is 0.16 in behind the hub's
  USB edge and covers the position of the USB-C port (the mini-USB port on an Expansion Hub). A servo power injector
  is 0.29 in behind the USB 2.0 position (render). Only the USB 3.0 position has an opening, about 1 in tall,
  between the two, and the battery itself is not modelled.

What follows:

- **The Control Hub belongs in the bay without the battery.** In the other bay its USB-C port, which deploying by
  cable at events needs ([`game-findings.md`](game-findings.md)), is behind the battery holder, and one usable
  USB-A port would have to serve both the camera hub and a later Expansion Hub link.
- **An Expansion Hub in the battery's bay cannot take a mini-USB plug as drawn.** The USB link needs one: move the
  battery holder or the hub to leave room for a plug and the cable's bend.
- **The cable for a USB link:** USB 2.0, USB-A male to Mini-B male, shielded, 3 ft (0.9 m). The route that stays
  clear of the transfer goes rearward from each hub along its wheel bay above the rear wheel, inboard through a
  pocket in the inner plate, and across inside the upper rear cross channel: 25 to 26 in (estimated from the plate
  and hub positions), 28 to 29 in with bends. A 2 ft cable is too short; 3 ft leaves about 7 in to tie down.
  Straight across behind the hubs is 15 to 16 in, but through the transfer chute's space.
- **Cables that meet it:** Monoprice 105447: 3 ft, straight plugs, 28/24 AWG, braid plus foil shield, ferrite
  core, by its Amazon listing (<https://www.amazon.com/dp/B003L1380E>, read 2026-11-11: $7.01, sold and shipped
  by Amazon, 4.7 stars from 925 ratings). Tripp Lite (Eaton) U030-003: 3 ft, straight plugs, tinned copper braid
  plus foil shield, moulded plugs with strain relief
  ([spec sheet](https://www.markertek.com/Attachments/Specifications/TRIPPLITE/U030-003-Specifications.pdf), read
  2026-11-11; wire gauge not stated); on that date its only Amazon listings were from a third-party seller with
  85% positive feedback and no product ratings. StarTech's straight USB2HABM3 is listed as 30 AWG with no shield
  stated. Its angled versions, USB2HABM3RA and USB2HABM3LA, list foil plus braid and need less room behind the
  port (retailer listings only); which of the two points the cable the right way has to be checked on the hub.
- **Tie it clear of** the rear wheels and drive belts under the run, the battery's slide path, the odometry pods at
  the rear corners (they move up and down) and the end of the transfer chute near the cross channel.
- **A USB 3.0 cable gains nothing.** The Expansion Hub's socket is a 5-pin Mini-B, which has no USB 3.0 contacts,
  and the SDK runs the link as a serial port at 460,800 baud (read from SDK 12.0.0 source,
  `LynxConstants.SERIAL_MODULE_BAUD_RATE`), far below what USB 2.0 carries.
- **Which Control Hub port: through the camera hub on USB 3.0, leaving USB 2.0 empty.** The USB 2.0 port shares a
  bus with the hub's Wi-Fi radio, and interference on a device there can drop communications
  ([ftc-docs hub tips](https://ftc-docs.firstinspires.org/tech_tips/tech-tips/tech-tip-hub-tips/tech-tip-hub-tips.html),
  fetched 2026-11-10), which stops the whole robot; no bench test covers static discharge. Behind the camera hub a
  fault costs two drive motors, the intake, the transfer and the cameras, but not the radio, and that can be tested.
  R707 allows a USB hub and an Expansion Hub on USB, and nothing we found forbids one behind the other.
- **Before trusting the Expansion Hub behind the camera hub** (untested on our hubs): with an OpMode running, unplug
  the hub's power bank and then each camera, and check the Expansion Hub stays connected; then run the Photon USB
  bench on that wiring with both cameras streaming.
- **Mini-USB pulls out easily.** REV's USB Retention Mount (REV-41-1214, $13.50 and in stock on 2026-11-11,
  <https://revrobotics.com/rev-41-1214>) is a base that sticks to the Expansion Hub and a cap that screws onto it
  over the plug; the mount is 27.6 x 23.7 mm and 14.1 mm tall
  ([drawing](https://revrobotics.com/content/docs/REV-41-1214-DR.pdf)). The kit's cable is a custom right-angle
  Mini-B to USB-A, 457 mm, and REV only claims the mount holds "when used with the included right-angle USB
  cable"; whether the cap fits another cable's plug is unverified, and a search on 2026-11-11 found nobody
  reporting it either way. REV's page gives no shielding or wire gauge for its cable. So there are two ways to buy:
  - the REV kit plus a USB 2.0 A-male to A-female extension of about 1.5 ft, which reaches the route at 36 in and
    locks the plug at the hub, with one joint in the run to tape and tie down (Monoprice 5431 is 1.5 ft and
    28/24 AWG by retailer listings; its shielding is not confirmed);
  - or the straight 3 ft cable above without the mount, tied to the frame near the plug.

## Rules and limits that bound the map

From the Competition Manual revision named in [`game-findings.md`](game-findings.md), and REV's and goBILDA's pages,
fetched 2026-11-10:

- R503: at most 8 motors and 8 servos. The map uses all 8 of each, so there is no spare. `Intran` builds a ninth
  servo (`ruler`) and must be rewritten to the CAD before it is used.
- R505: two servos per hub servo port or injector port.
- R608: hubs and injectors are powered only by XT30 from the main battery. R613(C): injector 6 V powers servos only.
- R701(C), R707, R708, R611(A): one extra Expansion Hub, which may connect by USB; UVC webcams, the Limelight and a
  USB hub on USB; a USB battery pack may power that hub.
- Per hub: 4 motor ports, 4 encoder ports, 6 servo ports, 4 analog inputs, 8 digital channels, 4 I2C buses (SDK
  12.0.0 `LynxConstants`). Motor ports 10 A continuous; servo port pairs 2 A each, 5 A across all servo and 5 V
  ports; encoder and I2C supplies 3.3 V, 500 mA total each (REV).
- goBILDA servo power injector 3125-0001-0001: 6 channels, 8 to 15 V in, 6 V and 24 A out, no output on a channel
  until it sees a PWM pulse over 500 µs
  (<https://www.gobilda.com/6v-servo-power-injector-6-channel-8-15v-input/>).

## Still unknown

- **The highest count rate hub encoder ports keep up with**, hardware-counted or software-counted. No REV or FIRST
  figure exists. The two encoder tests above settle it for our speeds.
- **The cost of a write on an RS485 Expansion Hub.** Every 2.5 ms here is the Control Hub cost plus the hop measured
  on reads. Test: time 1000 changed `setPower` calls on each hub in one OpMode.
- **Whether the hubs tolerate USB and RS485 connected at once.** Only USB alone has been benched. Test: on bench
  hubs, connect both and read the hub list the SDK builds at INIT.
- **Whether Photon overlaps writes to two hubs on USB.** The model adds the two hubs' write times, as the bench's
  blocks did (4.66 + 6.20 ms for 6 writes on each). If writes to different hubs overlap, splitting the every-loop
  outputs more evenly would beat the map's 9 on the Control Hub and 2 on the Expansion Hub by up to 1.6 ms per
  driving loop ([Estimated loop period](#estimated-loop-period)). Test: the two-hubs-at-once
  phase at the end of Bench: Photon, in the Photon block's USB run
  (`python3 scripts/bench/run_bench.py --photon --usb-expansion --photon-i2c --ports-empty`). It times 6 writes on
  each hub alone, all 12 in one loop (hub by hub and alternating) and the 3 + 3, 5 + 1 and 1 + 5 splits, and the
  **Two hubs at once** line in `summary.md` says whether the hubs' times add up or overlap.
- **How often the drive motors really write** in TeleOp shoot-on-the-move. Test: a per-loop write counter in SOTM
  Test.
- **Whether the flywheel power changes every loop** once its RPM reading is clean; if not, parked loops are shorter.
- **Current draw** of the RGB lights, and of three Through Bores and two pods against the 500 mA encoder and I2C
  supplies.
- **The splitter cable's pin mapping** and which wire the Floodgate's sense connector uses; which Floodgate version
  is fitted (`Drivetrain` assumes the 80 A scale).
- **Whether the injector cuts its output** when the SDK disables PWM (`Turret.disable()` does).
- **Whether the OctoQuad's localizer reset zeroes its channel counts.** Its bank mode is volatile like its velocity
  interval (the SDK's Javadoc on `setChannelBankConfig`). Test: Bench: OctoQuad's chain
  (`python3 scripts/bench/run_bench.py --octoquad --octoquad-chain --interactive --ports-empty`) reads the counts after
  `resetLocalizerAndCalibrateIMU()` and after `resetEverything()`, and the bank, velocity intervals and directions after
  a cable pull.
- **The turret encoder**: which way the 1:2.4 runs and its tooth counts, and whether the 200° is 100° either side of
  straight back, are questions for the team. Whether the split cable can be built and routed across the turret (the
  absolute signal is on the Through Bore's 6-pin connector, not on the 4-pin encoder lead) is for whoever builds it;
  if it can't, the absolute wire alone, read at INIT and now and then, is the fallback. This encoder's pulse limits,
  whether the OctoQuad's pulse register is whole microseconds and the backlash after the encoder are tests in
  [The nectar turret encoder](#the-nectar-turret-encoder).
- **Whether the absolute output read once at INIT works as the nectar turret's zero**, which the plan rests on.
  Test: Bench: Turret Zero (`python3 scripts/bench/run_bench.py --turret-zero --ports-empty`) and Bench: OctoQuad's
  bank phase (`--octoquad`), under [Zeroing the nectar turret](#zeroing-the-nectar-turret); then on the robot,
  power-cycle with the turret at several angles and compare the angle the code reports with a protractor mark.
- **Where the USB ports really are on the mounted hubs**, and whether the battery clears them: check on the built
  robot, since the CAD's port positions are estimates.
- **Inspection questions** only an inspector or the Q&A can settle: the Axon servos' and the power bank's
  compliance, and a resistor divider on an analog port for battery voltage (R612's note allows high-impedance
  monitoring).

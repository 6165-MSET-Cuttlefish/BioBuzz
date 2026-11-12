"""Estimated loop period of the wiring map in docs/wiring-findings.md, added up from per-command hub costs.

M = measured, E = estimated. HB = docs/hub-bench-findings.md, LT = docs/loop-time-findings.md.
Run: python3 -I scripts/looptimes/wiring_budget.py
"""

SDK = (1.42, 1.69)                          # M HB: iterative OpMode with an empty loop(), p50 and p95
CH_BULK = (2.22, 2.97)                      # M HB: Control Hub bulk read; 2.10 and 2.47 in other runs
EH_BULK_EXTRA = (5.72 - 2.10, 6.27 - 2.43)  # M HB: both hubs' bulk reads minus the Control Hub alone
OQ_LOCALIZER = (3.20, 3.77)                 # M HB: OctoQuad readLocalizerData
OQ_COMBINED = (6.58, 8.51)                  # M HB: OctoQuad readLocalizerDataAndAllEncoderData
VOLTAGE = (1.30, 1.56)                      # M HB: getVoltage on the Control Hub
CH_MOTOR = (1.64, 1.96)                     # M HB: setPower, changed value
CH_SERVO = (1.55, 2.02)                     # M HB: setPosition, changed value
CH_READ = (1.27, 1.55)                      # M HB: uncached read; stands in for the SDK's mode re-read (E)
RS485_HOP = (0.90, 1.00)                    # E: HB measured it on ADC and bulk reads only; assumed for writes
TELEMETRY = 2.7                             # M LT: Ivy Path Test log, updateTelemetry mean
OTHER_CPU = 0.9                             # M LT: Ivy Path Test log, everything else
FOLLOWER_CPU = 7.7 - 5.79                   # E: LT follower.update paused minus HB Pinpoint read
IVY_DRIVING = 13.4 - 7.7                    # M LT: what driving added to follower.update, 2 + 2 drive motors
DASHBOARD_OFF = 6.32 - 4.21                 # M HB: framework loop, defaults minus dashboard telemetry off, no DS
PHOTON_CH = 4.66 / 6                        # E from M LT: fork bench 2026-10-29, 6 parallel writes
PHOTON_USB_EH = 6.20 / 6                    # E from M LT: same bench, Expansion Hub on USB
VOLTAGE_EVERY = 50
BURSTS_PER_S = 1.0
POLLER_BULK_MS = (2.0, 2.7)                 # M HB: the 10 ms poller's own bulk read under load

MOTORS = ('flywheel', 'drive', 'intake', 'transfer')
EVERY_LOOP = {
    'parked': ('flywheel',),
    'driving': ('flywheel', 'drive', 'turret', 'hood'),
    'firing': ('flywheel', 'drive', 'turret', 'hood'),
}
SHOT_EDGE = ('blocker', 'transfer', 'transfer_servo')


def devices(**hubs):
    return [(group, hub) for group, spec in hubs.items() for hub in spec]


MAP = devices(flywheel=['CH', 'CH'], drive=['CH', 'CH', 'EH', 'EH'], turret=['CH'] * 3, hood=['CH'] * 2,
              blocker=['CH'], intake=['EH'], transfer=['EH'], transfer_servo=['EH'], flower=['EH'],
              light=['EH', 'EH'])
ALL_DRIVE_ON_EH = devices(flywheel=['CH', 'CH'], drive=['EH'] * 4, turret=['CH'] * 3, hood=['CH'] * 2,
                          blocker=['CH'], intake=['CH'], transfer=['CH'], transfer_servo=['EH'], flower=['EH'],
                          light=['EH', 'EH'])
FLYWHEELS_ON_EH = devices(flywheel=['EH', 'EH'], drive=['CH'] * 4, turret=['CH'] * 3, hood=['CH'] * 2,
                          blocker=['CH'], intake=['EH'], transfer=['EH'], transfer_servo=['EH'], flower=['EH'],
                          light=['EH', 'EH'])
TURRET_SERVOS_ON_EH = devices(flywheel=['CH', 'CH'], drive=['CH', 'CH', 'EH', 'EH'], turret=['EH'] * 3,
                              hood=['CH'] * 2, blocker=['CH'], intake=['EH'], transfer=['EH'],
                              transfer_servo=['CH'], flower=['CH'], light=['CH', 'EH'])
PLAN_BEFORE_CAD = devices(flywheel=['CH', 'CH'], drive=['CH', 'CH', 'EH', 'EH'], turret=['CH'] * 4,
                          hood=['CH'] * 2, blocker=['EH'], intake=['EH'], transfer=['EH'], flower=['EH'])


def command(base, hub, q):
    return base[q] + (RS485_HOP[q] if hub == 'EH' else 0.0)


def write(group, hub, q, photon):
    if photon and hub == 'CH':
        return PHOTON_CH
    if photon == 'usb' and hub == 'EH':
        return PHOTON_USB_EH
    return command(CH_MOTOR if group in MOTORS else CH_SERVO, hub, q)


def budget(devs, situation, q=0, drive_fraction=1.0, octoquad_encoders=False, eh_input=False,
           voltage_every_loop=False, photon=None, ivy_drive=False, dashboard_off=False):
    io = CH_BULK[q] + (EH_BULK_EXTRA[q] if eh_input else 0.0)
    io += OQ_COMBINED[q] if octoquad_encoders else OQ_LOCALIZER[q]
    io += VOLTAGE[q] if voltage_every_loop else VOLTAGE[q] / VOLTAGE_EVERY
    refresh = edges = edge_loop = 0.0
    for group, hub in devs:
        motor = group in MOTORS
        w = write(group, hub, q, photon)
        enable = command(CH_MOTOR if motor else CH_SERVO, hub, q)
        mode_read = command(CH_READ, hub, q) if motor else 0.0
        changing = mode_read + enable
        held = mode_read + w + enable
        if group in EVERY_LOOP[situation]:
            f = drive_fraction if group == 'drive' else 1.0
            io += 0.0 if group == 'drive' and ivy_drive else f * w
            refresh += f * changing + (1 - f) * held
        else:
            refresh += held
            if situation == 'firing' and group in SHOT_EDGE:
                edges += 2 * w * BURSTS_PER_S * 0.5
                edge_loop += w
    if ivy_drive and 'drive' in EVERY_LOOP[situation]:
        io += IVY_DRIVING
    cpu = SDK[q] + TELEMETRY + OTHER_CPU + FOLLOWER_CPU - (DASHBOARD_OFF if dashboard_off else 0.0)
    base = io + cpu
    return dict(io=io, cpu=cpu, base=base, refresh=refresh, edges=edges, edge_loop=edge_loop,
                period=base / (1 - (refresh + edges) / 500.0))


def row(name, devs, **kw):
    cells = []
    for situation in ('parked', 'driving', 'firing'):
        p50 = budget(devs, situation, 0, **kw)['period']
        p95 = budget(devs, situation, 1, **kw)['period']
        cells.append(f"{p50:5.1f} ({p95:5.1f})")
    print(f"{name:52s} " + "  ".join(cells))


def with_poller(devs, situation, poll_ms):
    b = budget(devs, situation)
    period = b['period']
    link_busy = (b['io'] + b['refresh'] / 500.0 * period) / period
    t = period
    for _ in range(50):
        t = period + link_busy * (t / 10.0) * poll_ms
    return t


if __name__ == '__main__':
    print("Loop period, ms: p50 costs (p95 costs), for parked / driving and aiming / the same while firing")
    print("period = (hub commands + CPU) / (1 - ms of SDK refresh and shot edges per 500 ms)\n")
    row("the map", MAP)
    for f in (0.0, 0.25, 0.5, 0.75):
        row(f"the map, drive motors write in {f:.0%} of loops", MAP, drive_fraction=f)
    row("the map, driving cost from the Ivy log", MAP, ivy_drive=True)
    row("the map, dashboard off", MAP, dashboard_off=True)
    row("the map, dashboard off, drive writes in 50%", MAP, dashboard_off=True, drive_fraction=0.5)
    print()
    row("any input on the Expansion Hub", MAP, eh_input=True)
    row("OctoQuad encoder block read every loop", MAP, octoquad_encoders=True)
    row("turret servos on the Expansion Hub", TURRET_SERVOS_ON_EH)
    row("flywheels on the Expansion Hub, 4 drive on CH", FLYWHEELS_ON_EH)
    for f in (0.0, 0.5):
        row(f"  the same, drive motors write in {f:.0%} of loops", FLYWHEELS_ON_EH, drive_fraction=f)
    row("intake and transfer on CH, 4 drive on EH", ALL_DRIVE_ON_EH)
    for f in (0.0, 0.5):
        row(f"  the same, drive motors write in {f:.0%} of loops", ALL_DRIVE_ON_EH, drive_fraction=f)
    row("battery voltage every loop", MAP, voltage_every_loop=True)
    row("layout planned before the CAD", PLAN_BEFORE_CAD)
    print()
    row("Photon on the Control Hub, Expansion Hub on RS485", MAP, photon='rs485')
    row("Photon on both hubs, Expansion Hub on USB", MAP, photon='usb')
    for mode in ('rs485', 'usb'):
        for name, devs in (("flywheels on the EH, 4 drive on CH", FLYWHEELS_ON_EH),
                           ("intake and transfer on CH, 4 drive on EH", ALL_DRIVE_ON_EH),
                           ("turret servos on the EH", TURRET_SERVOS_ON_EH)):
            row(f"  Photon, {mode}: {name}", devs, photon=mode)
    print()
    for situation in ('parked', 'driving', 'firing'):
        b = budget(MAP, situation)
        print(f"{situation:8s} hub commands {b['io']:5.2f}  CPU {b['cpu']:.2f}  refresh {b['refresh']:.1f} ms per 0.5 s"
              f"  shot edges {b['edges']:.2f} ms per 0.5 s  shot-start loop +{b['edge_loop']:.2f}"
              f"  whole refresh in one loop {b['base'] + b['refresh']:.1f}")
    print()
    for poll_ms in POLLER_BULK_MS:
        print(f"10 ms poller on the shared link, {poll_ms} ms per poll: driving "
              f"{budget(MAP, 'driving')['period']:.1f} -> {with_poller(MAP, 'driving', poll_ms):.1f} ms")

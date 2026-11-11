"""Writes results_cad.json (compact, every number the write-up quotes) and prints the hood-line and budget tables."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import flysim_cad as F
import cadcfg as C
import run_cad as R

S = json.load(open(os.path.join(R.OUT, 'summary.json')))
sens = os.path.join(R.OUT, 'sens_summary.json')
DS = [36, 60, 84, 108, 132]
FAST = ('fast_v_3', 'fast_v_5', 'fast_v_10', 'fast_c_10', 'fast_vh_10', 'fast_ch_10')


def slim(d):
    out = {}
    for k, v in d.items():
        if v.get('infeasible'):
            out[k] = dict(infeasible=True, frac=v['frac'])
            continue
        o = {f: (None if v[f] is None else int(round(v[f] * 1000))) for f in FAST}
        o.update(tgt_rpm=round(v['tgt_rpm'], 1), frac=round(v['frac'], 4), worst_92ms=v['worst'].get('0.092'),
                 ss_std_rpm=round(v['ss_std_rpm'], 2), meas_std_rpm=round(v['meas_std_rpm'], 2),
                 abs_meas_err_touch_rpm=round(v['abs_meas_err_touch_rpm'], 1))
        out[k] = o
    return out


per_ball = {}
for k, v in S['burst'].items():
    p = k.split('|')
    if p[1] == 'asbuilt' and p[3] == '0.45' and p[4] == '1.0' and not v.get('infeasible'):
        per_ball[k] = dict(mean=v['per_ball_mean'], min=v['per_ball_min'], max=v['per_ball_max'])

res = dict(
    key_format='shooter|plant|distance_in|exit_fraction|drag_factor|controller|measurement|octoquad_interval_s',
    controllers=dict(oracle='ideal: true speed at 5 kHz, full power below target else exact FF',
                     team='Shooter.java: FF + P(+I held while pinned), voltage every 50 loops, <=13 V, command lands 8-12 ms after compute',
                     rec='full power when >2 % low, else FF + P + I within 75 rpm, fresh voltage, full duty, command lands 1.5-3 ms after compute',
                     rect='as rec with the full-power threshold tuned (2-10 %)'),
    plants={sh: R.plants(sh) for sh in ('pollen', 'nectar')},
    motor=dict(R_ohm=F.RM, kt=F.KT_STALL, ke=F.KE, free_rpm_tested=F.W_FREE_RPM,
               loaded_free_rpm={V: F.free_speed(V, 0.1, 0.008) * F.RPM for V in (12.0, 12.5, 13.5)}),
    inertia=dict(design=C.J_DESIGN, rotor_est=C.J_ROTOR, roller_est=C.J_ROLLER, asbuilt=C.J_ASBUILT, steel82_vendor=C.J_STEEL82,
                 steel82_old_listing=C.J_STEEL82_OLD, wheel96_est=C.J_WHEEL96),
    shots={sh: C.SHOTS[sh] for sh in C.SHOTS}, shots_calibrated_angles={sh: C.SHOTS_CAL[sh] for sh in C.SHOTS_CAL},
    windows_ref=C.WIN_REF, required=S['required'], tune=S['tune'], single=S.get('single'),
    burst=slim(S['burst']), changes=slim(S.get('changes', {})), oq_sweep=slim(S.get('oq_sweep', {})),
    per_ball_asbuilt_f045_drag1=per_ball, energy=S.get('energy'), mass=S.get('mass'),
    sens=json.load(open(sens)) if os.path.exists(sens) else None,
    mass_budgeted=json.load(open(os.path.join(R.OUT, 'budget.json'))),
    budget_txt=open(os.path.join(HERE, 'budget.txt')).read(),
    headline=open(os.path.join(HERE, 'headline.txt')).read() if os.path.exists(os.path.join(HERE, 'headline.txt')) else None,
)
json.dump(res, open(os.path.join(HERE, 'results_cad.json'), 'w'), indent=1, default=lambda o: None)
print('results_cad.json written', os.path.getsize(os.path.join(HERE, 'results_cad.json')) // 1024, 'kB')

B = S['burst']
print('\nHood-optimistic line, as-built, f=0.45: fastest 10-ball interval ms, reference window -> reference+hood (drag x1.00 ; x1.14)')
for sh in ('pollen', 'nectar'):
    for cm in (('oracle', 'none', 0.02), ('team', 'hub', 0.02), ('rec', 'hub', 0.02), ('rect', 'oq', 0.02)):
        cells = []
        for d in DS:
            c = []
            for drag in C.DRAG:
                v = B.get('|'.join(str(x) for x in (sh, 'asbuilt', d, 0.45, drag, *cm)))
                if v is None or v.get('infeasible'):
                    c.append('inf'); continue
                f = lambda x: '>300' if x is None else str(int(round(x * 1000)))
                c.append(f"{f(v['fast_v_10'])}->{f(v['fast_vh_10'])}")
            cells.append(f'{d}: ' + ' ; '.join(c))
        print(f'{sh:6s} {cm[0]}/{cm[1]:4s} | ' + ' | '.join(cells))

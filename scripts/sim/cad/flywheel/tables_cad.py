"""Builds results_tables.txt and results/summary.json from the stage outputs."""
import json, math, os, sys
from collections import defaultdict
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import cadcfg as C
import ballistics as B
import run_cad as R

RES = os.path.join(HERE, 'results')
L = []
P = lambda *a: L.append(' '.join(str(x) for x in a))
INTS = R.INTERVALS
DS = [36, 60, 84, 108, 132]
CM_LABEL = {('oracle', 'none', 0.02): 'ideal', ('team', 'hub', 0.02): 'team/hub', ('team', 'oq', 0.02): 'team/OQ20',
            ('rec', 'hub', 0.02): 'rec/hub', ('rec', 'oq', 0.02): 'rec/OQ20', ('rect', 'hub', 0.02): 'rect/hub',
            ('rect', 'oq', 0.02): 'rect/OQ20', ('team', 'oq', 0.01): 'team/OQ10', ('team', 'oq', 0.05): 'team/OQ50',
            ('rec', 'oq', 0.01): 'rec/OQ10', ('rec', 'oq', 0.05): 'rec/OQ50', ('rect', 'oq', 0.01): 'rect/OQ10',
            ('rect', 'oq', 0.05): 'rect/OQ50'}
WIN_C = {sh: {d: (C.SHOTS[sh][d]['win_lo_pct'], C.SHOTS[sh][d]['win_hi_pct']) for d in DS} for sh in ('pollen', 'nectar')}


def load(name):
    p = os.path.join(RES, name + '.json')
    return json.load(open(p)) if os.path.exists(p) else None


def group(rows):
    G = defaultdict(list)
    for r in rows:
        G[(r['sh'], r['pk'], r['d'], r['f'], r['drag'], r['ctrl'], r['meas'], r['Nq'])].append(r)
    return G


def devs(rs):
    ref = np.mean([r['w_exit'][0] for r in rs])
    by = defaultdict(list)
    for r in rs:
        by[r['spacing']].append(np.array(r['w_exit']) / ref * 100 - 100)
    return ref, {k: np.array(v) for k, v in by.items()}


def passes(D, lo, hi, nb=10, absorb_pct=0.0):
    x = D[:, :nb]
    if absorb_pct > 0:
        x = np.where(x < 0, np.minimum(0.0, x + absorb_pct), x)
    return bool(np.all(np.isfinite(x)) and np.all(x >= lo) and np.all(x <= hi))


def fastest(by, lo, hi, nb=10, absorb_pct=0.0):
    ok = {sp: passes(by[sp], lo, hi, nb, absorb_pct) for sp in by}
    best = None
    for sp in sorted(ok, reverse=True):
        if ok[sp]:
            best = sp
        else:
            break
    return best


def fmt_iv(x):
    return '>300' if x is None else f'{int(round(x*1000))}'


def analyse_bursts(rows, skipped):
    G = group(rows)
    out = {}
    for key, rs in G.items():
        sh, pk, d, f, drag, ctrl, meas, Nq = key
        ref, by = devs(rs)
        tgt = rs[0]['tgt_rpm']
        absorb = C.hood_absorb_rpm(d) / tgt * 100
        lv, hv = C.WIN_REF[d]
        lc, hc = WIN_C[sh][d]
        rec = dict(tgt_rpm=tgt, frac=rs[0]['frac'], ref_exit_rpm=ref)
        for nb in (3, 5, 10):
            rec[f'fast_v_{nb}'] = fastest(by, lv, hv, nb)
            rec[f'fast_c_{nb}'] = fastest(by, lc, hc, nb)
            rec[f'fast_vh_{nb}'] = fastest(by, lv, hv, nb, absorb)
            rec[f'fast_ch_{nb}'] = fastest(by, lc, hc, nb, absorb)
        rec['worst'] = {f'{sp:.3f}': [float(np.nanmin(by[sp])), float(np.nanmax(by[sp]))] for sp in by}
        rec['per_ball_mean'] = {f'{sp:.3f}': [float(x) for x in np.nanmean(by[sp], 0)] for sp in by}
        rec['per_ball_min'] = {f'{sp:.3f}': [float(x) for x in np.nanmin(by[sp], 0)] for sp in by}
        rec['per_ball_max'] = {f'{sp:.3f}': [float(x) for x in np.nanmax(by[sp], 0)] for sp in by}
        rec['ss_std_rpm'] = float(np.mean([r['ss_std_rpm'] for r in rs]))
        rec['meas_std_rpm'] = float(np.mean([r['meas_std_rpm'] for r in rs]))
        rec['abs_meas_err_touch_rpm'] = float(np.nanmean([abs(x) for r in rs for x in r['wm_err_touch'][1:]]))
        out['|'.join(str(k) for k in key)] = rec
    for s in skipped:
        key = (s['sh'], s['pk'], s['d'], s['f'], s['drag'], s['ctrl'], s['meas'], s['Nq'])
        k = '|'.join(str(x) for x in key)
        if k not in out:
            out[k] = dict(infeasible=True, frac=s['frac'])
    return out


def K(sh, pk, d, f, drag, cm):
    return '|'.join(str(x) for x in (sh, pk, d, f, drag, cm[0], cm[1], cm[2]))


def cell(A, k, field):
    r = A.get(k)
    if r is None:
        return '  - '
    if r.get('infeasible'):
        return ' inf'
    return f"{fmt_iv(r[field]):>4s}"


def main():
    summary = {}
    sing = load('single'); bur = load('burst'); oqs = load('oqsweep'); chg = load('changes'); mass = load('mass')
    tune = load('tune')
    energy = load('energy')

    P('FLYWHEEL RAPID-FIRE STUDY, CAD NUMBERS. All speeds are flywheel rpm unless marked; dev = exit-speed deviation from the calibrated nominal (mean ball-1 exit), %.')
    P('')
    P('== INPUTS / DERIVED')
    import flysim_cad as F
    P(f'motor 5000-0002-4008 (goBILDA page): 12 V, 5800 rpm tested (6000 theor.), 0.25 A free, 9.2 A stall, 1.47 kg.cm -> R {F.RM:.3f} ohm, kt {F.KT_STALL:.5f} N.m/A, ke {F.KE:.5f} V.s/rad')
    P(f"J as-built: pollen {C.J_ASBUILT['pollen']:.3e} (design {C.J_DESIGN:.2e} + rotor {C.J_ROTOR:.0e} + roller {C.J_ROLLER:.2e}), nectar {C.J_ASBUILT['nectar']:.3e} (no roller)")
    P(f'loaded free speed @12.5 V/0.1 ohm, friction 0.008 N.m@3000: {F.free_speed(12.5,0.1,0.008)*F.RPM:.0f} rpm (12.0 V {F.free_speed(12.0,0.1,0.008)*F.RPM:.0f}, 13.5 V {F.free_speed(13.5,0.1,0.008)*F.RPM:.0f})')
    P('')
    P('Shots (drag-free, CAD exit points; nominal = min-speed angle or the hood\'s lowest angle when that is steeper)')
    P('shooter d  | hood m  launch  vmin-ang pinned | exit z  v in/s (+% vs vmin) | TOF ms descent | own window % | reference window %')
    for sh in ('pollen', 'nectar'):
        for d in DS:
            s = C.SHOTS[sh][d]
            P(f"{sh:6s} {d:3d} | {s['m']:5.2f} {s['launch_deg']:6.2f} {s['vmin_angle_deg']:6.2f} {'Y' if s['pinned'] else 'n'} | {s['exit_z']:5.2f} {s['v_in_s']:6.1f} (+{s['extra_vs_vmin_pct']:.2f}) | {1000*s['tof_s']:4.0f} {s['descent_deg']:5.1f} | {s['win_lo_pct']:+.2f}/{s['win_hi_pct']:+.2f} | {C.WIN_REF[d][0]:+.2f}/{C.WIN_REF[d][1]:+.2f}")
    P('Sim-calibrated launch angles (Pollen 55.9+m, Nectar 57.4+m) change v by: ' + '; '.join(
        f"{sh} " + ' '.join(f"{d}:{100*(C.SHOTS_CAL[sh][d]['v_in_s']/C.SHOTS[sh][d]['v_in_s']-1):+.2f}%" for d in DS) for sh in ('pollen', 'nectar')))
    P('')
    P('Required flywheel target rpm (as-built J, 72 mm, 1:1) and % of loaded free speed (5533 @12.5 V). Cells: drag x1.00 / x1.14')
    P('shooter f    | ' + ' | '.join(f'{d:>4d} in' for d in DS) + ' | rho=w_exit/w_tgt  Tc ms (36..132)')
    req = {}
    for sh in ('pollen', 'nectar'):
        for f in C.F_EXIT:
            cells = []
            for d in DS:
                a = R.tgt_info(sh, d, f, 1.0, 'asbuilt'); b = R.tgt_info(sh, d, f, 1.14, 'asbuilt')
                req[f'{sh}|{d}|{f}'] = dict(rpm_drag0=a['tgt'] * F.RPM, frac0=a['frac'], rpm_drag14=b['tgt'] * F.RPM, frac14=b['frac'],
                                           rho=a['rho'], Tc_ms=1000 * a['Tc'], w_exit_rpm=a['w_exit_rpm'])
                cells.append(f"{a['tgt']*F.RPM:4.0f}({100*a['frac']:3.0f}%)/{b['tgt']*F.RPM:4.0f}({100*b['frac']:3.0f}%)")
            t0 = R.tgt_info(sh, 36, f, 1.0, 'asbuilt'); t1 = R.tgt_info(sh, 132, f, 1.0, 'asbuilt')
            P(f"{sh:6s} {f:.2f} | " + ' | '.join(cells) + f" | {t0['rho']:.3f}  {1000*t0['Tc']:.1f}..{1000*t1['Tc']:.1f}")
    summary['required'] = req
    P('(>=99.5% of free speed = infeasible: not simulated)')
    P('')
    P('Tuned gains (cost: mean over 6 seeds x {150,250} ms of max |dev| over 5 balls, pollen/nectar at 84 in, f .45). kP V/rpm, kI V/(rpm s); team code units = /13')
    for sh in ('pollen', 'nectar'):
        for pk in R.BASE_PLANTS:
            P(f"  {sh:6s} {pk:8s} " + '  '.join(f"{c}/{m}{'' if q == 0.02 else int(q*1000)}: {tune[f'{sh}|{pk}|{c}|{m}|{q}']['gains']}" for c in ('team', 'rec', 'rect') for m, q in (('hub', 0.02), ('oq', 0.02))))
    summary['tune'] = tune

    if sing:
        P('')
        P('== ONE BALL from a steady wheel: dip (rpm, % of target) and time to stay within 1/2/3 % (ms; median over 8 seeds, [worst]); f=0.45, drag x1.00')
        G = group(sing['rows'])
        one_ball = {}
        for key, rs in G.items():
            dip = np.mean([r['dip1_rpm'] for r in rs])
            tg = rs[0]['tgt_rpm']
            d = dict(dip_rpm=float(dip), dip_pct=float(100 * dip / tg))
            for b in (1, 2, 3):
                v = np.array([r[f'rec_last_{b}'] for r in rs])
                d[f'rec{b}_med_ms'] = float(np.nanmedian(v) * 1000) if np.isfinite(v).any() else None
                d[f'rec{b}_max_ms'] = float(np.nanmax(v) * 1000) if np.all(np.isfinite(v)) else None
            one_ball['|'.join(str(x) for x in key)] = d
        summary['single'] = one_ball
        for sh in ('pollen', 'nectar'):
            P(f'-- {sh}')
            P('J        ctrl              | ' + ' | '.join(f'{d:>3d} in: dip   1%/2%/3% ms' for d in DS))
            for pk in R.BASE_PLANTS:
                for cm in R.CTRLM:
                    cells = []
                    for d in DS:
                        k = '|'.join(str(x) for x in (sh, pk, d, 0.45, 1.0, cm[0], cm[1], cm[2]))
                        r = one_ball.get(k)
                        if r is None:
                            cells.append(f'{"infeasible":>26s}')
                            continue
                        rr = lambda b: ('nev' if r[f'rec{b}_med_ms'] is None else f"{r[f'rec{b}_med_ms']:.0f}") + ('' if r[f'rec{b}_max_ms'] is not None else '*')
                        cells.append(f"{r['dip_rpm']:5.0f}({r['dip_pct']:4.1f}%) {rr(1)}/{rr(2)}/{rr(3)}".rjust(26))
                    P(f'{pk:8s} {CM_LABEL[cm]:17s} | ' + ' | '.join(cells))
        P('(* = at least one seed never settled inside that band; nev = median never)')

    if bur:
        A = analyse_bursts(bur['rows'], bur['skipped'])
        summary['burst'] = A
        P('')
        P('== FASTEST FEED INTERVAL (ms) at which every ball of a 10-ball burst (all 8 seeds) stays inside the window,')
        P('   requiring every longer interval tested to pass too. Intervals tested 92/105/120/135/150/175/200/250/300. >300 = none. inf = target beyond free speed.')
        P('   Cells: reference window | own window | reference + optimistic hood | own + hood')
        for f in C.F_EXIT:
            for drag in C.DRAG:
                P(f'-- f={f}, drag x{drag:.2f}')
                for sh in ('pollen', 'nectar'):
                    P(f'{sh:6s} J        ctrl              | ' + ' | '.join(f'{d:>3d} in: V   C   Vh  Ch' for d in DS))
                    for pk in R.BASE_PLANTS:
                        for cm in R.CTRLM:
                            cells = []
                            for d in DS:
                                k = K(sh, pk, d, f, drag, cm)
                                cells.append(' '.join(cell(A, k, fl) for fl in ('fast_v_10', 'fast_c_10', 'fast_vh_10', 'fast_ch_10')).rjust(23))
                            P(f'       {pk:8s} {CM_LABEL[cm]:17s} | ' + ' | '.join(cells))
        P('')
        P('3- and 5-ball bursts (reference window), f=0.45 drag x1.00: cells 3-ball/5-ball/10-ball fastest interval')
        for sh in ('pollen', 'nectar'):
            for pk in R.BASE_PLANTS:
                for cm in R.CTRLM:
                    cells = []
                    for d in DS:
                        k = K(sh, pk, d, 0.45, 1.0, cm)
                        cells.append('/'.join(cell(A, k, f'fast_v_{nb}').strip() for nb in (3, 5, 10)).rjust(14))
                    P(f'{sh:6s} {pk:8s} {CM_LABEL[cm]:17s} | ' + ' | '.join(cells))
        P('')
        P('Per-ball exit deviation (%), mean over seeds [worst seed], as-built J, f=0.45, drag x1.00')
        for sh in ('pollen', 'nectar'):
            for d in (36, 84, 132):
                for cm in (('oracle', 'none', 0.02), ('team', 'hub', 0.02), ('rec', 'oq', 0.02), ('rect', 'oq', 0.02)):
                    k = K(sh, 'asbuilt', d, 0.45, 1.0, cm)
                    r = A.get(k)
                    if r is None or r.get('infeasible'):
                        P(f'{sh} {d} {CM_LABEL[cm]}: infeasible'); continue
                    for sp in (0.092, 0.150, 0.250):
                        mb = r['per_ball_mean'][f'{sp:.3f}']; mn = r['per_ball_min'][f'{sp:.3f}']; mx = r['per_ball_max'][f'{sp:.3f}']
                        worst = [a if abs(a) > abs(b) else b for a, b in zip(mn, mx)]
                        P(f'{sh:6s} {d:3d} {CM_LABEL[cm]:17s} {int(sp*1000):3d} ms: ' + ' '.join(f'{a:+5.2f}[{w:+5.2f}]' for a, w in zip(mb, worst)))
        P('')
        P('Worst-case (over balls and seeds) deviation at 92 ms, as-built J; f=0.45; cells per distance: drag x1.00, x1.14')
        for sh in ('pollen', 'nectar'):
            for cm in R.CTRLM:
                cells = []
                for d in DS:
                    for drag in C.DRAG:
                        r = A.get(K(sh, 'asbuilt', d, 0.45, drag, cm))
                        cells.append('inf' if (r is None or r.get('infeasible')) else f"{r['worst']['0.092'][0]:+.1f}/{r['worst']['0.092'][1]:+.1f}")
                P(f'{sh:6s} {CM_LABEL[cm]:17s} | ' + ' | '.join(f'{d}:{cells[2*i]},{cells[2*i+1]}' for i, d in enumerate(DS)))

    if energy:
        P('')
        P('== ENERGY per ball drawn from the wheel (J), net motor power above friction at target, full duty, Rint 0.1 (W),')
        P('   and the sustained rate it supports with no margin (balls/s) at 12.0/12.5/13.5 V; team loop (13 V cap) in brackets at 13.5 V')
        for sh in ('pollen', 'nectar'):
            for pk in ('asbuilt', 'J2.4', '2mot', 'w96', 'g1.25', 'g1.5'):
                for f in C.F_EXIT:
                    for drag in C.DRAG:
                        rs = [e for e in energy if e['sh'] == sh and e['pk'] == pk and e['f'] == f and e['drag'] == drag]
                        rs.sort(key=lambda e: e['d'])
                        cells = []
                        for e in rs:
                            if not e['feasible']:
                                cells.append(f"{e['d']}: infeasible ({100*e['frac']:.0f}%)")
                            else:
                                cells.append(f"{e['d']}: {e['E_ball_J']:.2f}J {e['P_12.0']:.0f}/{e['P_12.5']:.0f}/{e['P_13.5']:.0f}W {e['rate_12.0']:.1f}/{e['rate_12.5']:.1f}/{e['rate_13.5']:.1f}[{e['rate13cap_13.5']:.1f}]/s")
                        P(f'{sh:6s} {pk:7s} f={f:.2f} x{drag:.2f} | ' + ' | '.join(cells))
        summary['energy'] = energy
        e = [x for x in energy if x['feasible']]
        P(f"E_wheel / ball KE (incl. spin) ratio: {min(x['ratio'] for x in e):.2f}..{max(x['ratio'] for x in e):.2f} (1/f in the J>>k limit)")

    if mass:
        P('')
        P('== BALL-MASS SPREAD: exit-speed change (%) for the lightest / heaviest ball vs nominal, same target (FF held through contact; ideal loop in brackets)')
        M = defaultdict(dict)
        for r in mass['rows']:
            M[(r['sh'], r['pk'], r['d'], r['f'], r['ctrl'])][round(r['m'], 4)] = r['w_exit'][0]
        mass_spread = {}
        for sh in ('pollen', 'nectar'):
            b = C.BALL[sh]
            P(f"-- {sh} m = {b['m_lo']*1000:.1f} / {b['m']*1000:.1f} / {b['m_hi']*1000:.1f} g")
            for pk in R.BASE_PLANTS + ['steel82', '2mot', 'w96', 'g1.25', 'g1.5']:
                cells = []
                for d in (36, 84, 132):
                    for f in C.F_EXIT:
                        v = M.get((sh, pk, d, f, 'ff')); o = M.get((sh, pk, d, f, 'oracle'))
                        if not v or len(v) < 3:
                            cells.append(f'{d}/{f}: inf'); continue
                        nom = v[round(b['m'], 4)]
                        lo = 100 * (v[round(b['m_lo'], 4)] / nom - 1); hi = 100 * (v[round(b['m_hi'], 4)] / nom - 1)
                        olo = 100 * (o[round(b['m_lo'], 4)] / o[round(b['m'], 4)] - 1); ohi = 100 * (o[round(b['m_hi'], 4)] / o[round(b['m'], 4)] - 1)
                        mass_spread[f'{sh}|{pk}|{d}|{f}'] = dict(lo=lo, hi=hi, ideal_lo=olo, ideal_hi=ohi)
                        cells.append(f'{d}/{f}: {lo:+.2f}/{hi:+.2f} [{olo:+.2f}/{ohi:+.2f}]')
                P(f'{pk:13s} ' + ' | '.join(cells))
        summary['mass'] = mass_spread

    if bur:
        allrows = bur['rows'] + (oqs['rows'] if oqs else [])
        A5 = analyse_bursts(allrows, bur['skipped'] + (oqs['skipped'] if oqs else []))
        summary['oq_sweep'] = {k: v for k, v in A5.items() if '|oq|0.01' in k or '|oq|0.05' in k}
        P('')
        P('== MEASUREMENT: hub port (50 ms window, +-0.5 ms stamps) vs OctoQuad latch 10/20/50 ms; f=0.45 drag x1.00')
        P('   cells: fastest 10-ball interval (reference window) ; true steady ripple std rpm ; measured std rpm ; |measured-true| at touches 2..10 rpm')
        cms5 = [('team', 'hub', 0.02), ('team', 'oq', 0.01), ('team', 'oq', 0.02), ('team', 'oq', 0.05), ('rec', 'hub', 0.02), ('rec', 'oq', 0.01),
                ('rec', 'oq', 0.02), ('rec', 'oq', 0.05), ('rect', 'hub', 0.02), ('rect', 'oq', 0.01), ('rect', 'oq', 0.02), ('rect', 'oq', 0.05)]
        for sh in ('pollen', 'nectar'):
            for pk in ('asbuilt', 'J1.6', 'J2.4', 'J4.0'):
                for cm in cms5:
                    cells = []
                    for d in DS:
                        r = A5.get(K(sh, pk, d, 0.45, 1.0, cm))
                        if r is None or r.get('infeasible'):
                            cells.append('inf'.rjust(20)); continue
                        cells.append(f"{fmt_iv(r['fast_v_10']):>4s};{r['ss_std_rpm']:5.1f};{r['meas_std_rpm']:5.1f};{r['abs_meas_err_touch_rpm']:5.0f}")
                    P(f'{sh:6s} {pk:7s} {CM_LABEL[cm]:17s} | ' + ' | '.join(cells))
        ov = [(d, 32767 / (8192 * 0.05) * 60) for d in DS]
        P(f'OctoQuad int16 counts-per-interval limit (counts/interval assumption): 50 ms overflows above {32767/(8192*0.05)*60:.0f} rpm, 20 ms above {32767/(8192*0.02)*60:.0f} rpm, 10 ms above {32767/(8192*0.01)*60:.0f}')

    if chg and bur:
        A6 = analyse_bursts(chg['rows'] + bur['rows'], chg['skipped'] + bur['skipped'])
        summary['changes'] = {k: v for k, v in A6.items() if k.split('|')[1] in R.CHANGE_PLANTS}
        P('')
        P('== WHAT CHANGES THE ANSWER: fastest 10-ball interval (ms), reference window; cells per distance; inf = beyond free speed')
        cms6 = [('oracle', 'none', 0.02), ('team', 'hub', 0.02), ('rec', 'oq', 0.02), ('rect', 'oq', 0.02), ('rect', 'hub', 0.02)]
        for f in C.F_EXIT:
            for drag in C.DRAG:
                P(f'-- f={f}, drag x{drag:.2f}')
                for sh in ('pollen', 'nectar'):
                    for pk in ['asbuilt', 'J2.4', 'J4.0'] + R.CHANGE_PLANTS:
                        pl = R.plants(sh)[pk]
                        tg = [R.tgt_info(sh, d, f, drag, pk) for d in DS]
                        cells = []
                        for cm in cms6:
                            cells.append(' '.join(cell(A6, K(sh, pk, d, f, drag, cm), 'fast_v_10') for d in DS))
                        P(f"{sh:6s} {pk:13s} J={pl['J']*1e4:4.2f}e-4 n={pl['nmot']} g={pl['g']} r={pl['r']*1000:.0f} tgt%free {'/'.join(f'{100*t['frac']:.0f}' for t in tg)} | " +
                          ' || '.join(f'{CM_LABEL[cm]}: {c}' for cm, c in zip(cms6, cells)))
    json.dump(summary, open(os.path.join(RES, 'summary.json'), 'w'), indent=1, default=lambda o: None)
    open(os.path.join(HERE, 'results_tables.txt'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()

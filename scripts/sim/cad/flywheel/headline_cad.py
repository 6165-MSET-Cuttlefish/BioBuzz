"""Headline: fastest 10-ball feed interval per shooter/distance, and the cheapest change that reaches 92 ms."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadcfg as C
import run_cad as R

S = json.load(open(os.path.join(R.OUT, 'summary.json')))
A = dict(S['burst']); A.update(S.get('changes', {}))
DS = [36, 60, 84, 108, 132]
REAL = [('team', 'hub', 0.02), ('team', 'oq', 0.02), ('rec', 'hub', 0.02), ('rec', 'oq', 0.02), ('rect', 'hub', 0.02), ('rect', 'oq', 0.02)]
CH_REAL = [('team', 'hub', 0.02), ('rec', 'oq', 0.02), ('rect', 'oq', 0.02), ('rect', 'hub', 0.02)]
LADDER = ['asbuilt', 'g1.25', 'g1.5', 'steel82', 'g1.25_steel82', 'g1.5_steel82', 'J4.0', '2mot', '2mot_steel82', 'w96']


def k(sh, pk, d, f, drag, cm):
    return '|'.join(str(x) for x in (sh, pk, d, f, drag, cm[0], cm[1], cm[2]))


def fast(sh, pk, d, f, drag, cm, fld='fast_v_10'):
    r = A.get(k(sh, pk, d, f, drag, cm))
    if r is None:
        return 'n/a'
    if r.get('infeasible'):
        return 'inf'
    v = r.get(fld)
    return '>300' if v is None else int(round(v * 1000))


def best_real(sh, pk, d, f, drag, cms, fld='fast_v_10'):
    vals = []
    for cm in cms:
        v = fast(sh, pk, d, f, drag, cm, fld)
        if isinstance(v, int):
            vals.append((v, cm))
    if not vals:
        return fast(sh, pk, d, f, drag, cms[0], fld), None
    return min(vals)


L = []
P = L.append
P('== CAN IT FIRE WITH NO WAITING? fastest 10-ball interval (ms) inside the reference window, as-built. 92 ms = transfer limit (10.9 balls/s)')
P('   cells: ideal / team loop on hub port / best real loop (which) ; f = exit-speed fraction; drag = required speed x1.00 or x1.14')
for sh in ('pollen', 'nectar'):
    for f in C.F_EXIT:
        for drag in C.DRAG:
            cells = []
            for d in DS:
                i = fast(sh, 'asbuilt', d, f, drag, ('oracle', 'none', 0.02))
                t = fast(sh, 'asbuilt', d, f, drag, ('team', 'hub', 0.02))
                b, cm = best_real(sh, 'asbuilt', d, f, drag, REAL)
                lab = '' if cm is None else f"{cm[0]}/{cm[1]}"
                cells.append(f'{d}: {i}/{t}/{b} {lab}')
            P(f'{sh:6s} f={f:.2f} x{drag:.2f} | ' + ' | '.join(cells))
P('')
P('== CHEAPEST CHANGE THAT REACHES 92 ms with a real loop (team/hub, rec/OQ20, rect/OQ20 or rect/hub), reference window;')
P('   ladder: ' + ' < '.join(LADDER) + '  (J4.0 = ~2 steel flywheels; g = wheel/motor speed ratio)')
for sh in ('pollen', 'nectar'):
    for f in C.F_EXIT:
        for drag in C.DRAG:
            cells = []
            for d in DS:
                found = None
                for pk in LADDER:
                    cms = REAL if pk in R.BASE_PLANTS else CH_REAL
                    v, cm = best_real(sh, pk, d, f, drag, cms)
                    if v == 92:
                        found = f'{pk} ({cm[0]}/{cm[1]})'
                        break
                if found is None:
                    bests = []
                    for pk in LADDER:
                        cms = REAL if pk in R.BASE_PLANTS else CH_REAL
                        v, cm = best_real(sh, pk, d, f, drag, cms)
                        if isinstance(v, int):
                            bests.append((v, pk))
                    found = 'none; best ' + (f'{min(bests)[0]} ms ({min(bests)[1]})' if bests else 'infeasible')
                cells.append(f'{d}: {found}')
            P(f'{sh:6s} f={f:.2f} x{drag:.2f} | ' + ' | '.join(cells))
P('')
P('== PER CHANGE: best real loop, fastest 10-ball interval (ms), reference window, f=0.45; cells 36/60/84/108/132 in, drag x1.00 ; x1.14')
for sh in ('pollen', 'nectar'):
    for pk in LADDER + ['J1.2', 'J1.6', 'J2.4', 'g0.8']:
        cms = REAL if pk in R.BASE_PLANTS else CH_REAL
        a = [str(best_real(sh, pk, d, 0.45, 1.0, cms)[0]) for d in DS]
        b = [str(best_real(sh, pk, d, 0.45, 1.14, cms)[0]) for d in DS]
        ia = [str(fast(sh, pk, d, 0.45, 1.0, ('oracle', 'none', 0.02))) for d in DS]
        fr = [f"{100*R.tgt_info(sh, d, 0.45, 1.0, pk)['frac']:.0f}" for d in DS]
        pl = R.plants(sh)[pk]
        P(f"{sh:6s} {pk:13s} J {pl['J']*1e4:.2f}e-4 n{pl['nmot']} g{pl['g']} r{pl['r']*1000:.0f} tgt%free {'/'.join(fr)} | real {'/'.join(a)} ; {'/'.join(b)} | ideal {'/'.join(ia)}")
out = '\n'.join(L)
open(os.path.join(HERE, 'headline.txt'), 'w').write(out + '\n')
print(out)

"""Compares the reruns in results/ (rerun.py orig, seeds, f040, ktke, dt, heavy) with ../results/summary.json."""
import json, os, sys
from collections import Counter
HERE = os.path.dirname(os.path.abspath(__file__))
S = json.load(open(os.path.join(HERE, '..', 'results', 'summary.json')))
SUMMARY = dict(S['burst']); SUMMARY.update(S.get('changes', {}))
INT = [92, 105, 120, 135, 150, 175, 200, 250, 300]
step = lambda v: 9 if v is None else INT.index(v)
fmt = lambda v: '>300' if v is None else str(v)
L = []
P = L.append


def load(e):
    p = os.path.join(HERE, 'results', f'rerun_{e}.json')
    return json.load(open(p)) if os.path.exists(p) else None


def summary_fast(k):
    r = SUMMARY.get(k)
    if r is None or r.get('infeasible'):
        return 'inf' if r else 'n/a'
    v = r['fast_v_10']
    return None if v is None else int(round(v * 1000))

orig = load('orig')
mism = [(k, v['anc10'], summary_fast(k)) for k, v in orig.items() if k != '_meta' and not v.get('infeasible') and v['anc10'] != summary_fast(k)]
P(f'orig rerun vs summary.json (rule anc, 10 balls): {len(orig)-1} cells, {len(mism)} mismatches {mism[:5]}')

seeds = load('seeds')
for rule in ('anc10', 'bias10', 'anc3'):
    c = Counter()
    worst = []
    for k, v in seeds.items():
        if k == '_meta' or v.get('infeasible'):
            continue
        o = orig[k][rule]
        d = step(v[rule]) - step(o)
        c[d] += 1
        if abs(d) >= 2:
            worst.append((k, fmt(o), fmt(v[rule])))
    P(f'seeds 901-908 vs 301-308, rule {rule}: step change distribution {dict(sorted(c.items()))}; |change|>=2 steps: {worst[:8]}')

P('\nseed check, headline rows (f .45): cells 36/60/84/108/132, original seeds -> new seeds, rule anc10 ; bias10')
for sh in ('pollen', 'nectar'):
    for pk in ('asbuilt', 'steel82', '2mot_steel82'):
        for dr in (1.0, 1.14):
            for cm in ('oracle|none|0.02', 'team|hub|0.02', 'rect|oq|0.02', 'rect|hub|0.02', 'rec|hub|0.02'):
                ks = [f'{sh}|{pk}|{d}|0.45|{dr}|{cm}' for d in (36, 60, 84, 108, 132)]
                a = lambda R, r: '/'.join('inf' if R[k].get('infeasible') else fmt(R[k][r]) for k in ks)
                P(f'{sh:6s} {pk:12s} x{dr:.2f} {cm:15s} | anc {a(orig,"anc10")} -> {a(seeds,"anc10")} | bias {a(orig,"bias10")} -> {a(seeds,"bias10")}')

for e, f_ in (('f040', '0.4'), ('ktke', '0.45')):
    R_ = load(e)
    P(f'\n{e} (rule anc10 ; bias10), cells 36..132')
    for sh in ('pollen', 'nectar'):
        for pk in (('asbuilt', 'steel82') if e == 'f040' else ('asbuilt',)):
            for dr in (1.0, 1.14):
                for cm in ('oracle|none|0.02', 'team|hub|0.02', 'rect|oq|0.02', 'rec|hub|0.02'):
                    ks = [f'{sh}|{pk}|{d}|{f_}|{dr}|{cm}' for d in (36, 60, 84, 108, 132)]
                    a = lambda r: '/'.join('inf' if R_[k].get('infeasible') else fmt(R_[k][r]) for k in ks)
                    P(f'{sh:6s} {pk:8s} x{dr:.2f} {cm:15s} | anc {a("anc10")} | bias {a("bias10")} | tgt rpm {"/".join("inf" if R_[k].get("infeasible") else str(round(R_[k]["tgt_rpm"])) for k in ks)}')

for e in ('dt', 'heavy'):
    R = load(e)
    P(f'\n{e}: {R["_meta"]}')
    for sh in ('pollen', 'nectar'):
        for cm in ('oracle|none|0.02', 'team|hub|0.02', 'rec|hub|0.02', 'rect|hub|0.02', 'rect|oq|0.02'):
            ks = [f'{sh}|asbuilt|{d}|0.45|1.0|{cm}' for d in (36, 60, 84, 108, 132)]
            P(f'{sh:6s} {cm:15s} | anc orig {"/".join(fmt(orig[k]["anc10"]) for k in ks):24s} -> {e} {"/".join(fmt(R[k]["anc10"]) for k in ks):24s} | '
              f'bias orig {"/".join(fmt(orig[k]["bias10"]) for k in ks)} -> {"/".join(fmt(R[k]["bias10"]) for k in ks)} | worst92 {R[ks[2]]["worst92"]} vs {orig[ks[2]]["worst92"]}')

txt = '\n'.join(L)
open(os.path.join(HERE, 'results', 'compare.txt'), 'w').write(txt + '\n')
print(txt)

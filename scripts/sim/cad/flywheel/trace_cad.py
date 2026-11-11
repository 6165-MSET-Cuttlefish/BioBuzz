import sys, os
os.environ['FLY_TRACE']='1'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import run_cad as R, flysim_cad as F
ctrl, meas, kP, kI, sp = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
pk = sys.argv[6] if len(sys.argv) > 6 else 'asbuilt'
sh = sys.argv[7] if len(sys.argv) > 7 else 'pollen'
r = R.row(sh, 84, 0.45, 1.0, pk, ctrl, meas, 0.02, sp, 1, dict(kP=kP, kI=kI))
P = {k: np.array([r[k]], float) for k in R.SIMKEYS}; P['f'] = np.array([0.45]); P['seed'] = np.array([1])
res = F.simulate(P, ctrl, 'hub' if meas == 'none' else meas, nb=4, warm=R.WARM, T_end=R.WARM + 3 * sp + 0.3)
tr = res['trace']; tg = res['tgt_rpm'][0]
print('tgt', tg, 'exit', np.round(res['w_exit'][0], 0), 'dev%', np.round((res['w_exit'][0] / res['w_exit'][0][0] - 1) * 100, 2))
for row in tr[::5]:
    if row[0] >= R.WARM - 0.01:
        print(f"{1000*(row[0]-R.WARM):6.0f} ms  w {row[1]-tg:+7.1f}  wm {row[2]-tg:+7.1f}  p {row[3]:.3f}  I {row[4]:+.3f}")

"""Offline threshold calibration (no ROS needed).

Simulates the per-frame detect/miss sequence of a real, static obstacle under the measured burst-dropout
model, and of an obstacle that has truly disappeared, and reports for each threshold:

  ARL   : mean frames between false deletions while the obstacle exists (larger = better)
  P_del : probability that one dropout burst causes a false deletion
  delay : frames from true disappearance to deletion (smaller = better)

for the existence CUSUM (h sweep) and for "delete after N consecutive misses" (N sweep), so both sit on
the same false-deletion vs delay trade-off curve. Usage:

  ros2 run fj_experiment calibrate                       # defaults = measured parameters
  ros2 run fj_experiment calibrate --target-arl 6000     # pick h for ~1 false deletion per 10 min @10 Hz
"""
import argparse
import csv
import math
import os
import random

from .common import ExistenceCusum, GilbertElliott


def run_h0(make_det, decide, n_frames, rng):
    """Count false deletions over n_frames of a real obstacle; detector restarts after each deletion."""
    det = make_det(rng)
    st = decide()
    alarms = 0
    for _ in range(n_frames):
        if st(det.step()):
            alarms += 1
            st = decide()
    return alarms


def run_h1(decide, p_fa, rng, cap=600):
    st = decide()
    for n in range(1, cap + 1):
        if st(rng.random() < p_fa):
            return n
    return cap


def cusum_decider(kw):
    def make():
        c = ExistenceCusum(**kw)

        def step(d):
            c.update(d)
            return c.alarmed
        return step
    return make


def khold_decider(N):
    def make():
        st = {'m': 0}

        def step(d):
            st['m'] = 0 if d else st['m'] + 1
            return st['m'] > N
        return step
    return make


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--p-enter', type=float, default=0.066)
    ap.add_argument('--p-exit', type=float, default=0.154)
    ap.add_argument('--p-fa', type=float, default=0.05)
    ap.add_argument('--pd-good', type=float, default=0.99)
    ap.add_argument('--frames', type=int, default=200000, help='frames simulated for ARL')
    ap.add_argument('--target-arl', type=float, default=6000)
    ap.add_argument('--out', default=os.path.expanduser('~/fj_ws/calibration'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rng = random.Random(0)
    kw = dict(p_fa=a.p_fa, pd_good=a.pd_good, pd_bad=0.0, p_enter=a.p_enter, p_exit=a.p_exit)
    make_det = lambda r: GilbertElliott(a.p_enter, a.p_exit, r)
    bursts = a.frames * a.p_enter * a.p_exit / (a.p_enter + a.p_exit)  # expected number of bursts

    c = ExistenceCusum(**kw, h=1e9)
    one_miss = c.llr(False)
    print(f'\n一次漏检的证据增量 ln f1/f0 = {one_miss:.2f}（s_soft 应大于它，否则单次漏检就会在两档之间来回切换）')

    rows = []
    for h in [2, 3, 4, 5, 6, 7, 8, 9, 10]:
        dec = cusum_decider(dict(kw, h=h))
        al = run_h0(make_det, dec, a.frames, rng)
        delay = sum(run_h1(dec, a.p_fa, rng) for _ in range(300)) / 300
        rows.append(dict(method='CUSUM', param=f'h={h}', arl=a.frames / max(al, 1), p_del=al / bursts,
                         delay=delay))
    for N in [2, 5, 10, 15, 20, 30]:
        dec = khold_decider(N)
        al = run_h0(make_det, dec, a.frames, rng)
        delay = sum(run_h1(dec, a.p_fa, rng) for _ in range(300)) / 300
        rows.append(dict(method='N帧删除', param=f'N={N}', arl=a.frames / max(al, 1), p_del=al / bursts,
                         delay=delay))

    print(f'\n{"方法":8s}{"参数":8s}{"平均误删间隔(帧)":>18s}{"每段漏检误删概率":>18s}{"删除延迟(帧)":>14s}')
    for r in rows:
        print(f'{r["method"]:8s}{r["param"]:8s}{r["arl"]:18.0f}{r["p_del"]:18.3f}{r["delay"]:14.1f}')
    cands = [r for r in rows if r['method'] == 'CUSUM' and r['arl'] >= a.target_arl]
    if cands:
        best = min(cands, key=lambda r: r['delay'])
        print(f'\n满足平均误删间隔 >= {a.target_arl:.0f} 帧的最小延迟阈值：{best["param"]}'
              f'（延迟 {best["delay"]:.1f} 帧 = {best["delay"] / 10:.1f} s @10 Hz）')
    with open(os.path.join(a.out, 'calibration.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 4))
        for name, mk in (('CUSUM', 'o-'), ('N帧删除', 's--')):
            rr = [r for r in rows if r['method'] == name]
            ax.plot([r['delay'] / 10 for r in rr], [r['arl'] / 10 / 60 for r in rr], mk,
                    label='CUSUM' if name == 'CUSUM' else 'N-frame hold')
            for r in rr:
                ax.annotate(r['param'], (r['delay'] / 10, r['arl'] / 600), fontsize=7, xytext=(3, 3),
                            textcoords='offset points')
        ax.set_yscale('log')
        ax.set_xlabel('deletion delay after true disappearance (s)')
        ax.set_ylabel('mean time between false deletions (min)')
        ax.legend()
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, 'calibration.png'), dpi=150)
        print(f'图和表已保存到 {a.out}')
    except ImportError:
        print(f'表已保存到 {a.out}（未安装 matplotlib，跳过画图）')


if __name__ == '__main__':
    main()

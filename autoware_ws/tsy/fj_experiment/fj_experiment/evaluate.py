"""Offline evaluation of the runner logs (no ROS needed).

  ros2 run fj_experiment evaluate --name straight
  -> ~/fj_ws/results/<name>/eval/{per_run.csv, summary.csv, *.png}

Metrics per run (window: engage .. engage + run_time):
  collision      ego rectangle intersects the true obstacle rectangle while it truly exists
  min_clear      minimum ego-obstacle rectangle distance (m)
  false_del      times the true obstacle vanished from the planning input while it truly exists
  absent_ratio   fraction of frames the true obstacle was missing from the planning input
  delay          S4: time from the real deletion to its disappearance from the planning input (s)
  jump_max       max frame-to-frame deviation of the planned trajectory, first `--jump-len` m (m)
  jump_cnt       frames whose deviation exceeds `--jump-thr` m
  latjerk_peak   peak |d(v*yaw_rate)/dt| from ego odometry, 0.5 s moving average (m/s^3)
  progress       distance driven in the first `--prog-time` s after engage (m)
  stop_time      time with speed < 0.1 m/s after the first 3 s (s)
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics as st

from .common import rect_corners, polys_intersect, poly_distance

EGO = dict(front=3.79, rear=1.1, half_w=0.948)   # sample_vehicle: wheel_base 2.79 + overhangs


def polyline_dist(p, pl):
    best = float('inf')
    for i in range(len(pl) - 1):
        ax, ay = pl[i]
        bx, by = pl[i + 1]
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
        best = min(best, math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy))
    return best


def first_len(pts, length):
    out, s = [pts[0]], 0.0
    for a, b in zip(pts, pts[1:]):
        s += math.hypot(b[0] - a[0], b[1] - a[1])
        out.append(b)
        if s >= length:
            break
    return out


def metrics(log, a):
    ev = log['events']
    t0 = ev['t_engage']
    t1 = t0 + a.run_time
    ob = ev.get('obstacle')
    t_del = ev.get('t_delete')
    odom = [r for r in log['odom'] if t0 <= r[0] <= t1]
    objs = [r for r in log['objs'] if t0 <= r[0] <= t1]
    traj = [r for r in log['traj'] if t0 <= r[0] <= t1 and len(r[1]) >= 2]
    m = dict(scenario=ev['scenario'], method=ev['method'], seed=ev['seed'])

    exists = (lambda t: ob is not None and (t_del is None or t < t_del))
    # --- safety
    coll, mind = False, float('inf')
    if ob is not None:
        orect = rect_corners(ob['x'], ob['y'], ob['yaw'], ob['length'] / 2, ob['length'] / 2, ob['width'] / 2)
        for t, x, y, yaw, v, wz in odom:
            if not exists(t) or math.hypot(x - ob['x'], y - ob['y']) > 30:
                continue
            er = rect_corners(x, y, yaw, EGO['front'], EGO['rear'], EGO['half_w'])
            if polys_intersect(er, orect):
                coll = True
            mind = min(mind, poly_distance(er, orect))
    m['collision'] = int(coll)
    m['min_clear'] = round(mind, 3) if mind < float('inf') else ''

    # --- presence of the true obstacle in the planning input
    fd, absent, n, seen = 0, 0, 0, False
    prev_present = None
    for t, xy in objs:
        if not exists(t):
            continue
        present = any(math.hypot(px - ob['x'], py - ob['y']) < a.match for px, py in xy)
        if present:
            seen = True
        if seen:
            n += 1
            absent += (not present)
            if prev_present and not present:
                fd += 1
        prev_present = present
    m['false_del'] = fd if ob is not None else ''
    m['absent_ratio'] = round(absent / n, 3) if n else ''

    # --- S4 delay
    m['delay'] = ''
    if ob is not None and t_del is not None:
        after = [(t, any(math.hypot(px - ob['x'], py - ob['y']) < a.match for px, py in xy))
                 for t, xy in objs if t >= t_del]
        last_present = max([t for t, p in after if p], default=None)
        gone = [t for t, p in after if not p and (last_present is None or t > last_present)]
        m['delay'] = round(gone[0] - t_del, 3) if gone else round(t1 - t_del, 3)

    # --- trajectory jumps (spatial alignment, robust to missing time_from_start)
    jumps = []
    for (ta, pa), (tb, pb) in zip(traj, traj[1:]):
        cur = first_len(pb, a.jump_len)
        jumps.append(max(polyline_dist(p, pa) for p in cur))
    m['jump_max'] = round(max(jumps), 3) if jumps else ''
    m['jump_cnt'] = sum(j > a.jump_thr for j in jumps)

    # --- lateral jerk
    jerk = []
    ay = [(t, v * wz) for t, x, y, yaw, v, wz in odom]
    for (ta, aa), (tb, ab) in zip(ay, ay[1:]):
        if tb - ta > 1e-3:
            jerk.append((tb, (ab - aa) / (tb - ta)))
    sm = []
    for i in range(len(jerk)):
        w = [j for t, j in jerk if jerk[i][0] - 0.5 <= t <= jerk[i][0]]
        sm.append(abs(sum(w) / len(w)))
    m['latjerk_peak'] = round(max(sm), 3) if sm else ''

    # --- progress and stop time
    prog, stop = 0.0, 0.0
    for r0, r1 in zip(odom, odom[1:]):
        if r1[0] <= t0 + a.prog_time:
            prog += math.hypot(r1[1] - r0[1], r1[2] - r0[2])
        if r1[0] > t0 + 3 and r1[4] < 0.1:
            stop += r1[0] - r0[0]
    m['progress'] = round(prog, 2)
    m['stop_time'] = round(stop, 2)
    return m


KEYS = ['collision', 'min_clear', 'false_del', 'absent_ratio', 'delay', 'jump_max', 'jump_cnt',
        'latjerk_peak', 'progress', 'stop_time']


def summarize(rows):
    groups = {}
    for r in rows:
        groups.setdefault((r['scenario'], r['method']), []).append(r)
    out = []
    for (s, mth), rr in sorted(groups.items()):
        o = dict(scenario=s, method=mth, n=len(rr))
        for k in KEYS:
            vals = [float(r[k]) for r in rr if r[k] != '']
            if not vals:
                o[k] = ''
                continue
            if k == 'collision':
                o[k] = f'{int(sum(vals))}/{len(vals)}'
                continue
            mu = st.mean(vals)
            ci = 1.96 * st.stdev(vals) / math.sqrt(len(vals)) if len(vals) > 1 else 0.0
            o[k] = f'{mu:.3f} ± {ci:.3f}'
        out.append(o)
    return out


def plots(rows, out_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('未安装 matplotlib，跳过画图')
        return
    scen = sorted({r['scenario'] for r in rows})
    meth = sorted({r['method'] for r in rows})
    for k in ['jump_max', 'false_del', 'progress', 'latjerk_peak', 'min_clear', 'delay']:
        fig, ax = plt.subplots(figsize=(1.4 + 1.3 * len(scen), 3.6))
        w = 0.8 / max(len(meth), 1)
        any_val = False
        for j, mt in enumerate(meth):
            mus, cis = [], []
            for s in scen:
                v = [float(r[k]) for r in rows if r['scenario'] == s and r['method'] == mt and r[k] != '']
                mus.append(st.mean(v) if v else float('nan'))
                cis.append(1.96 * st.stdev(v) / math.sqrt(len(v)) if len(v) > 1 else 0)
                any_val |= bool(v)
            ax.bar([i + j * w for i in range(len(scen))], mus, w, yerr=cis, capsize=2, label=mt)
        if not any_val:
            plt.close(fig)
            continue
        ax.set_xticks([i + w * (len(meth) - 1) / 2 for i in range(len(scen))])
        ax.set_xticklabels(scen)
        ax.set_title(k)
        ax.legend(fontsize=7, ncol=2)
        ax.grid(axis='y', alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f'{k}.png'), dpi=150)
        plt.close(fig)
    # trade-off: S1 false deletions vs S4 delay
    fig, ax = plt.subplots(figsize=(5, 4))
    for mt in meth:
        fdv = [float(r['false_del']) for r in rows if r['scenario'] == 'S1' and r['method'] == mt and r['false_del'] != '']
        dl = [float(r['delay']) for r in rows if r['scenario'] == 'S4' and r['method'] == mt and r['delay'] != '']
        if fdv and dl:
            ax.scatter(st.mean(dl), st.mean(fdv), s=40)
            ax.annotate(mt, (st.mean(dl), st.mean(fdv)), xytext=(4, 4), textcoords='offset points')
    ax.set_xlabel('S4 delay after true disappearance (s)')
    ax.set_ylabel('S1 false deletions per run')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'tradeoff.png'), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True)
    ap.add_argument('--root', default=os.path.expanduser('~/fj_ws/results'))
    ap.add_argument('--run-time', type=float, default=35.0)
    ap.add_argument('--prog-time', type=float, default=30.0)
    ap.add_argument('--jump-len', type=float, default=30.0)
    ap.add_argument('--jump-thr', type=float, default=0.3)
    ap.add_argument('--match', type=float, default=2.5)
    a = ap.parse_args()
    d = os.path.join(a.root, a.name)
    files = sorted(glob.glob(os.path.join(d, '*_seed*.json')))
    if not files:
        print(f'{d} 下没有结果文件')
        return
    rows = []
    for f in files:
        try:
            rows.append(metrics(json.load(open(f)), a))
        except Exception as e:  # keep going on a broken file
            print(f'跳过 {os.path.basename(f)}: {e}')
    out = os.path.join(d, 'eval')
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'per_run.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['scenario', 'method', 'seed'] + KEYS)
        w.writeheader()
        w.writerows(rows)
    summ = summarize(rows)
    with open(os.path.join(out, 'summary.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=['scenario', 'method', 'n'] + KEYS)
        w.writeheader()
        w.writerows(summ)
    for o in summ:
        print(o)
    plots(rows, out)
    print(f'\n结果已保存到 {out}')


if __name__ == '__main__':
    main()

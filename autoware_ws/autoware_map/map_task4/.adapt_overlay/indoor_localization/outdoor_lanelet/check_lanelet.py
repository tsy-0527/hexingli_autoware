#!/usr/bin/env python3
"""车道图检查：左右边界是否反了、同方向重叠、后继/前驱缺失（按 Lanelet2 的节点 ID 规则）。
用法：check_lanelet.py lanelet2_map.osm"""
import sys, collections, xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import cKDTree

def load(path):
    r = ET.parse(path).getroot()
    nodes = {}
    for n in r.findall('node'):
        t = {k.get('k'): k.get('v') for k in n.findall('tag')}
        nodes[n.get('id')] = np.array([float(t['local_x']), float(t['local_y'])])
    ways = {w.get('id'): [x.get('ref') for x in w.findall('nd')] for w in r.findall('way')}
    LL = {}
    for rl in r.findall('relation'):
        tags = {k.get('k'): k.get('v') for k in rl.findall('tag')}
        if tags.get('type') != 'lanelet':
            continue
        mem = {m.get('role'): m.get('ref') for m in rl.findall('member')}
        L = np.array([nodes[i] for i in ways[mem['left']]]); R = np.array([nodes[i] for i in ways[mem['right']]])
        def arclen(P): return float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
        n = max(2, int(max(arclen(L), arclen(R)) / 0.25) + 1)     # 按 0.25 m 弧长采样
        def rs(P):
            d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]; s = np.linspace(0, d[-1], n)
            return np.c_[np.interp(s, d, P[:, 0]), np.interp(s, d, P[:, 1])]
        Lr, Rr = rs(L), rs(R); C = (Lr + Rr) / 2; t = np.diff(C, axis=0); lr = (Lr - Rr)[:-1]
        LL[rl.get('id')] = dict(C=C, left_ok=float(np.mean(t[:, 0] * lr[:, 1] - t[:, 1] * lr[:, 0] > 0)),
                                le=ways[mem['left']][-1], re=ways[mem['right']][-1],
                                ls=ways[mem['left']][0], rs=ways[mem['right']][0])
    return LL

def main(path):
    LL = load(path)
    succ = {a: [b for b, lb in LL.items() if b != a and (la['le'], la['re']) == (lb['ls'], lb['rs'])] for a, la in LL.items()}
    inv = sorted(k for k, l in LL.items() if l['left_ok'] < 0.9)
    ov = []
    for a in LL:
        for b in LL:
            if a == b: continue
            A, B = LL[a]['C'], LL[b]['C']; d, j = cKDTree(B).query(A); on = d < 0.3
            if on.mean() > 0.8:
                ta = np.diff(A, axis=0); tb = np.diff(B, axis=0); jj = np.clip(j[:-1], 0, len(tb) - 1)
                cos = np.sum(ta * tb[jj], 1) / (np.linalg.norm(ta, axis=1) * np.linalg.norm(tb[jj], axis=1) + 1e-9)
                if np.mean(cos[on[:-1]] > 0.5) > 0.8: ov.append((a, b))
    print(f'lanelet 数量: {len(LL)}')
    print(f'左右边界反了: {inv or "无"}')
    print(f'同方向重叠: {ov or "无"}')
    print(f'没有后继（路的尽头/需换道）: {sorted(a for a in LL if not succ[a])}')
    print(f'没有前驱（路的起点/需换道）: {sorted(a for a in LL if not any(a in v for v in succ.values()))}')
    return 0 if not inv and not ov else 1

if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))

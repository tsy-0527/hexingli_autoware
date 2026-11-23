#!/usr/bin/env python3
"""修正室外车道图（只改拓扑/角色，不移动任何点）：
  1. 左右边界反了的车道：交换 left/right 成员
  2. 删除与分段车道同向完全重叠的整条车道（60110-60112、960110-960112）
  3. 补上最南侧车道在 x≈17-68 段的东西两向车道（删整条车道后这一段会缺）
用法：fix_lanelet.py <in.osm> <out.osm>
"""
import sys, copy, math
import xml.etree.ElementTree as ET
import numpy as np

SWAP = ['30112', '30113', '30114', '50110', '50111', '930112', '930113', '930114',
        '950110', '950111', '970382', '970383', '970384', '970385']
REMOVE = ['60110', '60111', '60112', '960110', '960111', '960112']

def main(src, dst):
    tree = ET.parse(src); root = tree.getroot()
    rels = {r.get('id'): r for r in root.findall('relation')}
    ways = {w.get('id'): w for w in root.findall('way')}
    for rid in SWAP:
        r = rels[rid]
        mem = {m.get('role'): m for m in r.findall('member')}
        mem['left'].set('role', 'right'); mem['right'].set('role', 'left')
    for rid in REMOVE:
        root.remove(rels[rid])
    # 南侧车道西段：东向 L=50107 R=50106；西向 L=50106 反向（新 way）R=950107
    rev = copy.deepcopy(ways['50106']); rev.set('id', '980010')
    nds = rev.findall('nd')
    for nd in nds: rev.remove(nd)
    tags = rev.findall('tag')
    for t in tags: rev.remove(t)
    for nd in reversed(nds): rev.append(nd)
    for t in tags: rev.append(t)
    last_way = root.findall('way')[-1]
    root.insert(list(root).index(last_way) + 1, rev)
    def lanelet(rid, left, right):
        r = ET.SubElement(root, 'relation', {'id': rid})
        ET.SubElement(r, 'member', {'type': 'way', 'role': 'left', 'ref': left})
        ET.SubElement(r, 'member', {'type': 'way', 'role': 'right', 'ref': right})
        for k, v in (('type', 'lanelet'), ('subtype', 'road'), ('speed_limit', '10'), ('location', 'urban'), ('one_way', 'yes')):
            ET.SubElement(r, 'tag', {'k': k, 'v': v})
    lanelet('980001', '50107', '50106')     # 东向
    lanelet('980002', '980010', '950107')   # 西向
    ET.indent(tree, space='  ')
    tree.write(dst, encoding='UTF-8', xml_declaration=True)

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])

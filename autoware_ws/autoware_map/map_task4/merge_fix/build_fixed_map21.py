#!/usr/bin/env python3
"""Rebuild lanelet2_map(21).osm: proper Y merge/exit, fully routable, 2-way.

The original map has 7 lanelets but *zero* routing relations (no shared
nodes).  At the Y junction around (60.7, -74) the ramp ``conn`` (from the
north), the western link ``main_w`` (from the south-west) and the eastern
lane ``east_in`` meet with three differently oriented end edges that only
touch at one centre point.  ``east_out`` starts as an isolated stub at
x ~= 50.6.

Rebuild:
  * main_w is split at x ~= 50 into the through part and ``main_link``
    (the branch climbing to the Y); east_out continues the through part.
  * the Y merge/exit cross-section is moved to east_in's vertex #13
    (x ~= 65.5) where east_in runs east-south-east, so both incoming
    ramps (conn, main_link) join / leave with a smooth common tangent.
  * conn and main_link tails are re-sampled with cubic Beziers that end
    exactly on that cross-section (shared nodes with east_in).
  * start and park are split at their T-junctions (lanelet2 needs
    junctions at lanelet ends), the uturn mouth is snapped to the new
    east_in/east_out end edges.
  * every connection shares node IDs; one_way=no is kept everywhere.
"""
import sys

sys.path.insert(0, '/home/tang/autoware_map/map_task4/merge_fix')
from build_fixed_map import (  # noqa: E402
    LANE_WIDTH, add, bezier, check_geometry, centerline, distance, lerp,
    load_map, mul, point_polyline_distance, sub, unit, vertex_left_normals,
    way_points, write_map,
)

ORIGINAL = '/home/tang/下载/lanelet2_map(21).osm'
OUTPUT = '/home/tang/autoware_map/map_task4/merge_fix/lanelet2_map21_merge_fixed.osm'

MERGE_I = 13      # east_in vertex used as the Y merge/exit cross-section
PARK_I = 25       # park boundary vertex before the conn branch
MAIN_I = 3        # main_w boundary vertex used for the x~50 fork split
CONN_TAIL = 42    # conn centreline vertex where the Bezier tail starts


def tail(center, left, right, keep, end_center, end_tangent, target_l,
         target_r, samples=24):
    """Replace the tail from ``keep`` on with a Bezier to the merge edge."""
    t_in = unit(sub(center[keep], center[keep - 1]))
    d = distance(center[keep], end_center)
    handle = min(0.45 * d, 4.0)
    p1 = add(center[keep], mul(t_in, handle))
    p2 = sub(end_center, mul(end_tangent, handle))
    curve = bezier(center[keep], p1, p2, end_center, samples)[1:]
    new_center = list(center[:keep + 1]) + curve
    normals = vertex_left_normals(new_center)
    new_l = list(left[:keep + 1]) + [
        add(new_center[i], mul(normals[i], LANE_WIDTH / 2))
        for i in range(keep + 1, len(new_center))]
    new_r = list(right[:keep + 1]) + [
        add(new_center[i], mul(normals[i], -LANE_WIDTH / 2))
        for i in range(keep + 1, len(new_center))]
    new_l[-1] = target_l
    new_r[-1] = target_r
    return new_center, new_l, new_r


def lane_from_centerline(center, first_l, first_r, last_l, last_r):
    """Build 2.5 m boundaries from a centreline, pinning the end edges."""
    normals = vertex_left_normals(center)
    left = [add(center[i], mul(normals[i], LANE_WIDTH / 2))
            for i in range(len(center))]
    right = [add(center[i], mul(normals[i], -LANE_WIDTH / 2))
             for i in range(len(center))]
    left[0], right[0] = first_l, first_r
    left[-1], right[-1] = last_l, last_r
    return left, right


def build_geometry(nodes, ways):
    def w(wid):
        return way_points(nodes, ways, wid)

    start_l, start_r = w('1000'), w('1002')
    park_l, park_r = w('1003'), w('1005')
    conn_l, conn_c, conn_r = w('1006'), w('1007'), w('1008')
    main_l, main_c, main_r = w('1009'), w('1010'), w('1011')
    out_l, out_r = w('1012'), w('1014')
    in_l, in_c, in_r = w('1015'), w('1016'), w('1017')
    ut_l, ut_r = w('1018'), w('1020')

    assert len(start_l) == len(start_r) == 20
    assert len(park_l) == len(park_r) == 63
    assert len(conn_l) == len(conn_c) == len(conn_r) == 44
    assert len(main_l) == len(main_c) == len(main_r) == 29
    assert len(out_l) == len(out_r) == 51
    assert len(in_l) == len(in_c) == len(in_r) == 64
    assert len(ut_l) == len(ut_r) == 71

    # ---- 1. start split at park's return cross-section --------------
    J1_l, J1_r = start_l[8], start_r[9]
    start_s_l, start_s_r = list(start_l[:9]), list(start_r[:10])
    start_n_l, start_n_r = list(start_l[8:]), list(start_r[9:])
    start_n_l[-1] = park_l[0]
    start_n_r[-1] = park_r[0]

    # ---- 2. park split at the conn branch ---------------------------
    branch_l, branch_r = conn_r[0], conn_l[0]
    park_a_l = list(park_l[:PARK_I + 1]) + [branch_l]
    park_a_r = list(park_r[:PARK_I + 1]) + [branch_r]
    park_b_l = [branch_l] + list(park_l[PARK_I + 1:])
    park_b_r = [branch_r] + list(park_r[PARK_I + 1:])
    park_b_l[-1] = J1_r
    park_b_r[-1] = J1_l

    # ---- 3. main_w split at the x~50 fork ---------------------------
    M_l, M_r = main_l[MAIN_I], main_r[MAIN_I]
    main_w_l = list(main_l[:MAIN_I + 1])
    main_w_r = list(main_r[:MAIN_I + 1])

    # east_out continues the through part from the same cross-section
    out_l = list(out_l)
    out_r = list(out_r)
    out_l[0] = M_l
    out_r[0] = M_r

    # ---- 4. Y merge/exit cross-section on east_in -------------------
    S_L = in_l[MERGE_I]
    S_R = in_r[MERGE_I]
    P3 = in_c[MERGE_I]
    T_OUT = unit(sub(in_c[MERGE_I], in_c[MERGE_I - 1]))
    in_tail_l = list(in_l[MERGE_I:])
    in_tail_r = list(in_r[MERGE_I:])
    in_tail_c = list(in_c[MERGE_I:])

    # ---- 5. reshape conn tail into the Y cross-section --------------
    conn_c_new, conn_l_new, conn_r_new = tail(
        conn_c, conn_l, conn_r, CONN_TAIL, P3, T_OUT, S_L, S_R)

    # ---- 6. main_link: smooth ramp from the fork to the Y -----------
    T_IN = (0.9659258, 0.2588190)          # 15 deg north of east
    p0 = main_c[MAIN_I]
    p1 = add(p0, mul(T_IN, 6.0))
    p2 = sub(P3, mul(T_OUT, 5.0))
    link_c_new = bezier(p0, p1, p2, P3, 40)
    link_l_new, link_r_new = lane_from_centerline(
        link_c_new, M_l, M_r, S_L, S_R)

    # ---- 7. U-turn mouth on the new lane end edges ------------------
    out_end_l, out_end_r = out_l[-1], out_r[-1]
    in_end_l, in_end_r = in_tail_l[-1], in_tail_r[-1]
    ut_l_new = list(ut_l)
    ut_r_new = list(ut_r)
    ut_l_new[0] = out_end_l
    ut_r_new[0] = out_end_r
    ut_l_new[-1] = in_end_r
    ut_r_new[-1] = in_end_l

    polylines = {
        'start.left': (start_s_l, 'solid'),
        'start.right': (start_s_r, 'solid'),
        'start_north.left': (start_n_l, 'solid'),
        'start_north.right': (start_n_r, 'solid'),
        'park.left': (park_a_l, 'solid'),
        'park.right': (park_a_r, 'solid'),
        'park_loop.left': (park_b_l, 'solid'),
        'park_loop.right': (park_b_r, 'solid'),
        'conn.left': (conn_l_new, 'solid'),
        'conn.right': (conn_r_new, 'solid'),
        'main_w.left': (main_w_l, 'solid'),
        'main_w.right': (main_w_r, 'solid'),
        'main_link.left': (link_l_new, 'solid'),
        'main_link.right': (link_r_new, 'solid'),
        'east_out.left': (out_l, 'solid'),
        'east_out.right': (out_r, 'solid'),
        'east_in.left': (in_tail_l, 'solid'),
        'east_in.right': (in_tail_r, 'solid'),
        'uturn.left': (ut_l_new, 'solid'),
        'uturn.right': (ut_r_new, 'solid'),
    }

    lanelets = [
        {'id': '2000', 'lane_id': 'start', 'left': 'start.left',
         'right': 'start.right', 'bounds': (start_s_l, start_s_r)},
        {'id': '2007', 'lane_id': 'start_north', 'left': 'start_north.left',
         'right': 'start_north.right', 'bounds': (start_n_l, start_n_r)},
        {'id': '2001', 'lane_id': 'park', 'left': 'park.left',
         'right': 'park.right', 'bounds': (park_a_l, park_a_r)},
        {'id': '2008', 'lane_id': 'park_loop', 'left': 'park_loop.left',
         'right': 'park_loop.right', 'bounds': (park_b_l, park_b_r)},
        {'id': '2002', 'lane_id': 'conn', 'left': 'conn.left',
         'right': 'conn.right', 'bounds': (conn_l_new, conn_r_new)},
        {'id': '2003', 'lane_id': 'main_w', 'left': 'main_w.left',
         'right': 'main_w.right', 'bounds': (main_w_l, main_w_r)},
        {'id': '2009', 'lane_id': 'main_link', 'left': 'main_link.left',
         'right': 'main_link.right', 'bounds': (link_l_new, link_r_new)},
        {'id': '2004', 'lane_id': 'east_out', 'left': 'east_out.left',
         'right': 'east_out.right', 'bounds': (out_l, out_r)},
        {'id': '2005', 'lane_id': 'east_in', 'left': 'east_in.left',
         'right': 'east_in.right', 'bounds': (in_tail_l, in_tail_r)},
        {'id': '2006', 'lane_id': 'uturn', 'left': 'uturn.left',
         'right': 'uturn.right', 'bounds': (ut_l_new, ut_r_new)},
    ]
    return polylines, lanelets


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else OUTPUT
    nodes, ways, relations = load_map(ORIGINAL)
    polylines, lanelets = build_geometry(nodes, ways)

    problems = check_geometry(polylines, lanelets)
    if problems:
        print('GEOMETRY PROBLEMS:')
        for p in problems:
            print(' -', p)
    else:
        print('geometry checks: OK')

    way_ids, nodes_out = write_map(out_path, polylines, lanelets, nodes)
    print(f'\nwritten: {out_path}')
    print(f'nodes={len(nodes_out)} ways={len(way_ids) + len(lanelets)} '
          f'lanelets={len(lanelets)}')
    for ll in lanelets:
        print(f"  {ll['id']} {ll['lane_id']:12s} "
              f"left={way_ids[ll['left']]:5s} right={way_ids[ll['right']]:5s}")
    print('\n=== Y merge/exit edges ===')
    for ll in lanelets:
        if ll['lane_id'] in ('conn', 'main_link', 'east_in'):
            left, right = ll['bounds']
            print(f"  {ll['lane_id']:10s} start L={left[0][0]:8.3f},{left[0][1]:8.3f}"
                  f" R={right[0][0]:8.3f},{right[0][1]:8.3f} | "
                  f"end L={left[-1][0]:8.3f},{left[-1][1]:8.3f}"
                  f" R={right[-1][0]:8.3f},{right[-1][1]:8.3f}")
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())



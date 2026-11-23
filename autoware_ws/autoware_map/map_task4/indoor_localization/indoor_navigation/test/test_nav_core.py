import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from indoor_navigation.ackermann import AckermannParams, AckermannShaper
from indoor_navigation.route_graph import RouteGraph

DT = 0.05

def run(sh, cmds, t0=0.0, v_meas=None):
    out = []
    t = t0
    for v, w in cmds:
        sh.on_cmd(t)
        out.append(sh.step(t, DT, v, w, 1.0, v_meas))
        t += DT
    return out, t

def test_curvature_and_limits():
    sh = AckermannShaper(AckermannParams())
    out, _ = run(sh, [(0.4, 0.4 / 1.3)] * 80)          # arc of radius 1.3 m
    o = out[-1]
    exp = math.atan(0.475 / 1.3)
    assert abs(o['steer'] - exp) < 1e-6 and abs(o['speed'] - 0.4) < 1e-6
    out, _ = run(AckermannShaper(AckermannParams()), [(0.4, 2.0)] * 80)   # too tight -> clipped
    assert abs(out[-1]['steer'] - math.radians(21)) < 1e-9
    print(f'curvature ok: R=1.3 m -> {math.degrees(exp):.2f} deg; tight command clipped to 21 deg')

def test_presteer_and_rate():
    p = AckermannParams(max_steer_rate=math.radians(30))
    sh = AckermannShaper(p)
    out, _ = run(sh, [(0.3, 0.3 / 1.3)] * 40)
    first_move = next(i for i, o in enumerate(out) if o['speed'] > 0)
    steer_at_move = math.degrees(out[first_move]['steer'])
    max_rate = max(abs(b['steer'] - a['steer']) for a, b in zip(out, out[1:])) / DT
    assert max_rate <= math.radians(30) + 1e-9
    assert steer_at_move > math.degrees(math.atan(0.475 / 1.3)) - 4.0 - 1e-6
    print(f'pre-steer ok: starts moving after {first_move * DT:.2f}s with wheels at {steer_at_move:.1f} deg; max steer rate {math.degrees(max_rate):.1f} deg/s')

def test_direction_change():
    sh = AckermannShaper(AckermannParams())
    out1, t = run(sh, [(0.3, 0.0)] * 40)
    out2, _ = run(sh, [(-0.2, 0.0)] * 60, t0=t)
    speeds = [o['speed'] for o in out1 + out2]
    first_neg = next(i for i, s in enumerate(speeds) if s < 0)
    zero_run = 0
    i = first_neg - 1
    while i >= 0 and speeds[i] == 0.0:
        zero_run += 1; i -= 1
    assert all(s >= 0 for s in speeds[:first_neg])
    assert zero_run * DT >= 0.5 - 1e-9
    assert abs(speeds[-1] + 0.2) < 1e-6
    print(f'direction change ok: held at standstill {zero_run * DT:.2f}s before reversing; reverse speed {speeds[-1]:.2f}')

def test_timeout_and_factor():
    sh = AckermannShaper(AckermannParams())
    run(sh, [(0.4, 0.0)] * 60)
    o = sh.step(10.0, DT, 0.4, 0.0, 1.0)          # no on_cmd for a long time -> timeout
    assert o['timed_out'] and o['speed'] < 0.4
    sh2 = AckermannShaper(AckermannParams())
    out = [None]
    t = 0
    for _ in range(80):
        sh2.on_cmd(t); out[0] = sh2.step(t, DT, 0.4, 0.0, 0.5); t += DT
    assert abs(out[0]['speed'] - 0.2) < 1e-6
    print('timeout brake ok; health speed factor 0.5 -> 0.2 m/s ok')

def test_route():
    import yaml
    d = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'stations.yaml')))
    g = RouteGraph.from_dict(d)
    nodes, wps = g.route(22.8, -10.6, 'hall_station_1')
    print('route room -> hall_station_1:', nodes, [(round(x, 2), round(y, 2), round(math.degrees(a))) for x, y, a in wps])
    assert nodes[-1] == d['stations']['hall_station_1']['node'] and nodes[0] != 'room_center'
    nodes2, wps2 = g.route(13.0, -46.0, 'room_home')
    print('route hall -> room_home:', nodes2, [(round(x, 2), round(y, 2), round(math.degrees(a))) for x, y, a in wps2])
    assert nodes2[-1] == 'room_center' and abs(math.degrees(wps2[-1][2]) - d['stations']['room_home']['yaw_deg']) < 1e-6

if __name__ == '__main__':
    test_curvature_and_limits(); test_presteer_and_rate(); test_direction_change(); test_timeout_and_factor(); test_route()
    print('ackermann core: ALL PASS')

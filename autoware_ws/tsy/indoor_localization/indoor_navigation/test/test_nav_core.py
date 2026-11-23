import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from indoor_navigation.ackermann import AckermannParams, AckermannShaper
from indoor_navigation.route_graph import RouteGraph

DT = 0.05

def run(sh, cmds, t0=0.0, v_meas=None, steer_meas=None):
    out = []
    t = t0
    for v, w in cmds:
        sh.on_cmd(t)
        measured = steer_meas(t) if callable(steer_meas) else steer_meas
        out.append(sh.step(t, DT, v, w, 1.0, v_meas, measured))
        t += DT
    return out, t

def test_curvature_and_limits():
    sh = AckermannShaper(AckermannParams())
    out, _ = run(sh, [(0.4, 0.4 / 1.3)] * 80)          # arc of radius 1.3 m
    o = out[-1]
    exp = math.atan(0.475 / 1.3)
    ratio = (exp - sh.p.tight_turn_start) / (sh.p.max_steer - sh.p.tight_turn_start)
    exp_cap = sh.p.max_speed_fwd + ratio * (sh.p.max_speed_tight_fwd - sh.p.max_speed_fwd)
    assert abs(o['steer'] - exp) < 1e-6 and abs(o['speed'] - exp_cap) < 1e-6
    out, _ = run(AckermannShaper(AckermannParams()), [(0.4, 2.0)] * 80)   # too tight -> clipped
    assert abs(out[-1]['steer'] - math.radians(21)) < 1e-9
    print(f'curvature ok: R=1.3 m -> {math.degrees(exp):.2f} deg at {exp_cap:.2f} m/s; tight command clipped to 21 deg')

def test_presteer_and_rate():
    p = AckermannParams(max_steer_rate=math.radians(30))
    sh = AckermannShaper(p)
    out, _ = run(sh, [(0.3, 0.3 / 1.3)] * 40)
    first_move = next(i for i, o in enumerate(out) if o['speed'] > 0)
    steer_at_move = math.degrees(out[first_move]['steer'])
    max_rate = max(abs(b['steer'] - a['steer']) for a, b in zip(out, out[1:])) / DT
    assert max_rate <= math.radians(30) + 1e-9
    assert steer_at_move > math.degrees(math.atan(0.475 / 1.3)) - 1.5 - 1e-6
    print(f'pre-steer ok: starts moving after {first_move * DT:.2f}s with wheels at {steer_at_move:.1f} deg; max steer rate {math.degrees(max_rate):.1f} deg/s')

def test_real_steering_feedback_gate():
    p = AckermannParams(steer_settle_time=0.4)
    sh = AckermannShaper(p)
    target = math.atan(p.wheelbase / 1.25)
    # The command reaches its target, but the reported real wheels remain
    # straight: motion must remain inhibited indefinitely.
    out, t = run(sh, [(0.25, 0.25 / 1.25)] * 30, steer_meas=0.0)
    assert max(abs(o['speed']) for o in out) == 0.0
    assert not out[-1]['steer_ready']
    # Once real feedback reaches the target it must remain there for the full
    # settle time before acceleration is permitted.
    out2, _ = run(sh, [(0.25, 0.25 / 1.25)] * 20, t0=t, steer_meas=target)
    first_move = next(i for i, o in enumerate(out2) if o['speed'] > 0.0)
    assert first_move * DT >= p.steer_settle_time - DT - 1e-9
    assert out2[first_move]['steer_ready']

def test_tight_turn_speed_cap():
    p = AckermannParams()
    sh = AckermannShaper(p)
    out, _ = run(sh, [(0.4, 2.0)] * 100, steer_meas=p.max_steer)
    assert abs(out[-1]['speed'] - p.max_speed_tight_fwd) < 1e-6
    assert abs(out[-1]['turn_speed_limit'] - p.max_speed_tight_fwd) < 1e-6

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
    # 起点坐标：旧室内坐标 (22.8, -10.6)（房间）按 legacy_room_registration.yaml 换算到统一 map 坐标
    nodes, wps = g.route(34.58, -27.43, 'hall_station_1')
    print('route room -> hall_station_1:', nodes, [(round(x, 2), round(y, 2), round(math.degrees(a))) for x, y, a in wps])
    assert nodes[-1] == d['stations']['hall_station_1']['node'] and nodes[0] != 'room_center'
    assert nodes[:3] == ['room_exit_prepare', 'room_exit_approach', 'room_exit_corner'], nodes
    nodes2, wps2 = g.route(26.91, -63.35, 'room_home')   # 旧室内坐标 (13.0, -46.0)，大厅
    print('route hall -> room_home:', nodes2, [(round(x, 2), round(y, 2), round(math.degrees(a))) for x, y, a in wps2])
    assert nodes2[-1] == 'room_center' and abs(math.degrees(wps2[-1][2]) - d['stations']['room_home']['yaw_deg']) < 1e-6
    # 交接点（南门外主路）进门、以及从房间出门到交接点
    # 门前路沿坎不能直接上，必须经坡道：进门坡道口朝北、坡道上朝东；出门反过来
    deg = lambda a: round(math.degrees(a) % 360.0, 3)
    n3, w3 = g.route(29.0, -76.05, 'room_home')
    print('route handover -> room_home:', n3, [(round(x, 2), round(y, 2), deg(a)) for x, y, a in w3])
    assert n3[:3] == ['ramp_approach', 'door_ramp', 'door_inside'] and n3[-1] == 'room_center', n3
    assert deg(w3[0][2]) == 90.0 and deg(w3[1][2]) == 2.634, w3[:2]
    n4, w4 = g.route(34.36, -27.24, 'south_door_out')
    print('route room -> south_door_out:', n4[-4:], [(round(x, 2), round(y, 2), deg(a)) for x, y, a in w4[-4:]])
    assert n4[-4:] == ['door_inside', 'door_ramp', 'ramp_approach', 'south_door_out'], n4
    assert deg(w4[-3][2]) == 182.634 and deg(w4[-2][2]) == 270.0 and abs(deg(w4[-1][2]) - 2.634) < 1e-6, w4[-3:]

if __name__ == '__main__':
    test_curvature_and_limits(); test_presteer_and_rate(); test_real_steering_feedback_gate(); test_tight_turn_speed_cap()
    test_direction_change(); test_timeout_and_factor(); test_route()
    print('ackermann core: ALL PASS')

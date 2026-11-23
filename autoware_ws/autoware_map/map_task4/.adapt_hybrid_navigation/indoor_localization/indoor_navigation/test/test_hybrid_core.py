from indoor_navigation.hybrid_core import ACTIVE, INDOOR, OUTDOOR, STOPPING
from indoor_navigation.hybrid_core import ArbiterParams, ModeArbiter
from indoor_navigation.hybrid_core import point_in_polygon


def test_stop_first_handover_requires_fresh_target():
    a = ModeArbiter(OUTDOOR, ArbiterParams(
        stop_speed=0.03, stop_hold_time=1.0, source_timeout=0.5))
    a.note_source(OUTDOOR, 0.0)
    a.request(INDOOR)
    assert a.phase == STOPPING
    assert a.step(0.0, 0.2) == (OUTDOOR, True)
    assert a.step(0.5, 0.0) == (OUTDOOR, True)
    assert a.step(1.6, 0.0) == (OUTDOOR, True)  # no target command
    a.note_source(INDOOR, 1.6)
    assert a.step(1.61, 0.0, target_healthy=True) == (INDOOR, False)
    assert a.phase == ACTIVE


def test_unhealthy_indoor_never_gets_authority():
    a = ModeArbiter(OUTDOOR, ArbiterParams(stop_hold_time=0.5))
    a.request(INDOOR)
    a.note_source(INDOOR, 0.6)
    a.step(0.0, 0.0)
    assert a.step(0.6, 0.0, target_healthy=False) == (OUTDOOR, True)


def test_motion_resets_stop_hold():
    a = ModeArbiter(OUTDOOR, ArbiterParams(stop_hold_time=1.0))
    a.request(INDOOR)
    a.note_source(INDOOR, 1.5)
    a.step(0.0, 0.0)
    a.step(0.8, 0.1)
    a.step(1.0, 0.0)
    assert a.step(1.5, 0.0) == (OUTDOOR, True)


def test_point_in_polygon():
    p = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    assert point_in_polygon(1.0, 1.0, p)
    assert not point_in_polygon(3.0, 1.0, p)



# ---------------------------------------------------------------- 修正后新增
from indoor_navigation.hybrid_core import check_manual_request, health_ok  # noqa: E402


def _ready(a, mode, t, v=0.0):
    a.note_source(mode, t, v)


def test_stale_speed_blocks_handover():
    a = ModeArbiter(OUTDOOR, ArbiterParams(stop_hold_time=0.5))
    a.request(INDOOR)
    _ready(a, INDOOR, 0.0)
    a.step(0.0, 0.0)
    _ready(a, INDOOR, 0.6)
    assert a.step(0.6, 0.0, speed_valid=False) == (OUTDOOR, True)
    assert a.block_reason == 'vehicle speed unavailable'
    # 恢复后重新计时
    _ready(a, INDOOR, 0.7)
    assert a.step(0.7, 0.0) == (OUTDOOR, True)
    _ready(a, INDOOR, 1.25)
    assert a.step(1.25, 0.0) == (INDOOR, False)


def test_target_must_be_idle():
    a = ModeArbiter(INDOOR, ArbiterParams(stop_hold_time=0.5))
    a.request(OUTDOOR)
    a.step(0.0, 0.0)
    _ready(a, OUTDOOR, 0.6, v=1.2)          # Autoware 还在执行旧路线
    assert a.step(0.6, 0.0) == (INDOOR, True)
    assert 'commanding motion' in a.block_reason
    _ready(a, OUTDOOR, 0.7, v=0.0)          # Autoware 停下来了
    assert a.step(0.7, 0.0) == (OUTDOOR, False)


def test_cancel_during_stopping_returns_to_original_after_hold():
    a = ModeArbiter(OUTDOOR, ArbiterParams(stop_hold_time=0.5))
    a.request(INDOOR)
    a.step(0.0, 0.0)
    a.request(OUTDOOR)                      # 撤回
    assert a.phase == STOPPING
    _ready(a, OUTDOOR, 0.1)
    a.step(0.1, 0.0)
    _ready(a, OUTDOOR, 0.7)
    assert a.step(0.7, 0.0) == (OUTDOOR, False)
    assert a.phase == ACTIVE


def test_manual_request_zone_gate():
    enter = [(0, 0), (10, 0), (10, 10), (0, 10)]           # 室内
    exit_ = [(-3, -3), (13, -3), (13, 13), (-3, 13)]      # 室内 + 门口带
    assert check_manual_request(INDOOR, (-2, 5), enter, exit_)[0]      # 门口带：可以进
    assert not check_manual_request(INDOOR, (-6, 5), enter, exit_)[0]  # 离门太远
    assert check_manual_request(OUTDOOR, (-2, 5), enter, exit_)[0]     # 门口带：可以出
    assert not check_manual_request(OUTDOOR, (5, 5), enter, exit_)[0]  # 还在室内深处
    assert not check_manual_request(INDOOR, None, enter, exit_)[0]
    ok, why = check_manual_request(INDOOR, (100, 100), [], [])
    assert ok and 'not configured' in why


def test_health_ok():
    assert health_ok({'level': 'OK', 'speed_factor': 1.0})
    assert health_ok({'level': 'WARN', 'speed_factor': 0.5})
    assert not health_ok({'level': 'ERROR', 'speed_factor': 0.0})
    assert not health_ok({})
    assert not health_ok(None)

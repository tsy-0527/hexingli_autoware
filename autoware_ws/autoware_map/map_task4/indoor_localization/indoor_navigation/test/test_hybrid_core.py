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


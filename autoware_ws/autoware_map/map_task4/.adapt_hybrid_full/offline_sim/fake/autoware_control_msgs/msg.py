
import types
class Control:
    def __init__(s):
        s.stamp=None; s.control_time=None
        s.lateral=types.SimpleNamespace(stamp=None,control_time=None,steering_tire_angle=0.0,steering_tire_rotation_rate=0.0)
        s.longitudinal=types.SimpleNamespace(stamp=None,control_time=None,velocity=0.0,acceleration=0.0,is_defined_acceleration=False)

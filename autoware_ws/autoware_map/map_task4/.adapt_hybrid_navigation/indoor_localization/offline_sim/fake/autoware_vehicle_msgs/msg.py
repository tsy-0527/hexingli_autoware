
class GearCommand:
    NEUTRAL=1; DRIVE=2; REVERSE=20
    def __init__(s): s.stamp=None; s.command=1
class VelocityReport:
    def __init__(s,v=0.0): s.longitudinal_velocity=v

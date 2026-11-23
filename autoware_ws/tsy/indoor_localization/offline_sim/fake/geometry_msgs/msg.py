
import types
class PoseStamped:
    def __init__(s):
        s.header=types.SimpleNamespace(frame_id='',stamp=None)
        s.pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=0.),orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))

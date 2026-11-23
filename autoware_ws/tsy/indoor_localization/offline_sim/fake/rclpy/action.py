
import types
from sim import BUS
class _F:
    def __init__(s): s._d=False; s._r=None; s._cbs=[]
    def set(s,r):
        s._d=True; s._r=r
        for cb in s._cbs: cb(s)
    def done(s): return s._d
    def result(s): return s._r
    def add_done_callback(s,cb):
        if s._d: cb(s)
        else: s._cbs.append(cb)
class ActionClient:
    def __init__(s,node,t,name): BUS.actions[name]=s; s.goals=[]; s.cancels=0; s.result_future=None
    def wait_for_server(s,timeout_sec=0): return True
    def send_goal_async(s,goal,feedback_callback=None):
        s.goals.append(goal); f=_F(); rf=_F(); s.result_future=rf
        def cancel():
            s.cancels+=1; rf.set(types.SimpleNamespace(status=5)); return _F()
        gh=types.SimpleNamespace(accepted=True,get_result_async=lambda: rf,cancel_goal_async=cancel)
        f.set(gh); return f

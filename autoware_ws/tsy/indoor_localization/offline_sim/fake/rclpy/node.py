
import types
from sim import BUS, CLOCK
class _T:
    def __init__(s,t): s.nanoseconds=int(t*1e9)
    def to_msg(s): return types.SimpleNamespace(sec=int(s.nanoseconds//10**9),nanosec=int(s.nanoseconds%10**9))
class _C:
    def now(s): return _T(CLOCK[0])
class _L:
    def __init__(s,n): s.n=n
    def _p(s,l,m): BUS.log.append((s.n,l,m))
    def info(s,m): s._p('I',m)
    def warn(s,m): s._p('W',m)
    def error(s,m): s._p('E',m)
class _P:
    def __init__(s,v): s.value=v
class Node:
    def __init__(s,name): s._name=name; s._p={}
    def declare_parameter(s,n,v): s._p[n]=BUS.params.get(s._name,{}).get(n,v)
    def get_parameter(s,n): return _P(s._p[n])
    def get_logger(s): return _L(s._name)
    def get_clock(s): return _C()
    def create_subscription(s,t,topic,cb,q,**k):
        topic=topic.replace('~/','/'+s._name+'/'); BUS.subs.setdefault(topic,[]).append(cb)
    def create_publisher(s,t,topic,q):
        topic=topic.replace('~/','/'+s._name+'/'); return types.SimpleNamespace(publish=lambda m,topic=topic: BUS.pub(topic,m))
    def create_service(s,t,name,cb): BUS.srv['/'+s._name+'/'+name.replace('~/','')]=(t,cb)
    def create_timer(s,p,cb): BUS.timers.append([p,cb,0.0])
    def destroy_node(s): pass

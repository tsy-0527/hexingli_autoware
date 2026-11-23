
class _Bus:
    def __init__(s): s.subs={}; s.srv={}; s.timers=[]; s.params={}; s.log=[]; s.last={}; s.actions={}; s.count={}
    def pub(s,topic,m):
        s.last[topic]=m; s.count[topic]=s.count.get(topic,0)+1
        for cb in list(s.subs.get(topic,[])): cb(m)
BUS=_Bus(); CLOCK=[0.0]

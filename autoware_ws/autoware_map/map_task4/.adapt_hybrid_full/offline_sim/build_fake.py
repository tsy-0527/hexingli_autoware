import os, textwrap
R=os.path.join(os.path.dirname(os.path.abspath(__file__)),'fake')
def w(p,s):
    os.makedirs(os.path.dirname(os.path.join(R,p)),exist_ok=True); open(os.path.join(R,p),'w').write(textwrap.dedent(s))
w('sim.py','''
class _Bus:
    def __init__(s): s.subs={}; s.srv={}; s.timers=[]; s.params={}; s.log=[]; s.last={}; s.actions={}; s.count={}
    def pub(s,topic,m):
        s.last[topic]=m; s.count[topic]=s.count.get(topic,0)+1
        for cb in list(s.subs.get(topic,[])): cb(m)
BUS=_Bus(); CLOCK=[0.0]
''')
w('rclpy/__init__.py','def init(*a,**k): pass\ndef spin(n): pass\ndef shutdown(): pass\n')
w('rclpy/qos.py','''
class QoSProfile:
    def __init__(s,**k): pass
class DurabilityPolicy: TRANSIENT_LOCAL=1
class ReliabilityPolicy: RELIABLE=1
''')
w('rclpy/node.py','''
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
''')
w('rclpy/action.py','''
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
''')
w('std_msgs/__init__.py',''); w('std_msgs/msg.py','''
class String:
    def __init__(s,data=''): s.data=data
''')
w('std_srvs/__init__.py',''); w('std_srvs/srv.py','''
class SetBool:
    class Request:
        def __init__(s,data=False): s.data=data
    class Response:
        def __init__(s): s.success=False; s.message=''
class Trigger:
    class Request: pass
    class Response:
        def __init__(s): s.success=False; s.message=''
''')
w('nav_msgs/__init__.py',''); w('nav_msgs/msg.py','class Odometry: pass\n')
w('action_msgs/__init__.py',''); w('action_msgs/msg.py','''
class GoalStatus: STATUS_SUCCEEDED=4; STATUS_CANCELED=5; STATUS_ABORTED=6
''')
w('geometry_msgs/__init__.py',''); w('geometry_msgs/msg.py','''
import types
class PoseStamped:
    def __init__(s):
        s.header=types.SimpleNamespace(frame_id='',stamp=None)
        s.pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=0.),orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
''')
w('nav2_msgs/__init__.py',''); w('nav2_msgs/action.py','''
class NavigateThroughPoses:
    class Goal:
        def __init__(s): s.poses=[]; s.behavior_tree=''
''')
w('autoware_vehicle_msgs/__init__.py',''); w('autoware_vehicle_msgs/msg.py','''
class GearCommand:
    NEUTRAL=1; DRIVE=2; REVERSE=20
    def __init__(s): s.stamp=None; s.command=1
class VelocityReport:
    def __init__(s,v=0.0): s.longitudinal_velocity=v
''')
w('autoware_control_msgs/__init__.py',''); w('autoware_control_msgs/msg.py','''
import types
class Control:
    def __init__(s):
        s.stamp=None; s.control_time=None
        s.lateral=types.SimpleNamespace(stamp=None,control_time=None,steering_tire_angle=0.0,steering_tire_rotation_rate=0.0)
        s.longitudinal=types.SimpleNamespace(stamp=None,control_time=None,velocity=0.0,acceleration=0.0,is_defined_acceleration=False)
''')
print('fake built')

"""离线仿真：不需要 ROS，用假的 rclpy 跑 hybrid_mode_manager + route_navigator 的交接流程（8 个场景）。
运行：python3 offline_sim/build_fake.py && python3 offline_sim/run_sim.py
默认车停在南门外交接点 (29.000, -76.049)；场景 9 用真实的 hybrid_switch_zones.yaml 和 stations.yaml。
改了 hybrid_mode_manager / route_navigator 之后跑一下，确认交权规则没被改坏。"""
import sys, os, json, types, importlib, math
HERE=os.path.dirname(os.path.abspath(__file__))
PKG=sys.argv[1] if len(sys.argv)>1 else os.path.join(os.path.dirname(HERE),'indoor_navigation')
sys.path[:0]=[os.path.join(HERE,'fake'),PKG]
from sim import BUS, CLOCK
NS=types.SimpleNamespace
STATIONS=os.path.join(PKG,'config','stations.yaml')

class World:   # 默认停在南门外交接点（hybrid_switch_zones.yaml 的 handover）
    def __init__(s): s.x,s.y=29.000,-76.049; s.v=0.0; s.aw_v=0.0; s.in_v=0.0; s.health='OK'; s.vr_on=True; s.odom_on=True; s.health_on=True
W=World()

def fresh(mgr_params=None, nav_params=None):
    BUS.__init__(); CLOCK[0]=0.0; W.__init__()
    BUS.params['hybrid_mode_manager']=dict(mgr_params or {})
    BUS.params['route_navigator']=dict({'stations_file':STATIONS}, **(nav_params or {}))
    for m in [k for k in sys.modules if k.startswith('indoor_navigation')]: del sys.modules[m]
    hm=importlib.import_module('indoor_navigation.hybrid_mode_manager'); rn=importlib.import_module('indoor_navigation.route_navigator')
    return hm.HybridModeManager(), rn.RouteNavigator()

def ctrl(v):
    from autoware_control_msgs.msg import Control
    m=Control(); m.longitudinal.velocity=v; return m

def step(dt=1/30):
    CLOCK[0]+=dt; k=round(CLOCK[0]*30)
    BUS.pub('/hybrid/outdoor/control_cmd',ctrl(W.aw_v)); BUS.pub('/hybrid/indoor/control_cmd',ctrl(W.in_v))
    if W.vr_on:
        from autoware_vehicle_msgs.msg import VelocityReport; BUS.pub('/vehicle/status/velocity_status',VelocityReport(W.v))
    if W.odom_on:
        BUS.pub('/localization/kinematic_state',NS(pose=NS(pose=NS(position=NS(x=W.x,y=W.y))),twist=NS(twist=NS(linear=NS(x=W.v)))))
    if W.health_on and k%6==0:
        BUS.pub('/indoor_localization/health',NS(data=json.dumps({'level':W.health,'speed_factor':{'OK':1.0,'WARN':0.5,'ERROR':0.0}[W.health],'reasons':[]})))
    for t in BUS.timers:
        t[2]+=dt
        if t[2]>=t[0]-1e-9: t[2]=0.0; t[1]()
    out=BUS.last.get('/control/command/control_cmd')
    if out is not None:
        tgt=out.longitudinal.velocity; W.v+= (tgt-W.v)*min(1.0,dt/0.3); W.x+=W.v*dt
def run(sec):
    for _ in range(int(round(sec*30))): step()
def st(): return json.loads(BUS.last['/hybrid_mode_manager/state'].data)
def set_indoor(b):
    from std_srvs.srv import SetBool
    t,cb=BUS.srv['/hybrid_mode_manager/set_indoor']; r=cb(SetBool.Request(b),SetBool.Response()); return r
def goto(station):
    from std_msgs.msg import String
    BUS.pub('/route_navigator/command',String(json.dumps({'cmd':'goto','station':station})))
    return json.loads(BUS.last['/route_navigator/status'].data)
def nav(): return json.loads(BUS.last['/route_navigator/status'].data)
def out_v(): return BUS.last['/control/command/control_cmd'].longitudinal.velocity

ok=True
def check(c,msg):
    global ok; ok&=bool(c); print(('  PASS ' if c else '  FAIL ')+msg)

print('== 1 室外时下发室内站点任务 → 拒绝（原版会接受，Nav2 空转后触发倒车恢复）')
fresh(); run(1)
s=goto('room_home'); check(s['state']=='REJECTED' and 'authority' in s['reason'], f"route_navigator: {s['state']} / {s.get('reason','')[:60]}")
check(len(BUS.actions['navigate_through_poses'].goals)==0,'没有发给 Nav2')

print('== 2 先交权再下任务：室外 → 室内')
r=set_indoor(True); check(r.success,'set_indoor true 接受（未配围栏，给出警告）')
run(0.5); check(st()['phase']=='stopping' and out_v()==0.0,'交接中输出停车')
run(1.0); s=st(); check(s['mode']=='indoor' and s['phase']=='active',f"室内拿到权限 mode={s['mode']} phase={s['phase']}")
s=goto('room_home'); check(s['state']=='NAVIGATING','室内站点任务被接受')
W.in_v=0.3; run(0.5); check(abs(out_v()-0.3)<1e-9,'底盘收到室内命令')

print('== 3 室内行驶中请求出门 → 先停车、取消 Nav2 任务，Autoware 空闲后才交权')
W.aw_v=0.8                                  # Autoware 还挂着一条旧路线想走
set_indoor(False); run(0.5)
check(out_v()==0.0,'切换开始即输出停车')
check(BUS.actions['navigate_through_poses'].cancels==1 and nav()['state'] in ('CANCELING','CANCELED'),f"route_navigator 取消任务（{nav()['state']}）")
W.in_v=0.0; run(2.0); s=st()
check(s['mode']=='indoor' and s['phase']=='stopping' and 'commanding motion' in s['block_reason'],f"Autoware 在输出 0.8 m/s 时不交权：{s['block_reason'][:50]}")
W.aw_v=0.0; run(0.3); s=st(); check(s['mode']=='outdoor' and s['phase']=='active','Autoware 停下后交权给室外')
W.aw_v=0.5; run(0.5); check(abs(out_v()-0.5)<1e-9,'之后 Autoware 的命令正常下发')

print('== 4 车速数据断流 → 不交权')
fresh(); run(1); W.vr_on=False; W.odom_on=False; set_indoor(True); run(3); s=st()
check(s['phase']=='stopping' and s['block_reason']=='vehicle speed unavailable',f"block: {s['block_reason']}")
W.vr_on=True; run(1.5); check(st()['mode']=='indoor','车速恢复后正常交权')

print('== 5 只有定位速度、底盘车速断流 → 用定位速度')
fresh(); run(1); W.vr_on=False; set_indoor(True); run(1.6); s=st(); check(s['mode']=='indoor' and s['speed_source']=='localization','改用定位速度并完成交权')

print('== 6 定位不健康 / 健康数据断流 → 两个方向都不交权')
fresh(); run(1); W.health='ERROR'; run(0.3); set_indoor(True); run(3); check(st()['phase']=='stopping' and 'healthy' in st()['block_reason'],'ERROR 时不进室内')
W.health='OK'; run(1.5); check(st()['mode']=='indoor','恢复后进入室内')
W.health_on=False; set_indoor(False); run(3); check(st()['phase']=='stopping','健康数据断流时也不出室外（NDT 是全程定位）')

print('== 7 配了围栏：位置不对时手动请求被拒绝')
import tempfile; zf=os.path.join(tempfile.gettempdir(),'hybrid_zones_test.yaml'); open(zf,'w').write('indoor_enter_polygon: [[0,0],[10,0],[10,10],[0,10]]\nindoor_exit_polygon: [[-3,-3],[13,-3],[13,13],[-3,13]]\n')
fresh({'switch_zones_file':zf}); W.x,W.y=-8.0,5.0; run(0.5)
r=set_indoor(True); check(not r.success and 'not at the doorway' in r.message,f'离门 8 m 时拒绝：{r.message[:60]}')
W.x=-2.0; run(0.5); r=set_indoor(True); check(r.success,'在门口交接带时接受'); run(1.6); check(st()['mode']=='indoor','交权完成')
W.x,W.y=5.0,5.0; run(0.5); r=set_indoor(False); check(not r.success and 'inside the indoor area' in r.message,'在室内深处请求出门被拒绝')

print('== 8 切换途中撤回请求 → 停稳后把权限还给原控制器')
fresh(); run(1); set_indoor(True); run(0.3); set_indoor(False); run(1.6); s=st(); check(s['mode']=='outdoor' and s['phase']=='active','回到室外')

print('== 9 真实围栏 + 站点：交接点进门，路线经坡道；出门回到交接点')
REAL_ZONES=os.path.join(PKG,'config','hybrid_switch_zones.yaml')
fresh({'switch_zones_file':REAL_ZONES}); run(0.5)
r=set_indoor(True); check(r.success,f'在交接点 (29.0, -76.05) 请求进室内被接受：{r.message[:50]}'); run(1.6)
check(st()['mode']=='indoor','交权完成')
s=goto('room_home'); g=BUS.actions['navigate_through_poses'].goals[-1]
wp=[(round(p.pose.position.x,1),round(p.pose.position.y,2),round(math.degrees(2*math.atan2(p.pose.orientation.z,p.pose.orientation.w))%360,1)) for p in g.poses]
check(s['state']=='NAVIGATING' and wp[0][:2]==(18.2,-76.3) and wp[0][2]==90.0 and wp[1][:2]==(21.2,-74.68) and wp[1][2]==2.6,
      f'先到坡道口（朝北）再上坡道（朝东）：{wp[:3]}')
W.x,W.y=26.9,-68.8; run(0.5); r=set_indoor(False); check(not r.success,'停在门内（室内区域里）请求出门被拒绝')
W.x,W.y=29.0,-76.049; run(0.5); r=set_indoor(False); check(r.success,'回到交接点后请求出门被接受'); run(1.6)
check(st()['mode']=='outdoor','交还室外')

print('\nALL OK' if ok else '\nSOME FAILED')

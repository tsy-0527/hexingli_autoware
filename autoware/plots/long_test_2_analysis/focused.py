import pickle,json,csv,sqlite3,re,collections,os
from pathlib import Path
os.environ['MPLCONFIGDIR']='/tmp/long_test2_mpl'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import analyze as a
OUT=Path(__file__).resolve().parent
d=pickle.load(open(OUT/'full_frames.pkl','rb'));t0=d['t0_ns']
c=sqlite3.connect('file:/home/tang/autoware_bag/long_test_2/long_test_2_0.db3?mode=ro',uri=True)
lookup={name:(tid,get_message(typ)) for tid,name,typ in c.execute('select id,name,type from topics') if name=='/diagnostics'}
rows=[];diag_times=collections.defaultdict(list)
tid,cls=lookup['/diagnostics']
for ts,blob in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
 m=deserialize_message(blob,cls);t=(ts-t0)/1e9
 for s in m.status:
  vals={v.key:v.value for v in s.values}
  if s.name in ['/adapi/node/localization: state','/adapi/node/routing: state','controller_node_exe: control_state']:
   diag_times[s.name].append([t,int(vals.get('control_state',0))])
  if s.name=='multi_object_tracker: Tracker Timing Diagnostics':
   rows.append([t,int.from_bytes(s.level,'little'),float(vals.get('Detection delay (s)','nan')),float(vals.get('Extrapolation time (s)','nan'))])
a.save_csv('tracker_delay.csv',['t','level','detection_delay','extrapolation'],rows)
summary={'tracker_delay':a.stats([r[2] for r in rows]),'tracker_extrapolation':a.stats([r[3] for r in rows])}
for name,rr in diag_times.items():
 x=np.array(rr);summary[name]={'dt':a.stats(np.diff(x[:,0])),'n':len(x),'dt_over1':int((np.diff(x[:,0])>1).sum())}
 a.save_csv(name.replace('/','_').replace(':','')+'.csv',['t','control_state_if_applicable'],rr)
# Extract time-stamped log records including continuation lines.
records=[]
for p in Path('/home/tang/.ros/log').glob('*179326196*.log'):
 current=None
 for line in p.read_text(errors='replace').splitlines():
  m=re.search(r'\[(179326\d+\.\d+)\]',line)
  if m:
   current=None;t=float(m[1])-t0/1e9
   if 0<=t<=256.224:current={'t':t,'file':str(p),'text':line};records.append(current)
  elif current:current['text']+='\n'+line
records.sort(key=lambda r:r['t']);(OUT/'runtime_logs.json').write_text(json.dumps(records,indent=2))
mrm=[];start=None
for r in records:
 if 'MRM State changed: NORMAL -> MRM_OPERATING' in r['text']:start=r['t']
 if 'MRM State changed: MRM_OPERATING -> NORMAL' in r['text'] and start is not None:
  reasons=[z['text'].split('reasons:')[-1].strip() for z in records if start-.2<z['t']<=start and 'following reasons' in z['text']]
  mrm.append([start,r['t'],r['t']-start,'; '.join(reasons)])
a.save_csv('mrm_events.csv',['start','end','duration','reason_logged'],mrm);summary['mrm_events']=mrm
# Geometry evidence.
def near(key,t):
 rr=d[key]
 if isinstance(rr,list):return rr[np.argmin([abs(v['t']-t) for v in rr])]
 return rr[np.argmin(abs(rr[:,0]-t))]
def local(points,e):
 ang=e[5];R=np.array([[np.cos(ang),-np.sin(ang)],[np.sin(ang),np.cos(ang)]])
 return (np.atleast_2d(points)[:,:2]-e[2:4])@R
snapshots=[];fig,axs=plt.subplots(2,3,figsize=(15,12))
for t,ax in zip([192.5,194.5,196,219.5,219.95,220.25],axs.flat):
 e=near('ego',t);obj=near('objects',t);tr=near('trajectory',t)['p'];path=near('path',t)['p']
 body=np.array([[-.1885,-.286],[.6635,-.286],[.6635,.286],[-.1885,.286],[-.1885,-.286]])
 trlocal=local(tr,e);plocal=local(path,e)
 ax.plot(plocal[:,1],plocal[:,0],color='blue',label='Behavior path')
 ax.plot(trlocal[:,1],trlocal[:,0],color='red',label='Trajectory')
 zero=trlocal[tr[:,3]<.01]
 ax.scatter(zero[:,1],zero[:,0],c='black',marker='x',s=20,label='Zero velocity')
 ax.fill([-.286,.286,.286,-.286],[-.1885,-.1885,.6635,.6635],color='green')
 relevant=[]
 for oid,o in obj['objects'].items():
  q=local(o['poly'],e);qp=MplPath(np.vstack([q,q[0]]))
  if len(q)<3:continue
  if q[:,0].max()>=-2 and q[:,0].min()<=8 and q[:,1].max()>=-4 and q[:,1].min()<=4:
   ax.fill(q[:,1],q[:,0],alpha=.2,color='purple');cent=local(o['xy'],e)[0]
   ax.text(cent[1],cent[0],oid[:4],fontsize=7)
   intersects=qp.intersects_path(MplPath(body),filled=True);dist=0.0 if intersects else float(min(a.project(q,body)[2].min(),a.project(body,np.vstack([q,q[0]]))[2].min()));s,lat,_=a.project(o['poly'],tr);se=a.project(e[2:4],tr)[0][0]
   relevant.append({'id':oid,'local_center':cent.tolist(),'forward_range':[float(q[:,0].min()),float(q[:,0].max())],'lateral_range':[float(q[:,1].min()),float(q[:,1].max())],'distance_to_body':dist,'intersects_body':bool(intersects),'path_s_range':[float(s.min()-se),float(s.max()-se)],'path_lat_range':[float(lat.min()),float(lat.max())]})
 ax.set_xlim(4,-4);ax.set_ylim(-2,8);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_title(f't={t:.2f}s / speed={e[9]:.2f} m/s');ax.set_xlabel('Left (m)');ax.set_ylabel('Forward (m)');ax.legend(fontsize=7)
 snapshots.append({'t':t,'object_time':obj['t'],'relevant':relevant})
fig.tight_layout();fig.savefig(OUT/'obstacle_snapshots.png',dpi=150);plt.close(fig)
(OUT/'obstacle_snapshots.json').write_text(json.dumps(snapshots,indent=2))
# Show a fixed UUID across consecutive frames, using the same ego frame for both.
t=128.755878035;ii=np.argmin([abs(r['t']-t) for r in d['objects']]);oid='ba180200000000004019dddff71dae35';rr=d['objects'][ii-1:ii+1];e=near('ego',t)
fig,axs=plt.subplots(1,2,figsize=(13,5))
for r,color in zip(rr,['royalblue','orangered']):
 o=r['objects'][oid];q=local(o['poly'],e);cent=local(o['xy'],e)[0];axs[0].fill(q[:,1],q[:,0],alpha=.2,color=color,label=f"t={r['t']:.3f}s");axs[0].plot(cent[1],cent[0],'o',color=color)
axs[0].plot(0,0,'s',color='green');axs[0].set_aspect('equal');axs[0].invert_xaxis();axs[0].legend();axs[0].grid(alpha=.3);axs[0].set_xlabel('Left (m)');axs[0].set_ylabel('Forward (m)');axs[0].set_title('Same UUID: 3.93 m jump in 0.090 s')
frames=[]
for r in d['objects']:
 if 122<=r['t']<=140 and oid in r['objects']:
  o=r['objects'][oid];q=local(o['xy'],e)[0];frames.append([r['t'],*q])
f=np.array(frames);axs[1].plot(f[:,0],f[:,1],'.-',label='Forward in fixed frame');axs[1].plot(f[:,0],f[:,2],'.-',label='Left in fixed frame');axs[1].legend();axs[1].grid(alpha=.3);axs[1].set_xlabel('Bag time (s)');axs[1].set_ylabel('Position (m)');axs[1].set_title('Object ba1802: repeated geometry / association jumps')
fig.tight_layout();fig.savefig(OUT/'perception_jump.png',dpi=150);plt.close(fig)
labels=collections.Counter(o['label'] for r in d['objects'] for o in r['objects'].values());summary['labels']=dict(labels)
summary['object_samples']=sum(labels.values());summary['zero_reported_speed']=int(sum(np.linalg.norm(o['v'])<1e-6 for r in d['objects'] for o in r['objects'].values()))
(OUT/'focused_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=1))
for r in snapshots:
 print('SNAP',r['t'], sorted(r['relevant'],key=lambda q:q['distance_to_body'])[:2])

import pickle,json,datetime,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
d=pickle.load(open(OUT/'full_frames.pkl','rb'));ex=json.load(open(OUT/'extras.json'))
print('MODES',json.dumps(ex['/system/operation_mode/state'],indent=1))
for k in ['/vehicle/status/control_mode','/vehicle/status/gear_status','/control/command/gear_cmd','/api/routing/state']:print(k,ex[k])
for k,v in ex['diagnostics'].items():
 if any(s[1] for s in v['states']):print('DIAG',k,'counts',v['counts'],'states',v['states'][:12])
def near(key,t):
 a=d[key]
 if isinstance(a,list):return a[np.argmin([abs(r['t']-t) for r in a])]
 return a[np.argmin(abs(a[:,0]-t))]
pr={}
for key in ['path','trajectory','scenario_trajectory']:
 rows=[]
 for r in d[key]:
  e=near('ego',r['t']);p=r['p'];dist=np.linalg.norm(p[:,:2]-e[2:4],axis=1);i=dist.argmin();arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))];zero=np.flatnonzero((p[:,3]<.01)&(arc>=arc[i]-.1))
  rows.append([r['t'],p[i,3],arc[zero[0]]-arc[i] if len(zero) else 999,dist[i]])
 pr[key]=np.array(rows)
np.savez(OUT/'planning_signals.npz',**pr)
print('EVENT samples: t actual cmd follower acc path traj stopdistance steeractual steercommand')
for t in [12,13,13.5,14,15,16,121,122,123,124,126,130,131,132,133,134,135,136,137,140,144,145,193,194,195,196,197,209,210,211,212,213,219,220,221,222,223,237,238,239,240]:
 v=near('vehicle_velocity',t);c=near('control',t);f=near('follower',t);p=pr['path'][np.argmin(abs(pr['path'][:,0]-t))];tr=pr['trajectory'][np.argmin(abs(pr['trajectory'][:,0]-t))]
 print(t,*(round(float(x),3) for x in [v[2],c[4],f[4],c[5],p[1],tr[1],tr[2],near('steering',t)[2],c[2]]))
fig,axs=plt.subplots(4,1,figsize=(17,12),sharex=True)
for key,col,lab in [('vehicle_velocity',2,'Chassis'),('control',4,'Gate command'),('follower',4,'Follower')]:axs[0].plot(d[key][:,0],d[key][:,col],label=lab,lw=1)
for k,v in pr.items():axs[1].plot(v[:,0],v[:,1],label=k,lw=1)
axs[2].plot(d['control'][:,0],np.degrees(d['control'][:,2]),label='Command')
axs[2].plot(d['steering'][:,0],np.degrees(d['steering'][:,2]),label='Actual',alpha=.65)
axs[3].plot([r['t'] for r in d['objects']],[len(r['objects']) for r in d['objects']],label='Object count')
for ax,label in zip(axs,['Speed (m/s)','Nearest path/trajectory v (m/s)','Steering (deg)','Objects']):ax.set_ylabel(label);ax.grid(alpha=.3);ax.legend(loc='upper right');ax.set_xlim(0,256.23)
axs[-1].set_xlabel('Seconds from 2026-10-29 16:30:54.479 UTC+08')
fig.tight_layout();fig.savefig(OUT/'overview.png',dpi=150)
fig,axs=plt.subplots(3,2,figsize=(17,11))
for ax,(lo,hi) in zip(axs.flat,[(10,18),(120,147),(192,199),(208,214),(218,224),(236,243)]):
 for key,col,lab in [('vehicle_velocity',2,'Chassis'),('control',4,'Gate cmd'),('follower',4,'Follower')]:a=d[key];mask=(a[:,0]>=lo)&(a[:,0]<=hi);ax.plot(a[mask,0],a[mask,col],label=lab)
 for k in ['path','trajectory']:a=pr[k];mask=(a[:,0]>=lo)&(a[:,0]<=hi);ax.plot(a[mask,0],a[mask,1],label=k,ls='--',alpha=.7)
 ax.set_xlim(lo,hi);ax.set_ylim(-.1,1.65);ax.grid(alpha=.3);ax.legend(fontsize=8);ax.set_xlabel('Bag time (s)');ax.set_ylabel('m/s')
fig.tight_layout();fig.savefig(OUT/'stop_details.png',dpi=150)

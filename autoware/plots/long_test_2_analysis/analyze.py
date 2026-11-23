#!/usr/bin/env python3
"""Full-frame, read-only continuity audit. Source ROS Humble and install first."""
import csv
import json
import math
import os
import pickle
import sqlite3
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR','/tmp/tracking1028_matplotlib')
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OUT=Path(__file__).resolve().parent
BAG=Path('/home/tang/autoware_bag/long_test_2/long_test_2_0.db3')
TOPICS={
 'ego':'/localization/kinematic_state',
 'gnss':'/sensing/gnss/pose_with_covariance',
 'pose_estimator':'/localization/pose_estimator/pose_with_covariance',
 'twist_estimator':'/localization/twist_estimator/twist_with_covariance',
 'objects':'/perception/object_recognition/objects',
 'path':'/planning/scenario_planning/lane_driving/behavior_planning/path',
 'trajectory':'/planning/trajectory',
 'scenario_trajectory':'/planning/scenario_planning/trajectory',
 'control':'/control/command/control_cmd',
 'follower':'/control/trajectory_follower/control_cmd',
 'vehicle_velocity':'/vehicle/status/velocity_status',
 'steering':'/vehicle/status/steering_status',
 'imu':'/sensing/imu/imu_data',
}

def yaw(q):
 return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def wrap(a):
 return (a+np.pi)%(2*np.pi)-np.pi

def stamp(s):
 return s.sec*1_000_000_000+s.nanosec

def extract():
 cache=OUT/'full_frames.pkl'
 if cache.exists():
  with cache.open('rb') as f:return pickle.load(f)
 c=sqlite3.connect(f'file:{BAG}?mode=ro',uri=True)
 t0=c.execute('SELECT min(timestamp) FROM messages').fetchone()[0]
 lookup={}
 for tid,name,typ in c.execute('SELECT id,name,type FROM topics'):
  if name in TOPICS.values():lookup[tid]=(next(k for k,v in TOPICS.items() if v==name),get_message(typ))
 data={k:[] for k in TOPICS};data['t0_ns']=t0
 last_print=0
 sql='SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ('+','.join('?' for _ in lookup)+') ORDER BY timestamp'
 for tid,ts,blob in c.execute(sql,tuple(lookup)):
  key,cls=lookup[tid];m=deserialize_message(blob,cls);t=(ts-t0)/1e9
  h=(stamp(m.header.stamp if hasattr(m,'header') else m.stamp)-t0)/1e9
  if key in ('ego','gnss','pose_estimator'):
   p=m.pose.pose;q=p.orientation
   row=[t,h,p.position.x,p.position.y,p.position.z,yaw(q),m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35]]
   if key=='ego':row += [m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]
   data[key].append(row)
  elif key=='twist_estimator':
   data[key].append([t,h,m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z])
  elif key=='imu':
   data[key].append([t,h,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y])
  elif key in ('control','follower'):
   data[key].append([t,h,m.lateral.steering_tire_angle,m.lateral.steering_tire_rotation_rate,m.longitudinal.velocity,m.longitudinal.acceleration])
  elif key=='vehicle_velocity':
   data[key].append([t,h,m.longitudinal_velocity,m.lateral_velocity,m.heading_rate])
  elif key=='steering':data[key].append([t,h,m.steering_tire_angle])
  elif key in ('path','trajectory','scenario_trajectory'):
   assert m.header.frame_id=='map'
   pts=np.array([[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation),p.longitudinal_velocity_mps,
                  getattr(p,'acceleration_mps2',0.),getattr(p,'front_wheel_angle_rad',0.)] for p in m.points])
   data[key].append({'t':t,'h':h,'p':pts})
  elif key=='objects':
   assert m.header.frame_id=='map'
   objects={}
   for o in m.objects:
    p=o.kinematics.initial_pose_with_covariance.pose;angle=yaw(p.orientation)
    if o.shape.type==2 and len(o.shape.footprint.points)>=3:
     poly=np.array([[p.x,p.y] for p in o.shape.footprint.points])
    elif o.shape.type==1:
     theta=np.linspace(0,2*np.pi,24,endpoint=False);poly=np.column_stack([np.cos(theta),np.sin(theta)])*o.shape.dimensions.x/2
    else:poly=np.array([[1,1],[1,-1],[-1,-1],[-1,1]])*[o.shape.dimensions.x/2,o.shape.dimensions.y/2]
    R=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    objects[bytes(o.object_id.uuid).hex()]={
     'xy':np.array([p.position.x,p.position.y]),'yaw':angle,'poly':poly@R.T+[p.position.x,p.position.y],
     'v':np.array([o.kinematics.initial_twist_with_covariance.twist.linear.x,o.kinematics.initial_twist_with_covariance.twist.linear.y])@R.T,
     'label':max(o.classification,key=lambda c:c.probability).label if o.classification else -1,
     'probability':o.existence_probability}
   data[key].append({'t':t,'h':h,'objects':objects})
  if t-last_print>60:print(f'extracted {t:.1f}s',flush=True);last_print=t
 c.close()
 for k in TOPICS:
  if k not in ('objects','path','trajectory','scenario_trajectory'):data[k]=np.array(data[k])
 with cache.open('wb') as f:pickle.dump(data,f)
 print('full frame counts', {k:len(data[k]) for k in TOPICS},flush=True)
 return data

def stats(v):
 v=np.asarray(v);v=v[np.isfinite(v)]
 if not len(v):return {}
 return {k:float(x) for k,x in zip(['median','p95','p99','max'],np.percentile(v,[50,95,99,100]))}

def save_csv(name,fields,rows):
 with (OUT/name).open('w') as f:
  w=csv.writer(f);w.writerow(fields);w.writerows(rows)

def timing(d):
 result={}
 for key in TOPICS:
  rows=d[key]
  if isinstance(rows,np.ndarray):t,h=rows[:,0],rows[:,1]
  else:t,h=np.array([[r['t'],r['h']] for r in rows]).T
  dt=np.diff(h);rt=np.diff(t)
  result[key]={'n':len(t),'header_dt':stats(dt),'receive_dt':stats(rt),
               'nonpositive_header_dt':int((dt<=0).sum()),'age_s':stats(t-h),
               'largest_receive_gaps':[[float(t[i+1]),float(rt[i])] for i in np.argsort(rt)[-5:][::-1]]}
 return result

def localization(d):
 summary={}
 for key in ('ego','gnss','pose_estimator'):
  r=d[key];dt=np.diff(r[:,1]);good=(dt>0)&(dt<.3)
  dp=np.diff(r[:,2:4],axis=0);dy=wrap(np.diff(r[:,5]));angle=(r[1:,5]+r[:-1,5])/2
  if key=='ego':
   vel=(r[1:,9:11]+r[:-1,9:11])/2
   expected=np.column_stack([vel[:,0]*np.cos(angle)-vel[:,1]*np.sin(angle),vel[:,0]*np.sin(angle)+vel[:,1]*np.cos(angle)])*dt[:,None]
   yaw_error=wrap(dy-(r[1:,11]+r[:-1,11])/2*dt)
  else:
   ego=d['ego'];v=np.interp((r[1:,1]+r[:-1,1])/2,ego[:,1],ego[:,9])
   expected=np.column_stack([v*np.cos(angle),v*np.sin(angle)])*dt[:,None]
   wz=np.interp((r[1:,1]+r[:-1,1])/2,ego[:,1],ego[:,11]);yaw_error=wrap(dy-wz*dt)
  residual=np.linalg.norm(dp-expected,axis=1)
  values=np.column_stack([r[1:,0],r[1:,1],dt,np.linalg.norm(dp,axis=1),residual,np.degrees(abs(dy)),np.degrees(abs(yaw_error)),np.diff(r[:,4])])
  save_csv(key+'_steps.csv',['t','header_t','dt','position_step_m','motion_residual_m','yaw_step_deg','yaw_residual_deg','z_step_m'],values)
  summary[key]={'position_step_m':stats(values[good,3]),'motion_residual_m':stats(residual[good]),
                'yaw_step_deg':stats(values[good,5]),'yaw_residual_deg':stats(values[good,6]),
                'z_step_m':stats(abs(values[good,7])),
                'residual_gt_0_1m':int(((residual>.1)&good).sum()),
                'residual_gt_0_2m':int(((residual>.2)&good).sum()),
                'yaw_residual_gt_2deg':int(((values[:,6]>2)&good).sum()),
                'largest_residuals':values[np.argsort(np.where(good,residual,-1))[-10:][::-1]].tolist()}
 return summary

def nearest_ego(d,t):
 r=d['ego'];idx=np.searchsorted(r[:,0],t)
 return r[min(range(max(0,idx-1),min(len(r),idx+1)),key=lambda i:abs(r[i,0]-t))]

def perception(d):
 rows=[];frame_rows=[];prevprev=None
 for prev,cur in zip(d['objects'],d['objects'][1:]):
  dt=cur['h']-prev['h'];old,new=prev['objects'],cur['objects'];common=old.keys()&new.keys()
  ego=nearest_ego(d,cur['t']);a=ego[5];R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
  frame_rows.append([cur['t'],cur['h'],dt,len(new),len(new.keys()-old.keys()),len(old.keys()-new.keys()),len(common)])
  if not (0<dt<.3):prevprev=prev;continue
  for oid in common:
   o,n=old[oid],new[oid];delta=n['xy']-o['xy'];step=np.linalg.norm(delta)
   residual=np.linalg.norm(delta-o['v']*dt)
   observed_residual=float('nan')
   if prevprev and oid in prevprev['objects']:
    dt0=prev['h']-prevprev['h']
    if 0<dt0<.3:observed_residual=np.linalg.norm(delta-(o['xy']-prevprev['objects'][oid]['xy'])*dt/dt0)
   # Translation-invariant shape change, using distance to polygon edges.
   p0=o['poly']-o['xy'];p1=n['poly']-n['xy']
   shape_change=max(np.max(project(p0,np.vstack([p1,p1[0]]))[2]),np.max(project(p1,np.vstack([p0,p0[0]]))[2]))
   local=(n['xy']-ego[2:4])@R
   poly_local=(n['poly']-ego[2:4])@R
   near=bool(poly_local[:,0].max()>=0 and poly_local[:,0].min()<=6 and poly_local[:,1].max()>=-1.5 and poly_local[:,1].min()<=1.5)
   rows.append([cur['t'],cur['h'],oid,dt,step,residual,observed_residual,shape_change,np.linalg.norm(n['xy']-ego[2:4]),*local,n['label'],int(near),int(n['label']!=o['label'])])
  prevprev=prev
 save_csv('object_steps.csv',['t','header_t','id','dt','position_step_m','reported_velocity_residual_m','observed_velocity_residual_m','shape_change_m','range_m','forward_m','left_m','label','near','class_change'],rows)
 save_csv('object_frames.csv',['t','header_t','dt','n','births','deaths','retained'],frame_rows)
 summary={}
 for key,selection in [('all',rows),('near',[r for r in rows if r[12]])]:
  summary[key]={'pairs':len(selection)}
  for index,name in [(4,'position_step_m'),(6,'observed_velocity_residual_m'),(7,'shape_change_m')]:
   val=np.array([r[index] for r in selection]);summary[key][name]=stats(val)
   summary[key][name+'_gt_0_5m']=int((val>.5).sum())
   summary[key][name+'_top']=sorted([r for r in selection if np.isfinite(r[index])],key=lambda r:r[index],reverse=True)[:10]
 summary['frame_count']=stats([r[3] for r in frame_rows]);summary['births']=stats([r[4] for r in frame_rows])
 return summary

def project(points,line):
 points=np.atleast_2d(points)[:,:2];a=line[:-1,:2];v=np.diff(line[:,:2],axis=0);length=np.linalg.norm(v,axis=1)
 delta=points[:,None,:]-a
 frac=np.clip(np.sum(delta*v,axis=2)/np.maximum(length**2,1e-12),0,1)
 err=delta-frac[:,:,None]*v;idx=np.argmin(np.sum(err**2,axis=2),axis=1);k=np.arange(len(points))
 s=np.r_[0,np.cumsum(length)][idx]+frac[k,idx]*length[idx]
 lat=(v[idx,0]*err[k,idx,1]-v[idx,1]*err[k,idx,0])/np.maximum(length[idx],1e-12)
 return s,lat,np.linalg.norm(err[k,idx],axis=1)

def arc(p):return np.r_[0,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]

def interp(p,s,query):
 result=np.column_stack([np.interp(query,s,p[:,i]) for i in range(p.shape[1])])
 result[:,2]=np.interp(query,s,np.unwrap(p[:,2]));return result

def planning(d):
 summary={}
 for key in ('path','trajectory','scenario_trajectory'):
  rows=[]
  for prev,cur in zip(d[key],d[key][1:]):
   p0,p1=prev['p'],cur['p'];dt=cur['h']-prev['h']
   if min(len(p0),len(p1))<3:continue
   ego=nearest_ego(d,cur['t']);s0=arc(p0);s1=arc(p1)
   se0=project(ego[2:4],p0)[0][0];se1=project(ego[2:4],p1)[0][0]
   local=(s1>=se1-3)&(s1<=se1+25)
   if local.sum()<3:continue
   p1local=p1[local];s1local=arc(p1local)
   row=[cur['t'],cur['h'],dt]
   for lo,hi in [(0,5),(5,10),(10,20)]:
    query=se0+np.arange(lo,hi+.01,.25);query=query[(query>=0)&(query<=s0[-1]-.25)]
    if not len(query):row += [float('nan')]*5+[0];continue
    ref=interp(p0,s0,query);sq,lat,dist=project(ref,p1local)
    valid=(sq>.15)&(sq<s1local[-1]-.15)
    nxt=interp(p1local,s1local,sq)
    valid &= abs(wrap(nxt[:,2]-ref[:,2]))<np.pi/2
    if not valid.any():row += [float('nan')]*5+[0];continue
    row += [float(np.max(abs(lat[valid]))),float(np.percentile(abs(lat[valid]),95)),
            float(np.max(np.degrees(abs(wrap(nxt[valid,2]-ref[valid,2]))))),
            float(np.max(abs(nxt[valid,3]-ref[valid,3]))),float(np.max(abs(nxt[valid,4]-ref[valid,4]))),int(valid.sum())]
   rows.append(row)
  fields=['t','header_t','dt']
  for band in ['0_5m','5_10m','10_20m']:fields += [band+'_'+v for v in ['lat_max','lat_p95','yaw_max_deg','speed_max_delta','accel_max_delta','samples']]
  save_csv(key+'_steps.csv',fields,rows)
  v=np.array(rows);summary[key]={}
  for idx in [3,4,5,6,9,15]:
   vv=v[:,idx];valid=np.isfinite(vv)
   summary[key][fields[idx]]=stats(vv)
   summary[key][fields[idx]+'_top']=v[np.argsort(np.where(valid,vv,-1))[-12:][::-1]].tolist()
  summary[key]['near_lateral_gt_0_2m']=int((v[:,3]>.2).sum())
  summary[key]['near_yaw_gt_10deg']=int((v[:,5]>10).sum())
  print('planned',key,flush=True)
 return summary

def main():
 d=extract();summary={'timing':timing(d),'localization':localization(d)}
 (OUT/'summary.json').write_text(json.dumps(summary,indent=2))
 print('localization complete',flush=True)
 summary['planning']=planning(d);(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
 print('starting perception',flush=True)
 summary['perception']=perception(d);(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
 print('complete',flush=True)

if __name__=='__main__':main()

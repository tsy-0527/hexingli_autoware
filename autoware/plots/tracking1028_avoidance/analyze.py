#!/usr/bin/env python3
"""Read-only offline extraction for tracking1028; source ROS and install first.

Uses bag receive timestamps; objects/paths sampled at <= 5 Hz, odometry kept in full.
Map lane membership is geometric, not the planner's internal lane selection.
"""
import csv
import json
import math
import os
import pickle
import sqlite3
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/tracking1028_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as PolygonPath
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OUT = Path(__file__).resolve().parent
BAG = Path('/home/tang/autoware_bag/tracking1028/tracking1028_0.db3')
MAP = Path('/home/tang/autoware_map/map_task1/lanelet2_map.osm')
TOPICS = {
    'ego': '/localization/kinematic_state',
    'objects': '/perception/object_recognition/objects',
    'path': '/planning/scenario_planning/lane_driving/behavior_planning/path',
    'trajectory': '/planning/trajectory',
    'mode': '/system/operation_mode/state',
    'route': '/planning/mission_planning/route',
}

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def xy(p):
    return [p.x, p.y]

def object_data(o):
    p = o.kinematics.initial_pose_with_covariance.pose
    a = yaw(p.orientation)
    shape = o.shape
    if shape.type == 2 and len(shape.footprint.points) >= 3:
        poly = np.array([xy(v) for v in shape.footprint.points])
    elif shape.type == 1:
        angle = np.linspace(0, 2*np.pi, 24)
        poly = np.column_stack([np.cos(angle), np.sin(angle)]) * shape.dimensions.x / 2
    else:
        poly = np.array([[1,1], [1,-1], [-1,-1], [-1,1]]) * [shape.dimensions.x/2, shape.dimensions.y/2]
    rot = np.array([[math.cos(a),-math.sin(a)], [math.sin(a),math.cos(a)]])
    return {'id': bytes(o.object_id.uuid).hex(), 'xy': xy(p.position),
            'poly': poly @ rot.T + xy(p.position),
            'label': max(o.classification, key=lambda x:x.probability).label if o.classification else -1,
            'speed': o.kinematics.initial_twist_with_covariance.twist.linear.x}

def extract():
    cache = OUT / 'extracted.pkl'
    if cache.exists():
        with cache.open('rb') as f:
            return pickle.load(f)
    c = sqlite3.connect(f'file:{BAG}?mode=ro', uri=True)
    t0 = c.execute('SELECT min(timestamp) FROM messages').fetchone()[0]
    result = {'t0_ns': t0}
    for key, topic in TOPICS.items():
        tid, typ = c.execute('SELECT id,type FROM topics WHERE name=?', (topic,)).fetchone()
        cls = get_message(typ)
        rows = []
        last = -float('inf')
        for ts, blob in c.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,)):
            t = (ts-t0)/1e9
            if key in ('objects','path','trajectory') and t-last < .195:
                continue
            last = t
            m = deserialize_message(blob, cls)
            if key == 'ego':
                rows.append([t, *xy(m.pose.pose.position), yaw(m.pose.pose.orientation), m.twist.twist.linear.x])
            elif key == 'objects':
                assert m.header.frame_id == 'map'
                rows.append({'t':t, 'objects':[object_data(o) for o in m.objects]})
            elif key in ('path','trajectory'):
                assert m.header.frame_id == 'map'
                item = {'t':t, 'points':np.array([[*xy(p.pose.position),p.longitudinal_velocity_mps] for p in m.points])}
                if key == 'path':
                    item.update(left=np.array([xy(p) for p in m.left_bound]), right=np.array([xy(p) for p in m.right_bound]))
                rows.append(item)
            elif key == 'mode':
                rows.append([t,m.mode,m.is_autoware_control_enabled,m.is_in_transition])
            elif key == 'route':
                rows.append({'t':t,'preferred':[s.preferred_primitive.id for s in m.segments],
                             'segments':[[p.id for p in s.primitives] for s in m.segments]})
        result[key] = np.array(rows) if key in ('ego','mode') else rows
        print(key, len(rows), flush=True)
    c.close()
    with cache.open('wb') as f:
        pickle.dump(result,f)
    return result

def map_data():
    import lanelet2
    import yaml
    projector = yaml.safe_load((MAP.parent/'map_projector_info.yaml').read_text())
    assert projector['projector_type'] == 'LocalCartesianUTM'
    origin = projector['map_origin']
    lane_map = lanelet2.io.load(str(MAP), lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(origin['latitude'], origin['longitude'])))
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                        lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(lane_map,rules)
    lanes = {}
    for lane in lane_map.laneletLayer:
        left = np.array([[p.x,p.y] for p in lane.leftBound])
        right = np.array([[p.x,p.y] for p in lane.rightBound])
        lanes[lane.id] = {'left':left,'right':right,'poly':np.vstack([left,right[::-1]]),
                         'center':np.array([[p.x,p.y] for p in lane.centerline]),
                         'left_id':lane.leftBound.id,'right_id':lane.rightBound.id,
                         'left_neighbor':graph.left(lane).id if graph.left(lane) else None,
                         'right_neighbor':graph.right(lane).id if graph.right(lane) else None}
    return lanes

def project(points, line):
    """Nearest polyline projection: arc length, signed lateral, distance."""
    points = np.atleast_2d(points)
    a = line[:-1,:2]
    v = np.diff(line[:,:2],axis=0)
    length = np.linalg.norm(v,axis=1)
    d = points[:,None,:]-a
    frac = np.clip(np.sum(d*v,axis=2)/np.maximum(length**2,1e-12),0,1)
    delta = d-frac[:,:,None]*v
    idx = np.argmin(np.sum(delta**2,axis=2),axis=1)
    k = np.arange(len(points))
    lat = (v[idx,0]*delta[k,idx,1]-v[idx,1]*delta[k,idx,0])/np.maximum(length[idx],1e-12)
    s = np.r_[0,np.cumsum(length)][idx]+frac[k,idx]*length[idx]
    return s, lat, np.linalg.norm(delta[k,idx],axis=1)

def nearest(rows,t):
    ts = [r['t'] for r in rows]
    idx = np.searchsorted(ts,t)
    idx = min(range(max(0,idx-1),min(len(rows),idx+1)),key=lambda i:abs(ts[i]-t))
    return rows[idx]

def snapshot(d, lanes, t, ax=None):
    e=d['ego'][np.argmin(abs(d['ego'][:,0]-t))]
    p=nearest(d['path'],t)
    tr=nearest(d['trajectory'],t)
    objects=nearest(d['objects'],t)
    angle=e[3]
    rot=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    def local(points):
        return (np.atleast_2d(points)[:,:2]-e[1:3])@rot
    se=project(e[1:3],p['points'])[0][0]
    relevant=[]
    for obj in objects['objects']:
        s,lat,_=project(obj['poly'],p['points'])
        s=s-se
        if s.max()<0 or s.min()>5 or lat.min()>1.0 or lat.max()<-1.0:
            continue
        relevant.append({'id':obj['id'],'label':int(obj['label']), 's_min':round(float(s.min()),3),
                         's_max':round(float(s.max()),3),'l_min':round(float(lat.min()),3),
                         'l_max':round(float(lat.max()),3),'speed':float(obj['speed']),
                         'ego_xy':local(obj['xy'])[0].round(3).tolist()})
    v=[]
    for key in ('path','trajectory'):
        pp=nearest(d[key],t)['points']
        ii=int(np.argmin(np.linalg.norm(pp[:,:2]-e[1:3],axis=1)))
        v.append(float(pp[ii,2]))
    if ax is not None:
        # Body-frame display: horizontal left-positive, vertical forward-positive.
        for lid,lane in lanes.items():
            for side in ('left','right'):
                pts=local(lane[side]);ax.plot(pts[:,1],pts[:,0],color='.8',lw=.6)
        for side in ('left','right'):
            pts=local(p[side]);ax.plot(pts[:,1],pts[:,0],'k--',lw=1,label=side+' bound')
        for key,color in [('path','royalblue'),('trajectory','orangered')]:
            pts=local(nearest(d[key],t)['points']);ax.plot(pts[:,1],pts[:,0],color=color,lw=1.5,label=key)
        # Yunle: width .572, front .6635 from rear axle, rear .1885.
        ax.fill([-.286,.286,.286,-.286],[-.1885,-.1885,.6635,.6635],color='green',alpha=.7)
        for obj in objects['objects']:
            pts=local(obj['poly'])
            if np.any((pts[:,0]>-2)&(pts[:,0]<9)&(abs(pts[:,1])<4)):
                ax.fill(pts[:,1],pts[:,0],color='purple',alpha=.35)
                cent=local(obj['xy'])[0]
                ax.text(cent[1],cent[0],str(obj['label']),fontsize=7)
        ax.set_xlim(3,-3);ax.set_ylim(-1,7);ax.set_aspect('equal');ax.grid(alpha=.15)
        ax.set_title(f't={t:.1f}s, v={e[4]:.2f}, path/traj={v[0]:.2f}/{v[1]:.2f}',fontsize=9)
        ax.set_xlabel('Lateral (m, left positive)');ax.set_ylabel('Forward (m)')
    return {'t':t,'ego':e.tolist(),'path_speed':v[0],'trajectory_speed':v[1],
            'objects':sorted(relevant,key=lambda o:o['s_min']),
            'path_dt':p['t']-t,'objects_dt':objects['t']-t}

def main():
    d = extract()
    lanes = map_data()
    ego = d['ego']
    membership = {lid:PolygonPath(l['poly']).contains_points(ego[:,1:3]) for lid,l in lanes.items()}
    # Opposite-direction lanes overlap in this map; disambiguate with heading.
    for lid,flags in membership.items():
        line=lanes[lid]['center']
        for i in np.flatnonzero(flags):
            delta=ego[i,1:3]-line[:-1]
            v=np.diff(line,axis=0)
            f=np.clip(np.sum(delta*v,axis=1)/np.maximum(np.sum(v*v,axis=1),1e-12),0,1)
            j=np.argmin(np.sum((delta-f[:,None]*v)**2,axis=1))
            flags[i] = np.dot(v[j],[np.cos(ego[i,3]),np.sin(ego[i,3])])>0
    print('route',d['route'])
    print('mode',d['mode'].tolist())
    print('lane membership samples', {k:int(v.sum()) for k,v in membership.items() if v.any()})
    stops = []
    flag = np.abs(ego[:,4]) < .05
    edges = np.diff(np.r_[False,flag,False].astype(int))
    for start,end in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
        if ego[end-1,0]-ego[start,0] < 2:
            continue
        row={'start':float(ego[start,0]),'end':float(ego[end-1,0]),
             'xy':np.median(ego[start:end,1:3],axis=0).tolist(),
             'lanes':[k for k,v in membership.items() if v[start:end].mean()>.5]}
        stops.append(row)
    print('stops', json.dumps(stops,indent=2))
    (OUT/'overview.json').write_text(json.dumps({'route':d['route'],'stops':stops,'mode':d['mode'].tolist(),
        'lane_samples':{k:int(v.sum()) for k,v in membership.items()}},indent=2))
    fig,axs=plt.subplots(1,2,figsize=(14,8),gridspec_kw={'width_ratios':[1,1.3]})
    ax=axs[0]
    for lid,l in lanes.items():
        for side in ('left','right'):
            p=l[side]; ax.plot(p[:,0],p[:,1],color='.65',lw=.6)
        p=l['poly'].mean(axis=0); ax.text(*p,str(lid),fontsize=6)
    scatter=ax.scatter(ego[::10,1],ego[::10,2],c=ego[::10,0],s=4,cmap='viridis')
    for i,s in enumerate(stops):
        ax.annotate(f'S{i}',s['xy'],fontsize=8,color='red')
    ax.set_aspect('equal');ax.set_title('Map and actual path (color: elapsed seconds)')
    ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)');fig.colorbar(scatter,ax=ax)
    axs[1].plot(ego[:,0],ego[:,4],lw=.8,label='Actual velocity')
    for m in d['mode']:
        axs[1].axvline(m[0],color='orange',alpha=.3)
    for i,s in enumerate(stops):
        axs[1].axvspan(s['start'],s['end'],alpha=.1,color='red')
        axs[1].text(s['start'],.1+(.15*(i%2)),f'S{i}',fontsize=8)
    axs[1].set_xlabel('Seconds from 2026-10-28 11:55:15.126 +08:00');axs[1].set_ylabel('m/s')
    axs[1].set_title('Velocity and operation-mode messages (orange)');axs[1].grid(alpha=.2)
    fig.tight_layout();fig.savefig(OUT/'overview.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(2,3,figsize=(13,11))
    snapshots=[]
    for s,ax in zip(stops[:6],axs.flat):
        snapshots.append(snapshot(d,lanes,(s['start']+s['end'])/2,ax))
    fig.tight_layout();fig.savefig(OUT/'stop_snapshots.png',dpi=170);plt.close(fig)
    (OUT/'stop_snapshots.json').write_text(json.dumps(snapshots,indent=2))
    print('snapshots',json.dumps(snapshots,indent=2))

if __name__ == '__main__':
    main()

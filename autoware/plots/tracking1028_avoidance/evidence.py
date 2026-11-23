#!/usr/bin/env python3
"""Reproduce focused evidence plots and CSVs. Run after sourcing ROS/install."""
import csv
import datetime as dt
import json
import sqlite3

import analyze as a
import matplotlib.pyplot as plt
import numpy as np

d=a.extract()
lanes=a.map_data()
overview=json.loads((a.OUT/'overview.json').read_text())
route_ids={i for segment in d['route'][0]['segments'] for i in segment}
map_check={lid:{k:lane[k] for k in ('left_neighbor','right_neighbor')}
           for lid,lane in lanes.items() if lid in route_ids}
(a.OUT/'map_neighbors.json').write_text(json.dumps(map_check,indent=2))

c=sqlite3.connect(f'file:{a.BAG}?mode=ro',uri=True)
signals={}
for name in ['/control/command/control_cmd','/control/trajectory_follower/control_cmd',
             '/vehicle/status/velocity_status']:
    tid,typ=c.execute('SELECT id,type FROM topics WHERE name=?',(name,)).fetchone()
    cls=a.get_message(typ)
    values=[]
    for ts,blob in c.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',
                            (tid,int(d['t0_ns']+285e9),int(d['t0_ns']+318e9))):
        m=a.deserialize_message(blob,cls)
        if hasattr(m,'longitudinal'):
            speed=m.longitudinal.velocity
            acc=m.longitudinal.acceleration
        else:
            speed=m.longitudinal_velocity
            acc=float('nan')
        values.append([(ts-d['t0_ns'])/1e9,speed,acc])
    signals[name]=np.array(values)
c.close()
with (a.OUT/'control_samples.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['topic','bag_time_s','velocity_mps','acceleration_mps2'])
    for topic,rows in signals.items():
        w.writerows([[topic,*row] for row in rows])

with (a.OUT/'stops.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['event','start_utc_plus_8','end_utc_plus_8','duration_s','route_lanelets','x','y'])
    for i,s in enumerate(overview['stops']):
        times=[dt.datetime.fromtimestamp(d['t0_ns']/1e9+s[k],dt.timezone(dt.timedelta(hours=8))).isoformat(timespec='milliseconds') for k in ('start','end')]
        w.writerow([i,*times,s['end']-s['start'],','.join(str(k) for k in s['lanes'] if k in route_ids),*s['xy']])

fig,axs=plt.subplots(1,3,figsize=(16,6),gridspec_kw={'width_ratios':[1.2,1,1.2]})
for ax,window,title in [(axs[0],(289,292),'First stop: vehicle slows before command'),
                        (axs[2],(304,309),'Second stop: obstacle present, human intervention')]:
    for name,label,color in [('/control/command/control_cmd','Control command','red'),
                             ('/vehicle/status/velocity_status','Chassis velocity','darkgreen')]:
        points=signals[name];mask=(points[:,0]>=window[0])&(points[:,0]<=window[1])
        ax.plot(points[mask,0],points[mask,1],label=label,color=color,lw=1.4)
    ego=d['ego'];mask=(ego[:,0]>=window[0])&(ego[:,0]<=window[1])
    ax.plot(ego[mask,0],ego[mask,4],label='Localization velocity',color='blue',alpha=.6)
    ax.set_xlim(*window);ax.set_ylim(-.05,1.6);ax.grid(alpha=.2)
    ax.set_title(title,fontsize=10);ax.set_ylabel('Velocity (m/s)')
    ax.set_xlabel('Seconds from 11:55:15.126 +08:00');ax.legend(fontsize=8)

snapshot=a.snapshot(d,lanes,305.2,axs[1])
axs[1].set_title('12:00:20.326: obstacle ahead, path unchanged',fontsize=9)
axs[1].legend(fontsize=7,loc='upper left')
fig.tight_layout();fig.savefig(a.OUT/'key_evidence.png',dpi=170);plt.close(fig)

# Quantify the straight-road event against the Lanelet2 centerline.
event=[]
for t in [305.2,306.0,307.0]:
    item=a.snapshot(d,lanes,t)
    path=a.nearest(d['path'],t)
    ego=np.array(item['ego'])
    center=lanes[950214]['center']
    se=a.project(ego[1:3],center)[0][0]
    ss,lat,_=a.project(path['points'][:,:2],center)
    mask=(ss-se>=0)&(ss-se<=5)
    item['max_path_lateral_from_lane_center_m']=float(np.max(abs(lat[mask])))
    item['ego_to_left_bound_m']=float(a.project(ego[1:3],path['left'])[2][0])
    item['ego_to_right_bound_m']=float(a.project(ego[1:3],path['right'])[2][0])
    event.append(item)
(a.OUT/'key_event.json').write_text(json.dumps(event,indent=2))
print('Saved key_evidence.png, key_event.json, control_samples.csv, stops.csv, map_neighbors.json')

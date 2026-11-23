import sqlite3,json,collections
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
OUT=Path(__file__).resolve().parent
c=sqlite3.connect('file:/home/tang/autoware_bag/long_test_2/long_test_2_0.db3?mode=ro',uri=True)
t0=c.execute('select min(timestamp) from messages').fetchone()[0]
result={};diag=collections.defaultdict(lambda:{'counts':collections.Counter(),'states':[],'previous':None,'values':{}})
for tid,name,typ in c.execute('select id,name,type from topics').fetchall():
 if name not in ['/diagnostics','/system/operation_mode/state','/vehicle/status/control_mode','/vehicle/status/gear_status','/control/command/gear_cmd','/planning/mission_planning/route','/api/routing/state','/tf_static']:continue
 cls=get_message(typ);rows=[];last=None
 for ts,blob in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
  m=deserialize_message(blob,cls);t=(ts-t0)/1e9
  if name=='/diagnostics':
   for s in m.status:
    dd=diag[s.name];dd['counts'][str(s.level)+':'+s.message]+=1
    state=[int.from_bytes(s.level,'little') if isinstance(s.level,bytes) else s.level,s.message]
    vals={v.key:v.value for v in s.values}
    if state!=dd['previous']:
     dd['states'].append([t,*state,vals]);dd['previous']=state
    dd['values']=vals
  else:
   obj=message_to_ordereddict(m)
   obj.pop('stamp',None)
   if name in ['/vehicle/status/control_mode','/vehicle/status/gear_status','/control/command/gear_cmd'] and obj==last:continue
   rows.append([t,obj]);last=obj
 result[name]=rows
 print(name,len(rows),flush=True)
result['diagnostics']=dict(diag)
(OUT/'extras.json').write_text(json.dumps(result,indent=2,default=lambda x:list(x)))
for name,dd in diag.items():
 print(name,dict(dd['counts']),len(dd['states']),flush=True)

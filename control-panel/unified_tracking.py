"""One proper source frame for position and receiver electrical orientation.

Ideal co-located dipoles: M = a (3 u u.T - I) R, a proportional to r^-3.
Free-motion fits identify relative entry gains, not absolute coil sensitivity.
No PCB-to-PCB distance or world translation is inferred from CAD dimensions.
"""
import datetime,json,re
from pathlib import Path
import numpy as np
from motion_calibration import fit_motion,motion_pose
from glove_tracking import align_acquisition_delay,register_to_references


def read_capture(root,name,port,device):
 if not isinstance(name,str) or not re.fullmatch(r'capture-\d{8}-\d{6}-\d{6}\.csv',name):raise ValueError('Invalid recording filename')
 c=json.loads((Path(root)/'captures'/(name+'.json')).read_text())
 if c.get('mask')!=1<<(port-1) or c.get('axes')!=7 or c.get('device_port')!=device:raise ValueError('Recording belongs to a different receiver or device')
 if c.get('firmware') not in ('0.2.4-diagnostics','0.2.5-diagnostics','0.2.6-diagnostics','0.2.7-diagnostics'):raise ValueError('Unsupported recording firmware')
 sp=c['spectrum'][str(port)]
 if sp['clipped']:raise ValueError('Clipped recording')
 v=[t.get('xyz_source_counts') for t in sp['amplitudes']]
 a=np.asarray(v,float)
 if a.shape!=(3,3,2) or not np.isfinite(a).all() or np.min(np.linalg.norm(a,axis=(1,2)))<5:raise ValueError('Weak or missing phase-referenced signal')
 return v


def fit_receivers(root,records,device):
 if not isinstance(records,dict) or not records or any(k not in ('1','2','3','4','5') for k in records):raise ValueError('Select connected receiver recordings')
 result={};folder=Path(root)/'calibrations';folder.mkdir(exist_ok=True)
 for key,record in records.items():
  try:
   port=int(key);files=record.get('files',[]);anchors=record.get('anchors',[])
   if not isinstance(files,list) or not 24<=len(files)<=80 or len(set(files))!=len(files):raise ValueError('Need 24–80 distinct valid captures')
   load=lambda name:read_capture(root,name,port,device)
   values=[load(f) for f in files]
   # Preserve actual same-receiver references. Never borrow another finger's anchors.
   if not isinstance(anchors,list) or len(anchors)>8:raise ValueError('Invalid reference list')
   av=[];dist=[]
   for a in anchors:
    p=np.asarray(a['position_m'],float)
    if p.shape!=(3,) or not np.isfinite(p).all() or not .04<=np.linalg.norm(p)<=.30:raise ValueError('Invalid saved reference')
    av.append(load(a['file']));dist.append(float(np.linalg.norm(p)))
   cal=fit_motion(values,av if len(av)>=2 else None,dist)
   cal.update(receiver_mask=1<<(port-1),files=files,anchors=anchors,published=True,workflow='unified-v1')
   cal['registration']=register_to_references(cal,load)
   name='unified-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'-J'+key+'.json'
   target=folder/name;tmp=target.with_suffix('.tmp');tmp.write_text(json.dumps(cal,indent=2));tmp.replace(target)
   if json.loads(target.read_text())!=cal:raise OSError('Profile verification failed')
   result[key]={'accepted':True,'file':name,'sample_count':cal['sample_count'],'held_out_rms':cal['held_out_rms'],'units':cal['units'],'accuracy_verified':False}
  except (ValueError,KeyError,TypeError,OSError) as e:result[key]={'accepted':False,'reason':str(e),'previous_profile_preserved':True}
 return result


def load_profiles(root,device):
 profiles={};errors=[]
 paths=sorted((Path(root)/'calibrations').glob('*.json'),key=lambda p:p.stat().st_mtime,reverse=True)
 for path in paths:
  if not path.name.startswith(('motion-','unified-')):continue
  try:
   c=json.loads(path.read_text());mask=c['receiver_mask'];port=mask.bit_length()
   if mask not in (1,2,4,8,16) or str(port) in profiles:continue
   load=lambda name:read_capture(root,name,port,device)
   load(c['files'][0]);registration=register_to_references(c,load)
   profiles[str(port)]={'cal':c,'registration':registration,'file':path.name}
  except (ValueError,KeyError,TypeError,OSError) as e:errors.append({'file':path.name,'reason':str(e)})
 return profiles,errors


def coherent_pose(cal,counts,source_rotation,previous=None):
 w=np.asarray(source_rotation,float)
 if w.shape!=(3,3) or not np.isfinite(w).all() or not np.allclose(w.T@w,np.eye(3),atol=1e-5) or np.linalg.det(w)<.999:raise ValueError('Source frame must be a proper rotation')
 corrected,timing=align_acquisition_delay(cal,counts)
 p=motion_pose(cal,corrected,previous_position=previous)
 # One left multiplication changes both vectors and receiver-to-source rotation.
 # No per-finger position reflections and no per-recording rotation zero.
 p['source_position']=(w@np.asarray(p['position'])).tolist()
 p['source_rotation']=(w@np.asarray(p['rotation_receiver_to_transmitter'])).tolist()
 p['metric']=cal.get('metres_per_unit') is not None
 p['automatic_timing']=timing
 return p

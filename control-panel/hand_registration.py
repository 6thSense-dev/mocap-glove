"""User-observed flat right-hand registration; does not change magnetic fits."""
import numpy as np
from scipy.spatial.transform import Rotation

def unit(v):
 v=np.asarray(v,float);n=np.linalg.norm(v)
 if n<1e-8:raise ValueError('Reference direction is missing')
 return v/n

def make_registration(record):
 groups={}
 for r in record['records']:
  if 'pose' in r:groups.setdefault(str(r['port']),[]).append(r['pose'])
 if not all(len(groups.get(p,[]))>=6 for p in ('1','2','5')):raise ValueError('Need at least six valid stationary samples on thumb, index and pinky')
 means={};spread={}
 for p,items in groups.items():
  vectors=np.array([x['position'] for x in items]);directions=vectors/np.linalg.norm(vectors,axis=1)[:,None]
  means[p]=vectors.mean(axis=0);spread[p]=float(np.max(np.degrees(np.arccos(np.clip(directions@unit(means[p]),-1,1)))))
  if spread[p]>8:raise ValueError('Reference moved or signal unstable on J'+p)
 # Select only the forward-pair mirror agreement, not arbitrary axis reflections.
 signs={p:1 for p in means}
 if unit(means['2'])@unit(means['5'])<0:means['5']=-means['5'];signs['5']=-1
 forward=unit(unit(means['2'])+unit(means['5']))
 right=unit(-means['1']-forward*np.dot(-means['1'],forward));up=unit(np.cross(right,forward))
 w=np.stack([right,forward,up]);errors={}
 targets={'1':[-1,0,0],'2':[0,1,0],'5':[0,1,0]}
 for p,target in targets.items():
  errors[p]=float(np.degrees(np.arccos(np.clip(unit(w@means[p])@target,-1,1))))
  if errors[p]>18:raise ValueError('One rigid source-frame rotation cannot explain the flat reference: J'+p)
 # Finger axes: +Y along finger, +Z dorsal. Right-hand thumb points -X.
 desired={p:(Rotation.from_euler('z',90,degrees=True).as_matrix() if p=='1' else np.eye(3)) for p in means}
 mounts={}
 for p,items in groups.items():
  raw=Rotation.from_matrix(np.array([v['rotation_receiver_to_transmitter'] for v in items])).mean().as_matrix()
  mounts[p]=((w@raw).T@desired[p]).tolist()
 return {'version':1,'hand':'right','source_rotation':w.tolist(),'mount_offsets':mounts,'branch_seeds':{p:means[p].tolist() for p in means},'profile_files':record['profiles'],'reference_direction_error_deg':errors,'reference_spread_deg':spread,'reference':'user-confirmed flat right hand: thumb -X, index/pinky +Y, dorsal +Z','accuracy_verified':False,'notes':'Mount offsets define anatomical finger axes from the observed pose. Inter-board distance scale is unchanged; joints are not measured.'}

def apply_registration(reg,port,profile,pose,reference_scale=None):
 if port in reg['profile_files'] and reg['profile_files'][port]!=profile:raise ValueError('Hand reference belongs to an older receiver fit; record a new flat reference')
 w=np.asarray(reg['source_rotation']);offset=np.asarray(reg['mount_offsets'].get(port,np.eye(3)));p=dict(pose);p['mount_registered']=port in reg['mount_offsets']
 # Unanchored receiver lengths use an explicit shared-sensitivity estimate only.
 scale=1 if p['metric'] else reference_scale
 if scale is None:raise ValueError('No display distance scale; relative measurements remain available in unified view')
 p['hub_relative_position_m']=(w@np.asarray(p['position'])*scale).tolist()
 p['display_rotation']=(w@np.asarray(p['rotation_receiver_to_transmitter'])@offset).tolist()
 p['anatomical']=True;p['distance_status']='approximate measured-reference scale' if p['metric'] else 'estimated from J1 sensitivity; not distance-calibrated'
 p['inferred_joints']=True
 return p

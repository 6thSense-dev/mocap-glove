"""Axis registration from known direction moves, without inventing distances."""
import copy
import numpy as np
from scipy.spatial.transform import Rotation

def unit(v):
 v=np.asarray(v,float);n=np.linalg.norm(v)
 if n<1e-10:raise ValueError('No movement measured')
 return v/n

def solve_axis_moves(groups):
 if set(groups)!=set(('start','x','y','z')):raise ValueError('Record a start and all three positive-axis moves')
 means={};rotations={}
 for name,items in groups.items():
  if len(items)<4:raise ValueError('Need four valid samples per pose')
  a=np.array([p['position'] for p in items]);mean=a.mean(axis=0);radius=np.linalg.norm(mean)
  if np.max(np.linalg.norm(a-mean,axis=1))>max(.002,radius*.025):raise ValueError('Receiver moved during '+name+' capture; repeat the check')
  means[name]=mean;rotations[name]=Rotation.from_matrix([p['rotation_receiver_to_transmitter'] for p in items]).mean()
 start=means['start'];directions=[]
 for name in ('x','y','z'):
  delta=means[name]-start
  if np.linalg.norm(delta)<np.linalg.norm(start)*.06:raise ValueError('The '+name.upper()+' move was too small; use about 3–5 cm')
  angle=(rotations['start'].inv()*rotations[name]).magnitude()*180/np.pi
  if angle>12:raise ValueError('Receiver rotated during '+name.upper()+' move; keep its orientation unchanged')
  directions.append(unit(delta))
 d=np.stack(directions,axis=1)
 if np.linalg.cond(d)>3:raise ValueError('Moves do not describe three independent axes')
 u,_,v=np.linalg.svd(d.T);q=u@v
 mirror=1 if np.linalg.det(q)>0 else -1;w=q*mirror
 errors=np.degrees(np.arccos(np.clip(np.diag(q@d),-1,1)))
 if np.max(errors)>15:raise ValueError('One rigid axis mapping does not explain these moves; check hub motion, receiver rotation or field distortion')
 return {'rotation':w.tolist(),'mirror':mirror,'direction_error_deg':errors.tolist(),'deltas':[ (means[n]-start).tolist() for n in ('x','y','z')],'metric_accuracy_verified':False,'status':'fitted to user-directed moves; independent movement validation still required'}

def update_reference(reference,fit):
 result=copy.deepcopy(reference);old=np.asarray(reference['source_rotation']);new=np.asarray(fit['rotation']);result['source_rotation']=new.tolist()
 for port,offset in reference['mount_offsets'].items():
  desired=Rotation.from_euler('z',90 if port=='1' else 0,degrees=True).as_matrix()
  raw=old.T@desired@np.asarray(offset).T
  result['mount_offsets'][port]=((new@raw).T@desired).tolist()
 result['branch_seeds']={p:(np.asarray(v)*fit['mirror']).tolist() for p,v in reference['branch_seeds'].items()}
 result['axis_validation']=fit;result['reference']='user-directed +X/+Y/+Z translations; flat pose retained for finger mounting orientation'
 return result

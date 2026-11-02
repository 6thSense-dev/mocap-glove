"""Per-receiver adaptive filtering; no fabricated poses through bad measurements."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

def alpha(cutoff,dt):return 1/(1+1/(2*math.pi*cutoff*dt))

class PoseStabilizer:
 def __init__(self):self.reset()
 def reset(self):
  self.last_time=None;self.last_position=None;self.raw=None;self.filtered=None;self.velocity=np.zeros(3);self.rotation=None;self.window=[];self.rotation_window=[];self.branch_uncertain=False
 def previous(self,now):
  return self.last_position if self.last_time is not None and 0<now-self.last_time else None
 def apply(self,pose,now):
  dt=now-self.last_time if self.last_time is not None else 0
  fresh=self.last_time is None or dt<=0 or dt>.5
  self.branch_uncertain=self.branch_uncertain or (self.last_time is not None and dt>.5)
  pose['branch_reacquired_after_gap']=self.branch_uncertain
  self.last_position=pose['position'];self.last_time=now
  v=pose.get('hub_relative_position_m');r=pose.get('display_rotation')
  if v is None or r is None:return pose
  x=np.array(v);rotation=Rotation.from_matrix(r)
  if fresh:
   self.window=[x.copy()];self.raw=x.copy();self.filtered=x.copy();self.velocity=np.zeros(3);self.rotation=rotation;self.rotation_window=[rotation]
  else:
   self.window=(self.window+[x.copy()])[-3:];x=np.median(self.window,axis=0)
   a=alpha(1.,dt);self.velocity+=( (x-self.raw)/dt-self.velocity)*a
   cutoff=2.5+12*float(np.linalg.norm(self.velocity))
   self.filtered+=(x-self.filtered)*alpha(cutoff,dt)
   # Select the rotation medoid on SO(3); a single angular spike must not
   # shake the whole fingertip. Never average Euler angles across wrapping.
   self.rotation_window=(self.rotation_window+[rotation])[-3:]
   if len(self.rotation_window)==3:
    costs=[sum((candidate.inv()*other).magnitude() for other in self.rotation_window) for candidate in self.rotation_window]
    rotation=self.rotation_window[int(np.argmin(costs))]
   delta=(self.rotation.inv()*rotation).as_rotvec()
   rotation_cutoff=3.+.5*min(float(np.linalg.norm(delta))/dt,20.)
   self.rotation=self.rotation*Rotation.from_rotvec(delta*alpha(rotation_cutoff,dt))
   self.raw=x.copy()
  pose['raw_hub_relative_position_m']=v;pose['raw_display_rotation']=r
  pose['hub_relative_position_m']=self.filtered.tolist();pose['display_rotation']=self.rotation.as_matrix().tolist()
  pose['smoothing']={'method':'three-frame median and adaptive position/rotation low-pass','reset':fresh,'minimum_position_cutoff_hz':2.5}
  return pose
 def flip(self):
  if self.last_position is None:raise ValueError('Get a valid position before selecting the other mirror branch.')
  self.branch_uncertain=False
  self.last_position=(-np.asarray(self.last_position)).tolist()
  if self.raw is not None:self.raw=-self.raw;self.filtered=-self.filtered;self.velocity=-self.velocity;self.window=[-v for v in self.window]

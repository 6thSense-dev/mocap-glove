"""Experimental blind calibration of concentric orthogonal TX/RX dipoles.
Free poses constrain corrected matrix singular values to 2:1:1. Learn positive
per-entry gains and stationary electrical phase; do not fit arbitrary coil
misalignment, finite windings, metal distortion or world axis labels. Scale and
coordinate sign gauges are not observable from unmeasured movement alone.
"""
import numpy as np
from scipy.optimize import least_squares
from pose_model import solve_pose

def unpack(c):
 a=np.asarray(c,float)
 if a.shape[-3:]!=(3,3,2) or not np.isfinite(a).all():raise ValueError('Invalid complex measurements.')
 return a[...,0]+1j*a[...,1]

def nearest(m):
 u,s,v=np.linalg.svd(m)
 amplitude=(2*s[...,0]+s[...,1]+s[...,2])/6
 predicted=(u*np.stack([2*amplitude,amplitude,amplitude],axis=-1)[...,None,:])@v
 return predicted

def gains(x):
 v=np.r_[x,-np.sum(x)]
 return np.exp(v.reshape(3,3))

def fit_motion(complex_samples, anchor_samples=None, anchor_distances=None):
 c=unpack(complex_samples)
 if c.ndim!=3 or len(c)<24:raise ValueError('At least 24 valid movement captures are needed.')
 # Fit electrical phase from training frames only. C squared removes magnetic sign.
 train=np.arange(len(c))%4!=0;test=~train
 phase=np.angle(np.sum(c[train]**2,axis=0))/2
 row_phase=np.angle(np.sum(c[train]**2,axis=(0,2)))/2
 phase+=np.pi*np.round((row_phase[:,None]-phase)/np.pi)
 rotated=c*np.exp(-1j*phase)
 imaginary=float(np.linalg.norm(rotated.imag)/np.linalg.norm(rotated.real))
 if imaginary>.12:raise ValueError(f'Phase is inconsistent across motion ({imaginary:.0%}). Check interference or wiring.')
 y=rotated.real
 if np.min(np.linalg.norm(y,axis=2))<5:raise ValueError('Some captures have a transmitter signal below the useful threshold.')
 # det sign is a coordinate convention. It is not a physical winding identification.
 parity=1 if np.median(np.linalg.det(y[train]))>=0 else -1
 y[:,2,:]*=parity
 norms=np.linalg.norm(y,axis=(1,2));normalized=y/norms[:,None,None]
 def residual(x,data):
  m=data/gains(x)
  return ((m-nearest(m))/np.linalg.norm(m,axis=(1,2))[:,None,None]).ravel()
 fits=[least_squares(residual,x,args=(normalized[train],),bounds=(-1.8,1.8),max_nfev=180,ftol=1e-9,xtol=1e-9,gtol=1e-9) for x in (np.zeros(8),np.random.default_rng(62).normal(0,.15,8))]
 fits.sort(key=lambda f:np.linalg.norm(f.fun));best=fits[0]
 singular=np.linalg.svd(best.jac,compute_uv=False)
 if not best.success or singular[-1]<singular[0]*.002:raise ValueError('Movement does not constrain the calibration. Include more tilts, turns and movement in different directions.')
 g=gains(best.x)
 if g.max()/g.min()>30:raise ValueError('Implausible fitted gains; calibration rejected.')
 if np.max(abs(np.log(g/gains(fits[1].x))))>.12:raise ValueError('Different fits disagree; more varied movement is needed.')
 err=np.linalg.norm(residual(best.x,normalized[test]).reshape(-1,9),axis=1)
 if np.sqrt(np.mean(err**2))>.08 or np.max(err)>.20:raise ValueError('Unseen movement does not fit the ideal-coil model; no pose calibration accepted.')
 m=y/g
 if np.mean(np.linalg.det(m)>0)<.95:raise ValueError('Inconsistent matrix handedness; weak signals or interference may be present.')
 # The first sample selects only a local branch, not known room/hand coordinates.
 _,v=np.linalg.eigh(m[0]@m[0].T);hemisphere=v[:,-1]
 if hemisphere[np.argmax(abs(hemisphere))]<0:hemisphere=-hemisphere
 result={'gains':g.tolist(),'phase':phase.tolist(),'parity':parity,'hemisphere':hemisphere.tolist(),
         'held_out_rms':float(np.sqrt(np.mean(err**2))),'held_out_worst':float(np.max(err)),
         'phase_residual':imaginary,'sample_count':len(c),'units':'relative','metres_per_unit':None,
         'frame':'transmitter electrical frame; physical axis names and mirror branch unverified','accuracy_verified':False}
 if anchor_samples is not None and len(anchor_samples)>=2:
  ac=unpack(anchor_samples)*np.exp(-1j*phase);am=ac.real/g;am[:,2,:]*=parity
  if np.linalg.norm(ac.imag)/np.linalg.norm(ac.real)>.12:raise ValueError('Saved references have incompatible phase.')
  ar=np.linalg.norm(am-nearest(am),axis=(1,2))/np.linalg.norm(am,axis=(1,2))
  if np.max(ar)>.15:raise ValueError('Saved references do not fit this motion calibration.')
  relative_range=(np.sqrt(np.sum(am**2,axis=(1,2))/6))**(-1/3)
  scales=np.array(anchor_distances,float)/relative_range
  disagreement=float(np.max(abs(scales/np.median(scales)-1)))
  result['anchor_scale_disagreement']=disagreement
  if disagreement<=.15:
   result['metres_per_unit']=float(np.median(scales));result['units']='approximate metres from two user references'
  else:result['scale_note']='Two saved distances disagree; only relative units are available.'
 # Reference orientation; corrected M remains in arbitrary electrical coordinates.
 p=solve_pose(m[0],phase_referenced=True,hemisphere=hemisphere,max_residual=.2)
 result['zero_rotation']=p['rotation_receiver_to_transmitter']
 return result

def motion_pose(calibration,complex_counts,previous_position=None):
 c=unpack(complex_counts)*np.exp(-1j*np.array(calibration['phase']))
 if np.linalg.norm(c.imag)/max(np.linalg.norm(c.real),1e-12)>.12:raise ValueError('Phase inconsistency; pose unavailable.')
 m=c.real/np.array(calibration['gains']);m[2]*=calibration['parity']
 p=solve_pose(m,phase_referenced=True,hemisphere=calibration['hemisphere'],max_residual=.1,previous_position=previous_position)
 p['position']=p.pop('position_m');p['units']=calibration['units'];scale=calibration['metres_per_unit']
 if scale is not None:p['position']=(np.array(p['position'])*scale).tolist()
 p['rotation_relative_to_start']=(np.array(p['rotation_receiver_to_transmitter'])@np.array(calibration['zero_rotation']).T).tolist()
 p['accuracy_verified']=False;p['frame']=calibration['frame']
 return p

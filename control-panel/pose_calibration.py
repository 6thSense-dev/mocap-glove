"""Experimental measured calibration for three source and receiver axes.
All teaching poses keep the receiver board in the SAME orientation, defined as
identity. Coordinates are relative to the transmitter center, in metres.
Complex per-tone receiver matrices absorb receiver axis alignment, polarity,
gain and phase. Source permutation is selected using held-out prediction.
Requires independent held-out poses. No hardware accuracy guarantee.
"""
import itertools
import numpy as np
from pose_model import solve_pose

def fields(position):
 p=np.asarray(position,float);r=np.linalg.norm(p)
 if p.shape!=(3,) or not np.isfinite(p).all() or not .04<=r<=.30:raise ValueError('Reference center distance must be 40–300 mm.')
 u=p/r
 return (3*np.outer(u,u)-np.eye(3))/r**3

def calibrate(samples):
 if len(samples)<8:raise ValueError('Collect eight distinct reference positions with the receiver orientation unchanged.')
 points=np.array([s['position_m'] for s in samples]);f=np.array([fields(p) for p in points])
 if np.min(np.linalg.norm(points[:,None]-points[None,:]+np.eye(len(points))[:,:,None],axis=2))<.005:raise ValueError('Reference positions must be distinct.')
 y=np.array([s['complex_counts'] for s in samples],float)
 if y.shape!=(len(samples),3,3,2) or not np.isfinite(y).all():raise ValueError('Invalid reference measurements.')
 y=y[...,0]+1j*y[...,1]
 if np.min(np.linalg.norm(y,axis=2))<5:raise ValueError('A transmitter signal is too weak at a reference position.')
 candidates=[]
 for perm in itertools.permutations(range(3)):
  errors=[];transforms=[];valid=True
  for t,physical in enumerate(perm):
   x=f[:,physical,:]
   if np.linalg.cond(x)>20:valid=False;break
   h=np.linalg.lstsq(x,y[:,t,:],rcond=None)[0]
   if np.linalg.cond(h)>30:valid=False;break
   transforms.append(h)
   for i in range(len(samples)):
    keep=np.arange(len(samples))!=i
    if np.linalg.cond(x[keep])>30:valid=False;break
    hi=np.linalg.lstsq(x[keep],y[keep,t,:],rcond=None)[0]
    errors.append(float(np.linalg.norm(x[i]@hi-y[i,t])/np.linalg.norm(y[i,t])))
  if valid:candidates.append((float(np.sqrt(np.mean(np.square(errors)))),perm,transforms,max(errors)))
 if not candidates:raise ValueError('Reference geometry or receiver responses are poorly conditioned. Use more varied positions.')
 candidates.sort(key=lambda x:x[0]);score,perm,transforms,worst=candidates[0]
 if score>.08 or worst>.15:raise ValueError(f'Reference poses do not fit: held-out RMS {score:.1%}, worst {worst:.1%}. Check orientation and measurements.')
 if len(candidates)>1 and candidates[1][0]<max(score*2,.03):raise ValueError('Transmitter axis order is ambiguous. Add varied reference positions.')
 return {'hemisphere':(points[0]/np.linalg.norm(points[0])).tolist(),'permutation':list(perm),'transforms':[[[[float(v.real),float(v.imag)] for v in row] for row in h] for h in transforms], 'held_out_rms':score,'held_out_worst':worst,'sample_count':len(samples),'model':'experimental ideal dipoles; receiver teaching orientation defines zero rotation'}

def track(calibration,complex_counts):
 c=np.asarray(complex_counts,float)
 if c.shape!=(3,3,2) or not np.isfinite(c).all():raise ValueError('Invalid live measurements.')
 c=c[...,0]+1j*c[...,1];m=np.zeros((3,3),complex)
 for t,physical in enumerate(calibration['permutation']):
  h=np.array(calibration['transforms'][t]);h=h[...,0]+1j*h[...,1]
  m[physical]=np.linalg.solve(h.T,c[t])
 imaginary=float(np.linalg.norm(m.imag)/max(np.linalg.norm(m.real),1e-12))
 if imaginary>.1:raise ValueError('Electrical phase differs from calibration; pose rejected.')
 result=solve_pose(m.real,phase_referenced=True,max_residual=.08,hemisphere=calibration.get('hemisphere',(0,0,1)))
 result['phase_residual']=imaginary
 result['accuracy_verified']=False
 return result

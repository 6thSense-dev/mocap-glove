"""Experimental display registration; never estimates hub translation from IMU."""
import numpy as np
from motion_calibration import motion_pose

def register_to_references(cal,complex_loader):
 anchors=cal.get('anchors',[])
 if len(anchors)<2 or cal.get('metres_per_unit') is None:return None
 estimated=[];known=[]
 for a in anchors:
  p=motion_pose(cal,complex_loader(a['file']))['position'];estimated.append(p);known.append(a['position_m'])
 x=np.array(estimated);y=np.array(known);xu=x/np.linalg.norm(x,axis=1)[:,None];yu=y/np.linalg.norm(y,axis=1)[:,None]
 if np.linalg.norm(np.cross(yu[0],yu[1]))<.15:return None
 u,_,v=np.linalg.svd(yu.T@xu);w=u@np.diag([1,1,np.linalg.det(u@v)])@v
 angles=np.degrees(np.arccos(np.clip(np.sum((xu@w.T)*yu,axis=1),-1,1)))
 if np.max(angles)>20:return None
 return {'rotation':w.tolist(),'reference_angle_rms_deg':float(np.sqrt(np.mean(angles**2))),
         'reference_position_rms_mm':float(np.sqrt(np.mean(np.sum((x@w.T-y)**2,axis=1)))*1000),
         'status':'approximate alignment to user references; physical axes and mirror branch unverified'}

def display_pose(cal,registration,counts,previous_position=None):
 p=motion_pose(cal,counts,previous_position=previous_position)
 if registration:
  w=np.array(registration['rotation']);p['hub_relative_position_m']=(w@np.array(p['position'])).tolist()
  p['display_rotation']=(w@np.array(p['rotation_relative_to_start'])@w.T).tolist()
 else:p['hub_relative_position_m']=None;p['display_rotation']=None
 p['registration']=registration
 return p


def signal_summary(spectrum):
 """Always show measured tone strengths; no invented direction or rotation."""
 values=np.asarray([t['xyz_counts'] for t in spectrum['amplitudes']],float)
 strengths=np.linalg.norm(values,axis=1)
 return {'tone_strength_counts':strengths.tolist(),'total_counts':float(np.linalg.norm(values)),
         'clipped':bool(spectrum['clipped']),'position_valid':False}


def align_acquisition_delay(cal, counts):
 """One bounded common time offset, not independent fitted channel phases.

 DRDY is hardware-synchronous but ISR timestamps can lag between SPI buses.
 Squared quadrature error is insensitive to legitimate dipole sign changes.
 +/-20 us keeps the search below half a 24 kHz sample and preserves polarity
 under the shared electrical-phase model. Never synthesize a missing reference.
 """
 from scipy.optimize import minimize_scalar
 from motion_calibration import unpack
 z=unpack(counts)
 if z.shape!=(3,3):raise ValueError('Expected three transmitter and receiver axes.')
 if np.min(np.linalg.norm(z,axis=1))<5:raise ValueError('Transmitter signal too weak for automatic timing.')
 phase=np.asarray(cal['phase']);frequencies=np.array([4500,5625,6750])[:,None]
 reference=z*np.exp(-1j*phase);energy=float(np.sum(abs(reference)**2))
 def loss(us):return float(np.sum((reference*np.exp(-2j*np.pi*frequencies*us*1e-6)).imag**2)/energy)
 fit=minimize_scalar(loss,bounds=(-20.,20.),method='bounded',options={'xatol':1e-5})
 if not fit.success or abs(fit.x)>19.5:raise ValueError('Acquisition timing outside supported range.')
 residual=float(np.sqrt(fit.fun))
 if residual>.10:raise ValueError('Signal phases do not match one acquisition delay.')
 corrected=z*np.exp(-2j*np.pi*frequencies*fit.x*1e-6)
 return np.stack([corrected.real,corrected.imag],axis=-1).tolist(),{'delay_us':float(fit.x),'quadrature_residual':residual,'method':'bounded common acquisition delay; shared electrical phase'}


def automatic_pose(cal,registration,counts,previous_position=None):
 corrected,timing=align_acquisition_delay(cal,counts)
 pose=display_pose(cal,registration,corrected,previous_position=previous_position)
 pose['automatic_timing']=timing
 return pose

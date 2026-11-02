"""Host reference: calibrated signed dipole matrix -> relative fingertip pose.

Not connected to live diagnostics: capture-local phase is insufficient for this
input contract. Rows must already be mapped to physical transmitter X,Y,Z,
columns to a right-handed receiver frame, and all electrical gain/phase removed.
The matrix has units m^-3: M=(3 uu.T-I) R / r^3. R maps receiver to transmitter.
Co-located ideal dipoles only; finite winding geometry and metal cause model error.
"""
import numpy as np

def solve_pose(matrix, *, phase_referenced=False, hemisphere=(0,0,1), max_residual=.05, previous_position=None):
    if not phase_referenced:
        raise ValueError('A calibrated transmitter-referenced signed matrix is required.')
    m=np.asarray(matrix,dtype=float)
    h=np.asarray(hemisphere,dtype=float)
    if m.shape!=(3,3) or not np.isfinite(m).all() or h.shape!=(3,) or not np.isfinite(h).all() or np.linalg.norm(h)==0:
        raise ValueError('Invalid matrix or hemisphere.')
    h=h/np.linalg.norm(h)
    values,vectors=np.linalg.eigh(m@m.T)
    if values[0]<=0 or values[-1]<=0:raise ValueError('Missing independent coil measurements.')
    scale=np.sqrt(np.sum(m*m)/6)
    u=vectors[:,-1]
    branch='initial hemisphere assumption'
    if previous_position is not None:
        prior=np.asarray(previous_position,float)
        if prior.shape!=(3,) or not np.isfinite(prior).all() or np.linalg.norm(prior)==0:raise ValueError('Invalid previous position.')
        prior=prior/np.linalg.norm(prior)
        if abs(u@prior)<.05:raise ValueError('Motion branch ambiguous after a large direction change.')
        if u@prior<0:u=-u
        branch='continuous from previous accepted position'
    elif abs(u@h)<.05:
        if u[np.argmax(abs(u))]<0:u=-u
        branch='initial mirror branch unresolved near reference plane'
    elif u@h<0:u=-u
    d=3*np.outer(u,u)-np.eye(3)
    approximate=np.linalg.solve(d,m/scale)
    a,_,bt=np.linalg.svd(approximate)
    rotation=a@bt
    if np.linalg.det(rotation)<0:raise ValueError('Incorrect winding signs or a reflected receiver frame.')
    residual=float(np.linalg.norm(m-scale*d@rotation)/np.linalg.norm(m))
    if residual>max_residual:raise ValueError('Measurements do not fit the calibrated dipole model.')
    r=scale**(-1/3)
    return {'position_m':(r*u).tolist(),'rotation_receiver_to_transmitter':rotation.tolist(),
            'relative_residual':residual,'model':'ideal dipole; unverified on hardware',
            'hemisphere_assumed':h.tolist(),'branch_status':branch}

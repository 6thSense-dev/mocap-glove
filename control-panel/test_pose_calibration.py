import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from pose_calibration import fields,calibrate,track

def pack(x):return np.stack((x.real,x.imag),axis=-1).tolist()
class CalTests(unittest.TestCase):
 def fixture(self):
  rng=np.random.default_rng(55);perm=[2,0,1]
  h=[Rotation.random(random_state=rng).as_matrix()@np.diag([.12,.09,.15])*np.exp(1j*phase) for phase in [.3,-1.2,2.1]]
  points=[[.06,.02,.08],[-.06,.04,.09],[.02,-.06,.10],[-.04,-.04,.07],[.07,.06,.09],[-.07,-.06,.12],[.03,.05,.13],[-.02,.01,.06]]
  samples=[{'position_m':p,'complex_counts':pack(np.array([fields(p)[k]@h[t] for t,k in enumerate(perm)]))} for p in points]
  return perm,h,samples
 def test_swapped_sources_receiver_rotation_and_polarity(self):
  perm,h,samples=self.fixture();c=calibrate(samples);self.assertEqual(c['permutation'],perm)
  rng=np.random.default_rng(4)
  for _ in range(20):
   p=rng.uniform(-.05,.05,3);p[2]=.1;r=Rotation.random(random_state=rng).as_matrix()
   y=np.array([(fields(p)@r)[k]@h[t] for t,k in enumerate(perm)])
   result=track(c,pack(y));np.testing.assert_allclose(result['position_m'],p,atol=1e-10);np.testing.assert_allclose(result['rotation_receiver_to_transmitter'],r,atol=1e-10)
 def test_insufficient_and_duplicate_poses_rejected(self):
  _,_,s=self.fixture()
  for samples in [s[:3],[s[0]]*8]:
   with self.assertRaises(ValueError):calibrate(samples)
 def test_forward_workspace_below_transmitter(self):
  from pose_model import solve_pose
  p=np.array([0,.165,-.015]);m=fields(p)
  result=solve_pose(m,phase_referenced=True,hemisphere=p)
  np.testing.assert_allclose(result['position_m'],p,atol=1e-12)
 def test_phase_drift_rejected(self):
  _,_,s=self.fixture();c=calibrate(s);y=np.array(s[0]['complex_counts']);z=(y[...,0]+1j*y[...,1])*1j
  with self.assertRaises(ValueError):track(c,pack(z))
if __name__=='__main__':unittest.main()
